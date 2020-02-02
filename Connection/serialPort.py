from DataStructures.makesmithInitFuncs import MakesmithInitFuncs
from Connection.serialPortThread import SerialPortThread

import serial
import threading

class SerialPort(MakesmithInitFuncs):
    """
    The SerialPort is an object which manages communication with the device over the serial port.
    The actual connection is run in a separate thread by an instance of a SerialPortThread object.
    """

    _worker = None
    _serialInstance = None
    _thread = None
    _reconnectTimer = None
    # Whether the port should connect automatically and attempt reconnections.
    _reconnect = True
    # Whether the port is connected successfullly.
    _connected = False

    def __init__(self):
        """
        Runs on creation, schedules the software to attempt to connect to the machine
        """
        # Serial instance in not automatically opened if created without specifiying port.
        self._serialInstance = serial.Serial(baudrate=57600, timeout=0.25)
        self._stop_event = threading.Event()
        self._worker = SerialPortThread(self._serialInstance)
        self._thread = threading.Thread(
            target=self._worker.getmessage, daemon=True, args=(self._stop_event,)
        )
        self._reconnectTimer = threading.Timer(2.0, self.openConnection)
        # schedule.every(5).seconds.do(self.openConnection)

    def setUpData(self, data):
        self.data = data
        self._worker.data = data

    def openConnection(self, reconnect=True):
        """
        Opens the serial port and starts the threaded worker.
        """

        # Check for serial version being > 3
        if float(serial.VERSION[0]) < 3:
            self.data.ui_queue1.put(
                "Alert",
                "Incompability Detected",
                "Pyserial version 3.x is needed, but version "
                + serial.VERSION
                + " is installed",
            )

        self._reconnect = reconnect

        # Set com port every time as it could have changed.
        self.data.comport = self.data.config.getValue("Maslow Settings", "COMport")
        self._serialInstance.port = self.data.comport

        if not self._serialInstance.is_open:
            try:
                connectMessage = (
                    "Trying to connect to controller on " + self.data.comport
                )
                self.data.console_queue.put(connectMessage)
                self._serialInstance.open()
            except Exception as e:
                self.data.console_queue.put("Error opening serial port: " + str(e))

        if self._serialInstance.is_open:
            self._connected = True
            self.data.currentTool = 0  # This is necessary since the controller will have reset tool to zero.
            self.data.console_queue.put(
                "\r\nConnected on port " + self.data.comport + "\r\n"
            )
            self.data.ui_queue1.put(
                "TextMessage", "", "Connected on port " + self.data.comport
            )
            self.data.ui_queue1.put(
                "Action",
                "connectionStatus",
                {
                    "status": "connected",
                    "port": self.data.comport,
                    "fakeServoStatus": self.data.fakeServoStatus,
                },
            )

            if self._thread is None:
                self._stop_event = threading.Event()
                self._thread = threading.Thread(
                    target=self._worker.getmessage,
                    daemon=True,
                    args=(self._stop_event,),
                )

            self._thread.start()

            # Stop the reconnect timer event.
            if self._reconnectTimer.is_alive():
                self._reconnectTimer.cancel()

            self._getFirmwareVersion()
            self._setupMachineUnits()
            self._requestSettingsUpdate()
        else:
            # Start the timer to attempt reconnects
            if self._reconnect and not self._reconnectTimer.is_alive():
                self._reconnectTimer.start()

            print("Serial connection failed to open.")
            self._connected = False
            self.data.uploadFlag = 0
            self.data.ui_queue1.put(
                "Action",
                "connectionStatus",
                {
                    "status": "disconnected",
                    "port": "none",
                    "fakeServoStatus": self.data.fakeServoStatus,
                },
            )

    def closeConnection(self, reconnect=True):
        if self._serialInstance is not None and self._serialInstance.is_open:
            # Stop the worker thread
            self._stop_event.set()
            # Close the connection
            self._serialInstance.close()
            self._thread = None
            print("Serial connection closed.")
            self.data.ui_queue1.put(
                "Action",
                "connectionStatus",
                {
                    "status": "disconnected",
                    "port": "none",
                    "fakeServoStatus": self.data.fakeServoStatus,
                },
            )
        else:
            print("Serial Instance is none??")

        self.data.uploadFlag = 0
        self._connected = False
        self._reconnect = reconnect
        return

    def resetConnection(self):
        self.closeConnection()
        self.openConnection()

    def connectionStatus(self):
        return self._connected

    def _getFirmwareVersion(self):
        """
        Send command to have controller report details
        :return:
        """
        self.data.gcode_queue.put("B05 ")

    def _setupMachineUnits(self):
        """
        Send command to put controller in correct units state.
        :return:
        """
        if self.data.units == "INCHES":
            self.data.gcode_queue.put("G20 ")
        else:
            self.data.gcode_queue.put("G21 ")

    def _requestSettingsUpdate(self):
        """
        Send command to have controller report settings
        :return:
        """
        self.data.gcode_queue.put("$$")

    def processReadException(self, exception):
        self.closeConnection()
        self.data.console_queue.put("Serial port read failed.")
        self.data.logger.writeToLog("Serial port read failed.")
        self.data.ui_queue1.put("SendAlarm", "Alarm: Serial read failed", "")

        #if self._reconnect and not self._reconnectTimer.is_alive():
            #self._reconnectTimer.start()

    def processWriteException(self, exception, message):
        self.closeConnection()
        self.data.console_queue.put("Serial port write failed.")
        self.data.logger.writeToLog(
            "Serial port write failed: " + str(message.decode())
        )
        if self.data.uploadFlag > 0:
            self.data.ui_queue1.put(
                "Alert",
                "Connection Lost",
                "Message: USB connection lost. This has likely caused the machine to loose it's calibration, which can cause erratic behavior. It is recommended to stop the program, remove the sled, and perform the chain calibration process. Press Continue to override and proceed with the cut.",
            )
        else:
            self.data.ui_queue1.put("SendAlarm", "Alarm: Connection Failed", "")

        self.data.ui_queue1.put(
            "Action",
            "connectionStatus",
            {
                "status": "disconnected",
                "port": "none",
                "fakeServoStatus": self.data.fakeServoStatus,
            },
        )

    def clearAlarm(self):
        if not self._connected:
            self.openConnection()
