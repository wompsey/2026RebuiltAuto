import os.path

from commands2 import CommandScheduler, TimedCommandRobot
from ntcore import NetworkTableInstance
from phoenix6 import SignalLogger
from wpilib import DataLogManager, DriverStation, Timer

from constants import Constants
from lib import elasticlib
from lib.elasticlib import Notification, NotificationLevel
from robot_container import RobotContainer

"""
import cv2

# USB Limelight 3A stream URL (roboRIO reaches it via USB interface)
_LIMELIGHT_USB_STREAM_URL = "http://172.29.0.1:5800/stream.mjpg"


def _limelight_stream_proxy_thread(source) -> None:

    #Pull MJPEG from USB Limelight and push to CameraServer for Elastic Camera Stream widget.
    cap = None
    while True:
        try:
            if cap is None or not cap.isOpened():
                cap = cv2.VideoCapture(_LIMELIGHT_USB_STREAM_URL)
                if not cap.isOpened():
                    if cap is not None:
                        cap.release()
                    cap = None
                    time.sleep(2.0)
                    continue
            ret, frame = cap.read()
            if not ret:
                if cap is not None:
                    cap.release()
                cap = None
                continue
            source.putFrame(frame)
        except Exception:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
                cap = None
            time.sleep(2.0)
"""

class Dwayne(TimedCommandRobot):

    def __init__(self, period = 0.02) -> None:
        super().__init__(period)

        DriverStation.silenceJoystickConnectionWarning(not DriverStation.isFMSAttached())
        self.container = RobotContainer()

        SignalLogger.enable_auto_logging(False)
        SignalLogger.stop()
        DataLogManager.start(period=0.3)
        #DriverStation.startDataLog(DataLogManager.getLog())

        # Limelight 3A on USB: forward roboRIO ports so driver station can reach UI/stream at robotIP:5801 (UI), robotIP:5800 (stream)
        # LimelightHelpers.setup_port_forwarding_usb(0)

        # Proxy: pull Limelight MJPEG from USB (172.29.0.1:5800) and publish to CameraServer so Elastic "Camera Stream" can display it
        """
        try:
            from cscore import CameraServer
            stream_name = Constants.VisionConstants.FRONT  # "limelight-front" to match elastic-layout.json topic
            limelight_source = CameraServer.putVideo(stream_name, 320, 240)
            t = threading.Thread(
                target=_limelight_stream_proxy_thread,
                args=(limelight_source,),
                daemon=True,
                name="LimelightStreamProxy",
            )
            t.start()
        except Exception as e:
            DataLogManager.log(f"Limelight stream proxy skipped: {e}")
        """

        DataLogManager.log("Robot initialized")

        dashboard_nt = NetworkTableInstance.getDefault().getTable("Elastic")
        self._match_time_pub = dashboard_nt.getFloatTopic("Match Time").publish()

    @staticmethod
    def get_deploy_directory():
        if os.path.exists("/home/lvuser"):
            return "/home/lvuser/py/deploy"
        else:
            return os.path.join(os.getcwd(), "deploy")

    def robotPeriodic(self) -> None:
        self._match_time_pub.set(Timer.getMatchTime())

    def _simulationPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        DataLogManager.log("Autonomous period started")

        selected_auto = self.container.get_autonomous_command()
        if selected_auto is not None:
            DataLogManager.log(f"Selected Auto: {selected_auto.getName()}")
            selected_auto.schedule()

        elasticlib.select_tab("Autonomous")
            
    def autonomousPeriodic(self) -> None:
        pass
    
    def autonomousExit(self) -> None:
        DataLogManager.log("Autonomous period ended")
        elasticlib.select_tab("Teleop")
            
    def teleopInit(self) -> None:
        DataLogManager.log("Teleoperated period started")

    def teleopExit(self) -> None:
        DataLogManager.log("Teleoperated period ended")
        if DriverStation.isFMSAttached():
            elasticlib.send_notification(
                Notification(
                    level=NotificationLevel.INFO.value,
                    title="Good match!",
                    description="(again)" if DriverStation.getReplayNumber() > 1 else ""
                )
            )

    def testInit(self):
        DataLogManager.log("Test period started")
        CommandScheduler.getInstance().cancelAll()
        elasticlib.select_tab("Debug")
        SignalLogger.start()

    def disabledInit(self):
        if self.container.vision is not None:
            self.container.vision.set_throttle(150)

    def disabledExit(self):
        if self.container.vision is not None:
            self.container.vision.set_throttle(0)

    def testExit(self):
        DataLogManager.log("Test period ended")
    
    def disabledPeriodic(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        pass

    def testPeriodic(self) -> None:
        pass