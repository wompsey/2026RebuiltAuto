from typing import Callable
from subsystems import Subsystem
from subsystems.hood.io import HoodIO
from wpilib import Alert
from wpimath.geometry import Pose2d, Rotation2d
from pathplannerlib.auto import AutoBuilder, FlippingUtil
from math import atan, sqrt

from pykit.logger import Logger
from wpimath.filter import Debouncer

class HoodSubsystem(Subsystem):
    def __init__(self, io: HoodIO, robot_pose_supplier: Callable[[], Pose2d]):
        super().__init__()

        self.io = io

        self.robot_pose_supplier = robot_pose_supplier()

        self.inputs = HoodIO.HoodIOInputs()
        self._motorDisconnectedAlert = Alert("Hood motor is disconnected.", Alert.AlertType.kError)

        self.atSetPointDebounce = Debouncer(0.1, Debouncer.DebounceType.kRising)

        self.hub_pose = Pose2d(4.625594, 4.034536, 0.0)
        self.distance = self.robot_pose_supplier.translation().distance(self.hub_pose.translation())
        self.launch_speed =  7.62 # meters per second
        self.height = 1.3860018 # hub height - initial height of shooter (17.433 inches) (in meters)

    def calculate_angle(self):
        self.angle = atan(self.launch_speed**2+sqrt(self.launch_speed**4 - 9.80665*(9.80665*self.distance**2+2*self.height*self.launch_speed**2)/9.80665*self.distance))
        return self.angle

    def periodic(self):
        self.io.updateInputs(self.inputs)
        """this below may not work because it passes in a float instead of a rotation, will get that fixed soon."""
        self.io.setPosition(self.calculate_angle()/360) # convert degrees to rotations,
    
        Logger.processInputs("Hood", self.inputs)

        self._motorDisconnectedAlert.set(not self.inputs.motorConnected)

        if AutoBuilder.shouldFlip():
            self.hub_pose = FlippingUtil.flipFieldPose(self.hub_pose)