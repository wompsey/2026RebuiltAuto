from abc import ABC
from dataclasses import dataclass, field
from typing import Final

from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import PositionVoltage

from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor
from wpimath.system.plant import LinearSystemId
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d

from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from phoenix6.signals import InvertedValue
from pykit.autolog import autolog
from wpimath.units import radians, radians_per_second, volts, amperes

from constants import Constants
from util import tryUntilOk


class HoodIO(ABC):
    """
    Abstract base class for hood IO implementations.
    Provides the interface for both real hardware and simulation.
    """

    @autolog
    @dataclass
    class HoodIOInputs:
        """Inputs from the hood hardware/simulation."""
        # Motor status
        motorConnected: bool = False
        motorPosition: radians = 0.0
        motorAppliedVolts: volts = 0.0
        motorCurrent: amperes = 0.0


    def updateInputs(self, inputs: HoodIOInputs) -> None:
        """Update the inputs with current hardware/simulation state."""
        pass

    def setPosition(self, rotation: Rotation2d) -> None:
        """set rotoation."""
        pass


class HoodIOTalonFX(HoodIO):
    """
    Real hardware implementation using TalonFX motor controller.
    """

    def __init__(self, motor_id: int) -> None:
        """
        Initialize the real hardware IO.

        :param motor_id: CAN ID of the TalonFX motor
        :param motor_config: TalonFX configuration to apply
        """

        self._hoodMotor: Final[TalonFX] = TalonFX(motor_id, "rio")

        motorConfig = TalonFXConfiguration()
        motorConfig.slot0 = Constants.HoodConstants.GAINS
        motorConfig.feedback.sensor_to_mechanism_ratio = Constants.HoodConstants.GEAR_RATIO
        motorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        motorConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE


        # Apply motor configuration
        tryUntilOk(5, lambda: self._hoodMotor.configurator.apply(motorConfig, 0.25))


        # Create status signals for motor
        self._position: Final = self._hoodMotor.get_position()
        self._velocity: Final = self._hoodMotor.get_velocity()
        self._appliedVolts: Final = self._hoodMotor.get_motor_voltage()
        self._current: Final = self._hoodMotor.get_stator_current()
        self._temperature: Final = self._hoodMotor.get_device_temp()

        # Configure update frequencies
        BaseStatusSignal.set_update_frequency_for_all(
            50,
            self._position,
            self._velocity,
            self._appliedVolts,
            self._current,
            self._temperature
        )
        self._hoodMotor.optimize_bus_utilization()

        # Voltage control request
        self._voltageRequest: Final[PositionVoltage] = PositionVoltage(0)

    def updateInputs(self, inputs: HoodIO.HoodIOInputs) -> None:
        """Update inputs with current motor state."""
        # Refresh all motor signals
        motorStatus = BaseStatusSignal.refresh_all(
            self._position,
            self._velocity,
            self._appliedVolts,
            self._current,
            self._temperature
        )

        # Update motor inputs
        inputs.motorConnected = motorStatus.is_ok()
        inputs.motorPosition = self._position.value_as_double
        inputs.motorAppliedVolts = self._appliedVolts.value_as_double
        inputs.motorCurrent = self._current.value_as_double

    def setPosition(self, rotation: Rotation2d) -> None:
        """Set the poistion."""
        self._positionRequest = PositionVoltage
        self._hoodMotor.set_control(self._positionRequest)



class HoodIOSim(HoodIO):
    def __init__(self) -> None:
        gearbox = DCMotor.krakenX44FOC(1)
        self.appliedVolts = 0.0
        self.closedloop = False

        self._hoodSim = DCMotorSim(
            LinearSystemId.DCMotorSystem(gearbox, 0.00783112228, Constants.HoodConstants.GEAR_RATIO),
            gearbox
        )

        self.controla = PIDController(
            Constants.HoodConstants.GAINS.k_p,
            Constants.HoodConstants.GAINS.k_i,
            Constants.HoodConstants.GAINS.k_d
        )


    def updateInputs(self, inputs: HoodIO.HoodIOInputs) -> None:
        if self.closedloop:
            self.appliedVolts = (self.controla.calculate(self._hoodSim.getAngularPosition()))
        else:
            self.controla.reset()

        self._hoodSim.setInputVoltage(max(-12.0, min(self.appliedVolts, 12.0)))

        self._hoodSim.update(0.02)
        inputs.motorConnected = True
        inputs.motorAppliedVolts = self.appliedVolts
        inputs.motorPosition = self._hoodSim.getAngularPosition()
        inputs.motorCurrent = abs(self._hoodSim.getCurrentDraw())


    def setOpenLoop(self, output: volts) -> None:
        self.closedloop = False
        self.appliedVolts = output

    def setPosition(self, rotation: Rotation2d) -> None:
        self.closedloop = True
        self.controla.setSetpoint(rotation)