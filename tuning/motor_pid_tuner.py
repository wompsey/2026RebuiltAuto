"""
PID turning tuner for Phoenix 6 motors on the Elastic Tuning tab.

Loads current slot0 gains from the selected motor into NT fields and applies
changes from those fields to the motor in real time. Only active when
Constants.tuningMode is True.
"""

from typing import Callable, Optional

from ntcore import NetworkTableInstance
from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX

from constants import Constants
from util import LoggedNetworkNumber, tryUntilOk


# NetworkTable key prefix for PID Turn tuning
PID_TURN_PREFIX = "/Tuning/PIDTurn"




class MotorPIDTuner:
    """
    Tunes slot0 PID gains for a set of Phoenix 6 TalonFX motors via the
    Tuning tab. Loads existing gains when the motor selection changes and
    applies field changes to the motor in real time.
    """

    def __init__(
        self,
        motors: list[tuple[str, Callable[[], Optional[TalonFX]]]],
    ) -> None:
        """
        :param motors: List of (display_name, get_motor) where get_motor
            returns the TalonFX for that motor or None if not available (e.g. sim).
        """
        self._motors = motors
        self._motor_names = [name for name, _ in motors]
        self._last_selected: str = ""
        self._last_kp: float = 0.0
        self._last_ki: float = 0.0
        self._last_kd: float = 0.0

        # NT entries (only created in tuning mode)
        self._selected_entry = None
        self._kp_nt: Optional[LoggedNetworkNumber] = None
        self._ki_nt: Optional[LoggedNetworkNumber] = None
        self._kd_nt: Optional[LoggedNetworkNumber] = None
        self._options_pub = None

        if not Constants.tuningMode or not self._motor_names:
            return

        inst = NetworkTableInstance.getDefault()
        self._selected_entry = inst.getStringTopic(f"{PID_TURN_PREFIX}/SelectedMotor").getEntry("")
        self._selected_entry.set(self._motor_names[0])

        # Publish options for ComboBox (array of strings)
        self._options_pub = inst.getStringArrayTopic(f"{PID_TURN_PREFIX}/Options").publish()
        self._options_pub.set(self._motor_names)

        # Gain fields: default 0 so we load from motor on first run
        self._kp_nt = LoggedNetworkNumber(f"{PID_TURN_PREFIX}/kP", 0.0)
        self._ki_nt = LoggedNetworkNumber(f"{PID_TURN_PREFIX}/kI", 0.0)
        self._kd_nt = LoggedNetworkNumber(f"{PID_TURN_PREFIX}/kD", 0.0)

    def _get_motor(self, name: str) -> Optional[TalonFX]:
        """Return the TalonFX for the given display name, or None."""
        for n, get_motor in self._motors:
            if n == name:
                return get_motor()
        return None

    def _load_gains_from_motor(self, motor: TalonFX) -> None:
        """Read slot0 from motor and push kP, kI, kD to NT."""
        cfg = TalonFXConfiguration()
        status = motor.configurator.refresh(cfg)
        if not status.is_ok():
            return
        slot = cfg.slot0
        kp, ki, kd = slot.k_p, slot.k_i, slot.k_d
        self._kp_nt.set(kp)
        self._ki_nt.set(ki)
        self._kd_nt.set(kd)
        self._last_kp, self._last_ki, self._last_kd = kp, ki, kd

    def _apply_gains_to_motor(self, motor: TalonFX, kp: float, ki: float, kd: float) -> None:
        """Apply slot0 P/I/D to motor; refresh full config first to keep kS/kV/kA."""
        cfg = TalonFXConfiguration()
        status = motor.configurator.refresh(cfg)
        if not status.is_ok():
            return
        cfg.slot0.k_p = kp
        cfg.slot0.k_i = ki
        cfg.slot0.k_d = kd
        tryUntilOk(3, lambda: motor.configurator.apply(cfg, 0.25))

    def periodic(self) -> None:
        """Call from robot periodic when tuning mode is enabled."""
        if (
            not Constants.tuningMode
            or not self._motor_names
            or self._selected_entry is None
            or self._kp_nt is None
        ):
            return

        selected = self._selected_entry.get()
        if not selected or selected not in self._motor_names:
            selected = self._motor_names[0]
            self._selected_entry.set(selected)

        motor = self._get_motor(selected)
        if motor is None:
            return

        # When selection changes, load current gains from motor into fields
        if selected != self._last_selected:
            self._last_selected = selected
            self._load_gains_from_motor(motor)
            return

        kp = self._kp_nt.get()
        ki = self._ki_nt.get()
        kd = self._kd_nt.get()

        # Apply to motor when values change
        if (kp, ki, kd) != (self._last_kp, self._last_ki, self._last_kd):
            self._last_kp, self._last_ki, self._last_kd = kp, ki, kd
            self._apply_gains_to_motor(motor, kp, ki, kd)
