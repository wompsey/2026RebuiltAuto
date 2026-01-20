from enum import Enum

from commands2 import Subsystem


class StateSubsystem(Subsystem):
    """
    Base class for subsystems that manage state machines.
    
    Provides state management functionality with validation for state transitions.
    Subclasses should define a SubsystemState Enum and implement state-specific logic.
    """

    def __init__(self, name: str, initial_state: Enum) -> None:
        """
        Initialize the state subsystem.

        :param name: The name of the subsystem (for logging/identification)
        :param initial_state: The initial state for the subsystem
        """
        super().__init__()
        self._name = name
        self._current_state: Enum = initial_state
        self._desired_state: Enum = initial_state

    def set_desired_state(self, desired_state: Enum) -> bool:
        """
        Set the desired state for the subsystem.

        :param desired_state: The state to transition to
        :return: True if the state transition is valid/allowed, False otherwise
        """
        if desired_state == self._current_state:
            return False  # Already in this state
        
        self._desired_state = desired_state
        self._current_state = desired_state
        return True  # State transition allowed by default

    def get_current_state(self) -> Enum:
        """
        Get the current state of the subsystem.

        :return: The current state
        """
        return self._current_state

    def get_desired_state(self) -> Enum:
        """
        Get the desired state of the subsystem.

        :return: The desired state
        """
        return self._desired_state
