import importlib.util
import sys
from pathlib import Path

from subsystems.intake.io import IntakeIO, IntakeIOTalonFX, IntakeIOSim

# Import IntakeSubsystem from hyphenated filename
_intake_subsystem_path = Path(__file__).parent / "intake-subsystem.py"
spec = importlib.util.spec_from_file_location("intake_subsystem", _intake_subsystem_path)
_intake_subsystem_module = importlib.util.module_from_spec(spec)
sys.modules["intake_subsystem"] = _intake_subsystem_module
spec.loader.exec_module(_intake_subsystem_module)
IntakeSubsystem = _intake_subsystem_module.IntakeSubsystem

__all__ = ["IntakeIO", "IntakeIOTalonFX", "IntakeIOSim", "IntakeSubsystem"]
