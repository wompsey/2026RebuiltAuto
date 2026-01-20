import importlib.util
import sys
from pathlib import Path

from subsystems.climber.io import ClimberIO, ClimberIOTalonFX, ClimberIOSim

# Import ClimberSubsystem from hyphenated filename
_climber_subsystem_path = Path(__file__).parent / "climber-subsystem.py"
spec = importlib.util.spec_from_file_location("climber_subsystem", _climber_subsystem_path)
_climber_subsystem_module = importlib.util.module_from_spec(spec)
sys.modules["climber_subsystem"] = _climber_subsystem_module
spec.loader.exec_module(_climber_subsystem_module)
ClimberSubsystem = _climber_subsystem_module.ClimberSubsystem

__all__ = ["ClimberIO", "ClimberIOTalonFX", "ClimberIOSim", "ClimberSubsystem"]
