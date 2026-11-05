"""Compatibility entry point; implementation lives in src."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import real_robot
from piper_dynamics.pipeline.friction_bins import main
if __name__=='__main__':main()
