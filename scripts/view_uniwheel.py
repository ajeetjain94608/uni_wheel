#!/usr/bin/env python3
"""Open the Uni Wheel MJCF model in the interactive MuJoCo viewer.

Usage:
    python3 scripts/view_uniwheel.py [path/to/model.xml]

If no path is given, defaults to the converted uniwheel.xml under
workspace/mechanical/urdf/uniwheel/.
"""

import sys
from pathlib import Path

import mujoco
import mujoco.viewer

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MODEL = REPO_ROOT / "workspace" / "mechanical" / "urdf" / "uniwheel" / "uniwheel.xml"


def main() -> None:
    model_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_MODEL
    if not model_path.exists():
        sys.exit(f"Model file not found: {model_path}")

    print(f"Loading model: {model_path}")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
