import sys

if __package__ in (None, ""):
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from SCARA_UI.ui.app_bootstrap import run_app


if __name__ == "__main__":
    sys.exit(run_app())
