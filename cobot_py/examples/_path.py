"""Makes ``cobot`` importable when running an example straight from a clone.

Importing this module puts ``../src`` on ``sys.path``. Harmless once the package
is properly installed with ``pip install -e .`` — the installed copy wins.
"""

import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parents[1] / "src"
if _SRC.is_dir() and str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))
