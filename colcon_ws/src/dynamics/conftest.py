"""Pytest configuration for dynamics package.

This file ensures the package can be imported without installation.
"""

import sys
from pathlib import Path
import importlib.util

# Add the src directory to the path so 'dynamics' package can be imported
package_root = Path(__file__).parent
src_dir = package_root / "src"

# Always reload to pick up changes
if "dynamics" in sys.modules:
    del sys.modules["dynamics"]
    # Also remove submodules
    to_remove = [k for k in sys.modules.keys() if k.startswith("dynamics.")]
    for k in to_remove:
        del sys.modules[k]

sys.path.insert(0, str(package_root))
# Make 'src' importable as 'dynamics'
spec = importlib.util.spec_from_file_location("dynamics", src_dir / "__init__.py",
                                                submodule_search_locations=[str(src_dir)])
dynamics = importlib.util.module_from_spec(spec)
sys.modules["dynamics"] = dynamics
dynamics.__path__ = [str(src_dir)]
spec.loader.exec_module(dynamics)
