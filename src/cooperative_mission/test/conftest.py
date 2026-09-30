import os
import sys

# Make the package and the shared simulation helper importable when pytest is
# run directly from the source tree (colcon test also works without this).
HERE = os.path.dirname(os.path.abspath(__file__))
for path in (HERE, os.path.dirname(HERE)):
    if path not in sys.path:
        sys.path.insert(0, path)
