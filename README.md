# go2Reachability

.obj files describing the kinematic constraints of the go2 robot.

---

## Installation

1. **Build and install using CMake**:

```bash
mkdir build
cd build
cmake ..
cmake --install . --prefix /desired/install/path

    This will install:

        .obj files in share/go2Reachability/reachability_constraints

        CMake config: share/go2Reachability/cmake/go2ReachabilityConfig.cmake

        Python config: share/go2Reachability/go2ReachabilityConfig.py

    Add Python config to your PYTHONPATH:

export PYTHONPATH=/desired/install/path/share/go2Reachability:$PYTHONPATH

This allows Python scripts to import go2ReachabilityConfig and access the .obj files.
Usage
Install example robot data

conda install example-robot-data scipy -c conda-forge

Generate reduced .obj files using Blender 4.3

    Make the script executable:

chmod +x script/gen_and_copy.sh

    Run the script with the path to your Blender 4.3 executable:

./script/gen_and_copy.sh /path/to/blender-4.3

This will generate reduced .obj files from the original kinematic constraint models.
Using the files in Python

from go2ReachabilityConfig import GO2REACHABILITY_CONSTRAINTS_DIR
import os, glob

obj_files = glob.glob(os.path.join(GO2REACHABILITY_CONSTRAINTS_DIR, "*.obj"))
print("Found OBJ files:", obj_files)

Notes

    The project is portable: .obj files are installed relative to the prefix defined in CMake (CMAKE_INSTALL_PREFIX).

    Compatible with both C++ (via find_package(go2Reachability)) and Python.

    Blender 4.3 is required for generating reduced .obj files.
