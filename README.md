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
```

This will install:

- .obj files in share/go2Reachability/reachability_constraints

- CMake config: share/go2Reachability/cmake/go2ReachabilityConfig.cmake

- Python config: share/go2Reachability/go2ReachabilityConfig.py

2. **Add Python config to your PYTHONPATH**:

```bash
export PYTHONPATH=/desired/install/path/share/go2Reachability:$PYTHONPATH
```

This allows Python scripts to import go2ReachabilityConfig and access the .obj files.
Usage
Install example robot data

## Regenerate the obj files

This requires example-robot-data and Blender 4.3

```bash
conda install example-robot-data scipy -c conda-forge
```

1. **Make the script script executable**:
```bash
chmod +x script/gen_and_copy.sh
```


2.  **Run the script with the path to your Blender 4.3 executable**:
```bash
./script/gen_and_copy.sh /path/to/blender-4.3
```

This will generate reduced .obj files from the original kinematic constraint models.

## Usage
1. **Using the files in Python**: 
```python
from go2ReachabilityConfig import GO2REACHABILITY_CONSTRAINTS_DIR
import os, glob

obj_files = glob.glob(os.path.join(GO2REACHABILITY_CONSTRAINTS_DIR, "*.obj"))
print("Found OBJ files:", obj_files)
```

2. **Using the files in C++**:
CMakeLists.txt example
```cmake
find_package(go2Reachability REQUIRED PATHS /desired/install/path/share/go2Reachability/cmake)

message(STATUS "Reachability OBJ dir = ${GO2REACHABILITY_CONSTRAINTS_DIR}")

add_executable(test_find main.cpp)

target_compile_definitions(test_find PRIVATE
    GO2REACHABILITY_CONSTRAINTS_DIR="${GO2REACHABILITY_CONSTRAINTS_DIR}")
```
cpp file
```c++
// main.cpp
#include <iostream>
#include <filesystem>

int main() {
    std::string obj_dir = GO2REACHABILITY_CONSTRAINTS_DIR;

    std::cout << "Reachability OBJ dir: " << obj_dir << std::endl;

    for (const auto &entry : std::filesystem::directory_iterator(obj_dir)) {
        std::cout << "  - " << entry.path() << std::endl;
    }

    return 0;
}
```
