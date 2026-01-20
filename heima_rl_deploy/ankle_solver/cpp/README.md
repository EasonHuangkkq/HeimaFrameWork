# Ankle Solver C++ Implementation

This is a C++ port of the Python `solve_ankle_v3.py` ankle solver.

## Dependencies

- **Eigen3**: Linear algebra library for vectors and matrices
  - Install on Ubuntu/Debian: `sudo apt-get install libeigen3-dev`
  - Install on macOS: `brew install eigen`

## Building

```bash
mkdir build
cd build
cmake ..
make
```

## Running

```bash
./bin/ankle_solver
```

## Key Differences from Python Version

1. **Eigen library**: Used instead of NumPy for vector/matrix operations
2. **Newton-Raphson**: Custom implementation instead of `scipy.optimize.fsolve`
3. **No plotting**: The matplotlib plotting code is replaced with console output

## Structure

- `ankle_solver.h`: Header file with class definition
- `ankle_solver.cpp`: Implementation of the AnkleSolver class
- `main.cpp`: Main function with test code
- `CMakeLists.txt`: CMake build configuration

