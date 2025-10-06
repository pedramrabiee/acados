# Acados C++ Interface

Modern C++14 wrapper for the Acados C API.

## Overview

This C++ interface provides a RAII-based wrapper around the Acados C library, making it easier to use from C++ code.

## Features

- **RAII resource management**: Automatic cleanup of Acados structures
- **Type-safe API**: C++ types instead of raw pointers
- **Modern C++14**: Uses `std::vector`, exceptions, etc.
- **Minimal overhead**: Thin wrapper over C API

## Current Status

**Work in Progress** - Placeholder implementation

The API is defined but not yet fully implemented. TODOs are marked in the source code.

## Building

The C++ interface is built as part of the main Acados CMake build:

```bash
mkdir build
cd build
cmake ..
make acados_cpp
```

## Usage Example

```cpp
#include <acados_cpp/ocp_nlp.hpp>

// Define problem dimensions
acados::OcpNlpDims dims;
dims.nx = 4;   // 4 states
dims.nu = 2;   // 2 inputs
dims.N = 20;   // 20 shooting intervals

// Create solver
acados::OcpNlpSolver solver(dims);

// Set initial state
std::vector<double> x0 = {0.0, 0.0, 0.0, 0.0};
solver.set_initial_state(x0);

// Solve
int status = solver.solve();

// Get solution
auto u0 = solver.get_control(0);
```

## Integration with 3Laws Planner

This C++ wrapper is designed to integrate with the 3Laws planner module via the `AcadosSolverBridge`.

## License

Same as Acados (2-Clause BSD License)

## Author

Pedram Rabiee (prabiee@3laws.io)
