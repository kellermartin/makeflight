# makeflight

Modular C++17 flight controller framework focused on clean interfaces, portability, and composable subsystems. The design starts with RP2350 support and a 4‑thruster ROV reference vehicle, while keeping HAL boundaries clear for additional MCU targets.

## Highlights
- Clear HAL separation (`ITime`, `IFlash`, `II2c`, `ISpi`, `IUart`, `IGpio`).
- Swappable modules for sensors, estimators, controllers, receivers, actuators.
- Scheduler that supports mixed‑rate control loops.
- Vehicle factory for multiple vehicle archetypes.
- Doxygen + MkDocs documentation.

## Repository Layout
- `include/flight/`: Public framework headers.
- `src/`: Implementations.
- `docs/`: MkDocs documentation.
- `Doxyfile`: API doc config.
- `CMakeLists.txt`: Build and docs targets.

## Build (Host)
```bash
cmake -S . -B build
cmake --build build
```

## Generate API Docs (Doxygen)
```bash
cmake --build build --target docs
```

## MkDocs (Local)
```bash
pip install mkdocs mkdocs-material
mkdocs serve
```

## Unit Tests
```bash
cmake -S . -B build -DBUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## Read the Docs (RTD) Publishing
This repository includes configuration for Read the Docs. Connect the repo in RTD and select the default branch.

## GitHub Actions
- `ci.yml`: build, test, and MkDocs build on PRs and pushes.
- `pages.yml`: publish MkDocs to GitHub Pages on `main`.

To enable Pages, set the GitHub Pages source to **GitHub Actions** in repository settings.

## Coverage
```bash
cmake -S . -B build -DBUILD_TESTS=ON -DENABLE_COVERAGE=ON
cmake --build build
ctest --test-dir build --output-on-failure
gcovr -r . --html-details -o coverage.html
```
## Quick ROV Demo
```bash
cmake -S . -B build
cmake --build build
./build/flight_demo
```

## UDP Control Packet
Use the UDP receiver to send normalized channels to port 14550.

```cpp
struct UdpPacket {
  uint32_t magic;      // 0x4D465454 ("MFTT")
  uint8_t version;     // 1
  uint8_t channel_count;
  float channels[16];
};
```

Channel mapping for the ROV:
- `ch0`: surge (forward/back)
- `ch1`: yaw (turn)
- `ch2`: heave (up/down)

## License
TBD
