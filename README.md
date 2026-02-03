# MAKeFlight

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

## Documentation
- [Flight Controller Concepts Guide](docs/flight-controller-guide.md)

## Build (Host)
```bash
cmake -S . -B build
cmake --build build
```

## Build (RP2350 Pico SDK)
```bash
export PICO_SDK_PATH=/path/to/pico-sdk
cmake -S . -B build-pico -DBUILD_PICO=ON
cmake --build build-pico
```

Or let CMake fetch the SDK:
```bash
cmake -S . -B build-pico -DBUILD_PICO=ON -DFETCH_PICO_SDK=ON -DPICO_SDK_TAG=2.0.0
cmake --build build-pico
```

If you see toolchain errors, install `gcc-arm-none-eabi` and `ninja` in your environment (already included in the devcontainer).

Flash `build-pico/flight_pico.uf2` via USB BOOTSEL.

## Pico I2C Pins
Default I2C pins in `include/flight/hal/pico_config.h`:
- `SDA = GPIO4`
- `SCL = GPIO5`

Adjust these to match your WeAct Pico2 wiring.
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
