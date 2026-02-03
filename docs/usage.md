# Usage

## Build
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

Flash `build-pico/flight_pico.uf2` via USB BOOTSEL.

Default I2C pins in `include/flight/hal/pico_config.h`:
- `SDA = GPIO4`
- `SCL = GPIO5`

## Run Demo
```bash
./build/flight_demo
```

## Doxygen API Docs
```bash
cmake --build build --target docs
```

## Unit Tests
```bash
cmake -S . -B build -DBUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## Coverage
```bash
cmake -S . -B build -DBUILD_TESTS=ON -DENABLE_COVERAGE=ON
cmake --build build
ctest --test-dir build --output-on-failure
gcovr -r . --html-details -o coverage.html
```

## MkDocs
```bash
pip install mkdocs mkdocs-material
mkdocs serve
```

## Read the Docs
Read the Docs will build `mkdocs.yml` by default. Connect the repository and select the default branch. If you need a custom build config, add `.readthedocs.yaml`.
