# Usage

## Build
```bash
cmake -S . -B build
cmake --build build
```

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
