# JSim Physics Library

[![CI](https://github.com/Yet-Another-Software-/JSim/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/Yet-Another-Software-Suite/JSim/actions/workflows/ci.yml)
[![jsim.dev](https://github.com/Yet-Another-Software-Suite/JSim/actions/workflows/deploy.yml/badge.svg?branch=main)](https://github.com/Yet-Another-Software-Suite/JSim/actions/workflows/deploy-mkdocs.yml)

JSim is an open-source FRC physics library for simulation.

## Overview

This repository is organized as a monorepo containing the core simulation engine, language bindings, runtime apps, examples, and documentation tooling.

### Vendordep URL
```
https://jsim.dev/JSim.json
```

### Build and Test

```bash
./scripts/build-all.sh
```

### Run Tests

Run the vendordep Gradle test task used in CI:

```bash
cd vendordep
./gradlew test
```

This runs Java tests and the native verification suites in `vendordep/tests/`.

Vendordep tooling requires Java 17.

### Preview Docs Locally

```bash
pip install mkdocs mkdocs-material
mkdocs serve --config-file mkdocs/mkdocs.yml
```

## Repository Structure

### Core Libraries

- `core/driver/`: C++ physics engine implementation and headers
- `core/python/`: Python-side native bindings and tests
- `cad-import/`: CAD import utilities and Python package wrapper
- `apps/viewer-plugin/`: visualization and rendering plugin (Python wrapper available)
- `apps/sim-runtime/`: Python runtime integration app (importable wrapper available)

### Applications

- `apps/sim-runtime/`: Python runtime integration app
- `apps/viewer-plugin/`: visualization and rendering plugin

### Tooling and Integration

- `cad-import/`: CAD and geometry import utilities, including JSim-owned snapshot tracking for simulation integrations
- `examples/`: language-specific examples (C++, Java, Python)
- `mkdocs/`: docs source and MkDocs configuration
- `vendordep/`: WPILib vendordep packaging and Gradle-based build/testing

## Documentation

- Docs source: `mkdocs/docs/`
- MkDocs config: `mkdocs/mkdocs.yml`
- Deployed docs: https://jsim.dev
- Java API (Javadocs): https://jsim.dev/api/javadoc/
- Native API (Doxygen): https://jsim.dev/api/doxygen/html/
- Example project

## Contributing

Contributions are welcome. For code and process expectations, please review:

- `CODE_OF_CONDUCT.md`
- open issues and discussions in this repository

## License

This project is licensed under the terms in `LICENSE.txt`.
