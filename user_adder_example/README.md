# User Adder Caravel Example

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![UPRJ_CI](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml/badge.svg)](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml) [![Caravel Build](https://github.com/efabless/caravel_project_example/actions/workflows/caravel_build.yml/badge.svg)](https://github.com/efabless/caravel_project_example/actions/workflows/caravel_build.yml)

<!-- | :exclamation: Important Note            | -->
<!-- |-----------------------------------------| -->

<!-- ## Please fill in your project documentation in this README.md file  -->

<!-- Refer to [README](docs/source/quickstart.rst) for a quick start of how to use caravel_user_project -->

<!-- Refer to [README](docs/source/index.rst) for this sample project documentation.  -->

## Introduction
This project builds on the Caravel User Project, adding a simple adder that can be used for testing an environment setup. Every time you open a new shell, you must initialize the environment variables manually or by running "shell.sh" from this directory.

## Requirements
In order to setup and harden, you must have
- Docker
- Python 3.6+ with PIP

## Getting started
To setup your environment, you must run ``source ./setup.sh script``. This will create a new directory called "dependencies" and export environment variables needed by the project
Run ``make setup`` to fetch the required openlane resources for hardening.

## Hardening
To harden the user_adder, run ``make user_adder``.
