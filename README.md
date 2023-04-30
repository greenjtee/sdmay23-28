# Caravel User Project - sdmay23-28 Senior Design - Digital Chip Fabrication

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![UPRJ_CI](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml/badge.svg)](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml) [![Caravel Build](https://github.com/efabless/caravel_project_example/actions/workflows/caravel_build.yml/badge.svg)](https://github.com/efabless/caravel_project_example/actions/workflows/caravel_build.yml)

| :exclamation: Important Note            |
|-----------------------------------------|

## Digital Chip Spiking Neural Network - Image Classifier

This caravel design implements a spiking neural network for classifying handwritten digits.
 - Weights can be trained using snntorch.
 - Images from MNIST handwritten digit dataset.
 - Wishbone bus allows CPU communication with the user area to initialize weights, image, and control data
