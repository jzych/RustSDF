# RustSDF
[![codecov](https://codecov.io/gh/jzych/RustSDF/graph/badge.svg?token=9F0xQDmylK)](https://codecov.io/gh/jzych/RustSDF)[![Clippy](https://github.com/aeMortis/RustSDF/actions/workflows/rust.yml/badge.svg)](https://github.com/aeMortis/RustSDF/actions/workflows/rust.yml)

This project demonstrates that fusing data from an accelerometer and GPS using a Kalman filter can provide significantly more accurate position estimates compared to using either data source alone.

![working RustSDF](RustSDF.gif)

## Setup

To setup project, first you need to initialize submodules with:

`git submodule init --update`

Additional dependencies for Linux:

Ubuntu Linux
`sudo apt install pkg-config libfreetype6-dev libfontconfig1-dev`

Fedora Linux
`sudo dnf install pkgconf freetype-devel fontconfig-devel`

## Run

To run project use command:

`cargo run`
