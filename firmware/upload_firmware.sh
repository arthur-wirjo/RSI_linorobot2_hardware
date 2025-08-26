#!/bin/bash

pio run --target clean -e esp32s3
pio run --target upload -e esp32s3
