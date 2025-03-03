# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(push_button)

target_sources(app PRIVATE src/accel.c)
target_sources(app PRIVATE src/battery.c)
target_sources(app PRIVATE src/button.c)
target_sources(app PRIVATE src/led.c)
target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/therm.c)
