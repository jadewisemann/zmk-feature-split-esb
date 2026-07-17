# Copyright (c) 2025 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library()
zephyr_include_directories(include)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
zephyr_library_sources(split_central_status_changed.c)
