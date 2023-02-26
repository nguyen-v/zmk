# Copyright (c) 2021 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_sources_ifdef(CONFIG_IL0323		il0323.c)
zephyr_sources_ifdef(CONFIG_LS0XX_CUSTOM ls0xx_custom.c)