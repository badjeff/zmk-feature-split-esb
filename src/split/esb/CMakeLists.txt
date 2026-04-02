# Copyright (c) 2025 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library()

zephyr_library_sources( common.c )
zephyr_library_sources_ifdef(CONFIG_ZMK_SPLIT_ROLE_CENTRAL  central.c)
zephyr_library_sources_ifndef(CONFIG_ZMK_SPLIT_ROLE_CENTRAL peripheral.c)

# zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/include)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)

if(CONFIG_SOC_NRF5340_CPUAPP)
    zephyr_library_sources(
        53_app/app_esb_53_app.c
    )
else ()
    # assume a 52 series board is selected
    zephyr_library_sources(app_esb.c)
    zephyr_library_sources(timeslot.c)
endif()
zephyr_library_include_directories(.)

set(CENTRAL_H "${APPLICATION_SOURCE_DIR}/../app/include/zmk/split/central.h")
file(READ "${CENTRAL_H}" CONTENTS)
set(DEFAULT_LINE "#define ZMK_SPLIT_CENTRAL_PERIPHERAL_COUNT MAX(BLE_PERIPHERAL_COUNT, WIRED_PERIPHERAL_COUNT)")
if(CONFIG_ZMK_SPLIT_ESB_PERIPHERAL_COUNT GREATER 0)
    message(STATUS "ESB peripheral count is > 0")
    set(NEW_LINE "#define ZMK_SPLIT_CENTRAL_PERIPHERAL_COUNT ${CONFIG_ZMK_SPLIT_ESB_PERIPHERAL_COUNT}")
else()
    message(STATUS "Restoring default peripheral count")
    set(NEW_LINE "${DEFAULT_LINE}")
endif()
# Replace the whole line
string(REGEX REPLACE
    "#define[ \t]+ZMK_SPLIT_CENTRAL_PERIPHERAL_COUNT[^\r\n]*"
    "${NEW_LINE}"
    CONTENTS_MODIFIED
    "${CONTENTS}"
)
# Only write if changed
if(NOT CONTENTS STREQUAL CONTENTS_MODIFIED)
    file(WRITE "${CENTRAL_H}" "${CONTENTS_MODIFIED}")
    message(STATUS "central.h updated")
endif()

#---
# add a pointer to "static void radio_dynamic_irq_handler(const void *args)" in nrf/subsys/esb/esb.c
# to make it be accessible via "extern"
#---
# file(READ   ${APPLICATION_SOURCE_DIR}/../nrf/subsys/esb/esb.c CONTENTS)
# if(NOT CONTENTS MATCHES "__ptr__radio_dynamic_irq_handler")
#     set(PATCH "void *__ptr__radio_dynamic_irq_handler = &radio_dynamic_irq_handler;\n")
#     file(APPEND ${APPLICATION_SOURCE_DIR}/../nrf/subsys/esb/esb.c "${PATCH}")
# endif()
