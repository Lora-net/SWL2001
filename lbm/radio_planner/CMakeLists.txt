# Configuration for the radio_planner library

add_library(radio_planner STATIC
    src/duty_cycle.c
    src/radio_planner.c
)

# Set rp path to current scope
set(SMTC_RADIO_PLANNER_PATH ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

# Optional: Disable Radio Planner trace
# Disable with -DSMTC_RADIO_PLANNER_TRACE=OFF
option(SMTC_RADIO_PLANNER_TRACE "Enable Radio Planner trace output" ON)
if(SMTC_RADIO_PLANNER_TRACE)
    target_compile_definitions(radio_planner PRIVATE SMTC_RADIO_PLANNER_TRACE=1)
else()
    target_compile_definitions(radio_planner PRIVATE SMTC_RADIO_PLANNER_TRACE=0)
    message(STATUS "SMTC_RADIO_PLANNER_TRACE: OFF")
endif()

# Optional: Enable Radio Planner deep (debug) trace
# Enable with -DSMTC_RADIO_PLANNER_TRACE_DBG=ON (requires SMTC_RADIO_PLANNER_TRACE=ON)
option(SMTC_RADIO_PLANNER_TRACE_DBG "Enable Radio Planner deep debug trace output" OFF)
if(SMTC_RADIO_PLANNER_TRACE_DBG)
    target_compile_definitions(radio_planner PRIVATE SMTC_RADIO_PLANNER_TRACE_DBG=1)
    message(STATUS "SMTC_RADIO_PLANNER_TRACE_DBG: ON")
endif()

# Set preprocessor macros
if(NOT DEFINED RP_MARGIN_DELAY)
    set(RP_MARGIN_DELAY "8")
endif()
message(STATUS "radio_planner: RP_MARGIN_DELAY=${RP_MARGIN_DELAY}")
target_compile_definitions(radio_planner PUBLIC RP_MARGIN_DELAY=${RP_MARGIN_DELAY} )


# Include necessary directories
target_include_directories(radio_planner PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Add LR11xx include if geoloc (geoloc implies LR1110 / LR1120 chip so no need to check radio aswell)
if(LBM_GEOLOCATION)
    target_include_directories(radio_planner PUBLIC
        ${SMTC_RADIO_DRIVERS_PATH}/lr11xx_driver/src 
    )
endif()

# Link the dependency to the ralf library
target_link_libraries(radio_planner PUBLIC
    smtc_core_hal_interface
    smtc_ralf_${RADIO_FAMILY}
    smtc_ral_${RADIO_FAMILY}
)
# target_link_libraries(radio_planner PUBLIC lora_basics_modem)
# Set C standards
set_target_properties(radio_planner PROPERTIES C_STANDARD 11)

# Install the library
install(TARGETS radio_planner
    EXPORT RadioPlannerTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install the header files
install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/radio_planner.h
    DESTINATION include/radio_planner
)
