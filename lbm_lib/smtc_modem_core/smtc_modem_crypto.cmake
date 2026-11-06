# SPDX-License-Identifier: BSD-3-Clause-Clear

set(SMTC_MODEM_CRYPTO_DIR ${CMAKE_CURRENT_LIST_DIR}/smtc_modem_crypto)

add_library(smtc_modem_crypto OBJECT)

target_include_directories(smtc_modem_crypto PUBLIC
    ${SMTC_MODEM_CRYPTO_DIR}/smtc_secure_element
)

target_link_libraries(smtc_modem_crypto PRIVATE
    smtc_modem_hal_interface
    smtc_modem_api_interface
    smtc_modem_logging
    smtc_ral::smtc_ral_${RADIO_FAMILY}
    smtc_ralf::smtc_ralf_${RADIO_FAMILY}
)

target_compile_definitions(smtc_modem_crypto PRIVATE NUMBER_OF_STACKS=${LBM_NUMBER_OF_STACKS})

target_sources(smtc_modem_crypto PRIVATE
    ${SMTC_MODEM_CRYPTO_DIR}/smtc_modem_crypto.c
)

if(LBM_CRYPTO STREQUAL "SOFT")
    target_sources(smtc_modem_crypto PRIVATE
        ${SMTC_MODEM_CRYPTO_DIR}/soft_secure_element/aes.c
        ${SMTC_MODEM_CRYPTO_DIR}/soft_secure_element/cmac.c
        ${SMTC_MODEM_CRYPTO_DIR}/soft_secure_element/soft_se.c
    )
endif()

# NOTE: LBM_FUOTA fragmentation package v2 uses aes / cmac from soft crypto
if(LBM_FUOTA AND LBM_FUOTA_VERSION STREQUAL "2")
    target_include_directories(smtc_modem_crypto INTERFACE
        ${SMTC_MODEM_CRYPTO_DIR}/soft_secure_element
    )
endif()

if(LBM_CRYPTO MATCHES "^LR11XX")
    target_sources(smtc_modem_crypto PRIVATE
        ${SMTC_MODEM_CRYPTO_DIR}/lr11xx_crypto_engine/lr11xx_ce.c
    )
    target_link_libraries(smtc_modem_crypto PRIVATE ${radio_driver_library})
    # lr11xx_ce.c needs includes from smtc/lr1mac
    target_include_directories(smtc_modem_crypto PRIVATE
        ${SMTC_MODEM_CORE_INCLUDE_DIRS}
    )
endif()

if(LBM_CRYPTO MATCHES "_WITH_CREDENTIALS$")
    target_compile_definitions(smtc_modem_crypto PRIVATE USE_PRE_PROVISIONED_FEATURES)
endif()
