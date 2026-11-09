/**
 * @file      smtc_lbm_code_ptr.h
 *
 * @brief     Optional validation of function/callback pointers before indirect calls
 *
 * Default: NULL is invalid; any non-NULL is accepted.
 * When SMTC_LBM_CODE_PTR_USE_FLASH_BOUNDS is 1, non-NULL pointers must lie in
 * the linker-provided used-flash range [_snvm, _envm) (e.g. ARC Cervin firmware).
 *
 * Ports may pre-define SMTC_LBM_IS_VALID_FN_PTR to override the policy.
 */
#ifndef SMTC_LBM_CODE_PTR_H
#define SMTC_LBM_CODE_PTR_H

#include <stdint.h>

#ifndef SMTC_LBM_IS_VALID_FN_PTR

#if defined( SMTC_LBM_CODE_PTR_USE_FLASH_BOUNDS ) && ( SMTC_LBM_CODE_PTR_USE_FLASH_BOUNDS != 0 )

/* Linker symbols: start and end of used program image in non-volatile memory. */
extern unsigned char _snvm[];
extern unsigned char _envm[];

#define SMTC_LBM_IS_VALID_FN_PTR( fptr )                                         \
    ( ( ( uintptr_t ) ( fptr ) ) == ( uintptr_t ) 0                              \
          ? 0U                                                                   \
          : ( ( ( uintptr_t ) ( fptr ) ) >= ( uintptr_t ) ( void* ) _snvm &&     \
                      ( ( uintptr_t ) ( fptr ) ) < ( uintptr_t ) ( void* ) _envm \
                  ? 1U                                                           \
                  : 0U ) )

#else

/* Function pointers: compare as uintptr_t (C avoids void* round-trip on fnptr). */
#define SMTC_LBM_IS_VALID_FN_PTR( fptr ) ( ( ( uintptr_t ) ( fptr ) ) != ( uintptr_t ) 0 )

#endif /* SMTC_LBM_CODE_PTR_USE_FLASH_BOUNDS */

#endif /* SMTC_LBM_IS_VALID_FN_PTR */

#endif /* SMTC_LBM_CODE_PTR_H */
