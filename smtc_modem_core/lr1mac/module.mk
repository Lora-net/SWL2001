# ----------------------------------------------------------------------------
# @file module.mk
#
# Contains list of source files to be compiled in this module.
# ----------------------------------------------------------------------------

# Radio specific sources 
ifeq ($(RADIO),lr1110)
MODULE_C_SOURCES = \
	src/smtc_real/src/region_eu_868.c\
	src/smtc_real/src/region_as_923.c\
	src/smtc_real/src/region_us_915.c\
	src/smtc_real/src/region_in_865.c\
	src/smtc_real/src/region_kr_920.c
endif

ifeq ($(RADIO),sx1261)
MODULE_C_SOURCES = \
	src/smtc_real/src/region_eu_868.c\
	src/smtc_real/src/region_as_923.c\
	src/smtc_real/src/region_us_915.c\
	src/smtc_real/src/region_in_865.c\
	src/smtc_real/src/region_kr_920.c
endif

ifeq ($(RADIO),sx1262)
MODULE_C_SOURCES = \
	src/smtc_real/src/region_eu_868.c\
	src/smtc_real/src/region_as_923.c\
	src/smtc_real/src/region_us_915.c\
	src/smtc_real/src/region_in_865.c\
	src/smtc_real/src/region_kr_920.c
endif

ifeq ($(RADIO),sx128x)
MODULE_C_SOURCES = \
	src/smtc_real/src/region_ww2g4.c 
endif

# Common sources
MODULE_C_SOURCES += \
	src/lr1_stack_mac_layer.c\
	src/lr1mac_core.c\
	src/lr1mac_utilities.c\
	src/smtc_real/src/smtc_real.c

MODULE_C_INCLUDES = \
    . \
	src \
    src/smtc_real/src
