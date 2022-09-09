
#-----------------------------------------------------------------------------
# Region selection. They are all enabled by default, unless one is selected via the REGION flag
#-----------------------------------------------------------------------------
REGION_AS_923 = no
REGION_AU_915 = no
REGION_CN_470 = no
REGION_CN_470_RP_1_0 = no
REGION_EU_868 = no
REGION_IN_865 = no
REGION_KR_920 = no
REGION_RU_864 = no
REGION_US_915 = no
REGION_WW_2G4 = no

ifndef REGION
ifneq ($(RADIO),sx128x)
REGION_AS_923 = yes
REGION_AU_915 = yes
REGION_CN_470 = yes
REGION_CN_470_RP_1_0 = yes
REGION_EU_868 = yes
REGION_IN_865 = yes
REGION_KR_920 = yes
REGION_RU_864 = yes
REGION_US_915 = yes
else
REGION_WW_2G4 = yes
endif
ifeq ($(RADIO),lr1120)
REGION_WW_2G4 = yes
endif
endif # REGION

#-----------------------------------------------------------------------------
# Regional Parameter Version
#-----------------------------------------------------------------------------

ifndef RP_VERSION
MODEM_C_DEFS += -DRP2_101
endif

ifeq ($(RP_VERSION),RP2_103)
MODEM_C_DEFS += -DRP2_103
endif

ifeq ($(RP_VERSION),RP2_101)
MODEM_C_DEFS += -DRP2_101
endif

#-----------------------------------------------------------------------------
# Extract all comma-separated regions into a list
#-----------------------------------------------------------------------------
COMMA := ,
REGION_LIST = $(subst $(COMMA), ,$(REGION))

#-----------------------------------------------------------------------------
# Manual selection of one region
#-----------------------------------------------------------------------------
ifneq ($(filter AS_923,$(REGION_LIST)),)
REGION_AS_923 = yes
endif
ifneq ($(filter AU_915,$(REGION_LIST)),)
REGION_AU_915 = yes
endif
ifneq ($(filter CN_470,$(REGION_LIST)),)
REGION_CN_470 = yes
endif
ifneq ($(filter CN_470_RP_1_0,$(REGION_LIST)),)
REGION_CN_470_RP_1_0 = yes
endif
ifneq ($(filter EU_868,$(REGION_LIST)),)
REGION_EU_868 = yes
endif
ifneq ($(filter IN_865,$(REGION_LIST)),)
REGION_IN_865 = yes
endif
ifneq ($(filter KR_920,$(REGION_LIST)),)
REGION_KR_920 = yes
endif
ifneq ($(filter RU_864,$(REGION_LIST)),)
REGION_RU_864 = yes
endif
ifneq ($(filter US_915,$(REGION_LIST)),)
REGION_US_915 = yes
endif
ifneq ($(filter WW_2G4,$(REGION_LIST)),)
REGION_WW_2G4 = yes
endif

#-----------------------------------------------------------------------------
# Region sources and defines
#-----------------------------------------------------------------------------
ifeq ($(REGION_AS_923), yes)
LR1MAC_C_SOURCES += smtc_modem_core/lr1mac/src/smtc_real/src/region_as_923.c
MODEM_C_DEFS += -DREGION_AS_923
endif
ifeq ($(REGION_AU_915), yes)
LR1MAC_C_SOURCES += smtc_modem_core/lr1mac/src/smtc_real/src/region_au_915.c
MODEM_C_DEFS += -DREGION_AU_915
endif
ifeq ($(REGION_CN_470), yes)
LR1MAC_C_SOURCES += smtc_modem_core/lr1mac/src/smtc_real/src/region_cn_470.c
MODEM_C_DEFS += -DREGION_CN_470
endif
ifeq ($(REGION_CN_470_RP_1_0), yes)
LR1MAC_C_SOURCES += smtc_modem_core/lr1mac/src/smtc_real/src/region_cn_470_rp_1_0.c
MODEM_C_DEFS += -DREGION_CN_470_RP_1_0
endif
ifeq ($(REGION_EU_868), yes)
LR1MAC_C_SOURCES += smtc_modem_core/lr1mac/src/smtc_real/src/region_eu_868.c
MODEM_C_DEFS += -DREGION_EU_868
endif
ifeq ($(REGION_IN_865), yes)
LR1MAC_C_SOURCES += smtc_modem_core/lr1mac/src/smtc_real/src/region_in_865.c
MODEM_C_DEFS += -DREGION_IN_865
endif
ifeq ($(REGION_KR_920), yes)
LR1MAC_C_SOURCES += smtc_modem_core/lr1mac/src/smtc_real/src/region_kr_920.c
MODEM_C_DEFS += -DREGION_KR_920
endif
ifeq ($(REGION_RU_864), yes)
LR1MAC_C_SOURCES += smtc_modem_core/lr1mac/src/smtc_real/src/region_ru_864.c
MODEM_C_DEFS += -DREGION_RU_864
endif
ifeq ($(REGION_US_915), yes)
LR1MAC_C_SOURCES += smtc_modem_core/lr1mac/src/smtc_real/src/region_us_915.c
MODEM_C_DEFS += -DREGION_US_915
endif
ifeq ($(REGION_WW_2G4), yes)
LR1MAC_C_SOURCES += smtc_modem_core/lr1mac/src/smtc_real/src/region_ww2g4.c
MODEM_C_DEFS += \
	-DREGION_WW2G4\
	-DWW2G4_SINGLE_DATARATE
endif


