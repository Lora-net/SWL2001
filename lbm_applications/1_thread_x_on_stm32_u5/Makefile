##############################################################################
# Main makefile for basic_modem
##############################################################################
-include app_makefiles/app_printing.mk
-include app_makefiles/app_options.mk


#-----------------------------------------------------------------------------
# default action: print help
#-----------------------------------------------------------------------------
help:
	$(call echo_help_b, "Available TARGETs:	lr1110	lr1120	sx1261	sx1262")
	$(call echo_help, "")
	$(call echo_help_b, "-------------------------------- Clean -------------------------------------")
	$(call echo_help, " * make clean_<TARGET>             : clean basic_modem app and lib for a given target")
	$(call echo_help, " * make clean_all                  : clean all")
	$(call echo_help, " * make clean_app                  : clean basic_modem app")
	$(call echo_help, "")
	$(call echo_help_b, "----------------------------- Compilation ----------------------------------")
	$(call echo_help, " * make <TARGET>                   : build basic_modem app and lib on a given target")
	$(call echo_help, " * make <TARGET>_tb                : build basic_modem app and lib on a given target for testbench")
	$(call echo_help, "")
	$(call echo_help_b, "---------------------------- All inclusive ---------------------------------")
	$(call echo_help, " * make full_<TARGET>              : clean and build basic_modem on a given target")
	$(call echo_help, " * make full_<TARGET>_tb           : clean and build basic_modem on a given target for testbench")
	$(call echo_help, "")
	$(call echo_help_b, "-------------------- Optional makefile parameters --------------------------")
	$(call echo_help, " * MULTITHREAD=no                  : Disable multithreaded build")
	$(call echo_help, " * VERBOSE=yes                     : Increase build verbosity")
	$(call echo_help, " * SIZE=yes                        : Display size for all objects")
	$(call echo_help, " * DEBUG=yes                       : Compile library and application with debug symbols")



#-----------------------------------------------------------------------------
# Makefile include selection
#-----------------------------------------------------------------------------
ifeq ($(TARGET_RADIO),lr1110)
-include app_makefiles/app_lr11xx.mk
endif

ifeq ($(TARGET_RADIO),lr1120)
-include app_makefiles/app_lr11xx.mk
endif

ifeq ($(TARGET_RADIO),sx1261)
-include app_makefiles/app_sx126x.mk
endif

ifeq ($(TARGET_RADIO),sx1262)
-include app_makefiles/app_sx126x.mk
endif


#-----------------------------------------------------------------------------
-include app_makefiles/app_common.mk

.PHONY: clean_all help

#-----------------------------------------------------------------------------
# Clean
#-----------------------------------------------------------------------------
clean_all: clean_app
	$(MAKE) -C $(LORA_BASICS_MODEM) clean_all $(MTHREAD_FLAG)

clean_lr1110:
	$(MAKE) clean_modem TARGET_RADIO=lr1110
	$(MAKE) clean_target TARGET_RADIO=lr1110

clean_lr1120:
	$(MAKE) clean_modem TARGET_RADIO=lr1120
	$(MAKE) clean_target TARGET_RADIO=lr1120

clean_sx1261:
	$(MAKE) clean_modem TARGET_RADIO=sx1261
	$(MAKE) clean_target TARGET_RADIO=sx1261

clean_sx1262:
	$(MAKE) clean_modem TARGET_RADIO=sx1262
	$(MAKE) clean_target TARGET_RADIO=sx1262



clean:
	$(MAKE) clean_modem
	$(MAKE) clean_target

clean_app:
	-rm -rf $(APPBUILD_ROOT)*


#-----------------------------------------------------------------------------
# Application Compilation
#-----------------------------------------------------------------------------

#-- Generic -------------------------------------------------------------------
app:
ifeq ($(TARGET_RADIO),nc)
	$(call echo_error,"No radio selected! Please specified the target radio using TARGET_RADIO=radio_name option")
else
ifneq ($(CRYPTO),SOFT)
ifneq ($(LBM_NB_OF_STACK),1)
	$(call echo_error, "----------------------------------------------------------")
	$(call echo_error, "More than one stack compiled: only soft crypto can be used")
	$(call echo_error, "Please use CRYPTO=SOFT option")
	$(call echo_error, "----------------------------------------------------------")
	exit 1
endif
ifneq ($(TARGET_RADIO),lr1110)
ifneq ($(TARGET_RADIO),lr1120)
	$(call echo_error, "------------------------------------------------------------")
	$(call echo_error, "sx126x, sx127x and sx128x radio tagets: only soft crypto can be used")
	$(call echo_error, "Please use CRYPTO=SOFT option")
	$(call echo_error, "------------------------------------------------------------")
	exit 1
endif
endif
endif
	$(MAKE) app_build
endif

#-- LR1110 -------------------------------------------------------------------
full_lr1110:
	$(MAKE) clean_modem TARGET_RADIO=lr1110
	$(MAKE) clean_target TARGET_RADIO=lr1110
	$(MAKE) app_build TARGET_RADIO=lr1110 $(MTHREAD_FLAG)


#-- LR1120 -------------------------------------------------------------------

full_lr1120:
	$(MAKE) clean_modem TARGET_RADIO=lr1120
	$(MAKE) clean_target TARGET_RADIO=lr1120
	$(MAKE) app_build TARGET_RADIO=lr1120 $(MTHREAD_FLAG)

#-- SX1261 -------------------------------------------------------------------
full_sx1261:
	$(MAKE) clean_modem TARGET_RADIO=sx1261
	$(MAKE) clean_target TARGET_RADIO=sx1261
	$(MAKE) app_build TARGET_RADIO=sx1261 $(MTHREAD_FLAG)

#-- SX1262 -------------------------------------------------------------------
full_sx1262:
	$(MAKE) clean_modem TARGET_RADIO=sx1262
	$(MAKE) clean_target TARGET_RADIO=sx1262
	$(MAKE) app_build TARGET_RADIO=sx1262 $(MTHREAD_FLAG)


