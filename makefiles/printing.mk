######################################
# Some pretty colors and printing
######################################

######################################
# Verbose build
######################################
ifdef VERBOSE
SILENT = 
else
SILENT = @
endif

# Shell colors
BOLD_RED := '\033[1;31m'
BOLD_CYAN := '\033[1;36m'
BOLD_GREEN := '\033[1;32m'
BOLD_PURPLE := '\033[1;35m'
BOLD_YELLOW:= '\033[1;33m'
IPURPLE:= '\033[0;95m'
BIPURPLE:= '\033[1;95m'

NC := '\033[0m' # no color
ECHO = @echo -e $(LIGHT_CYAN)
ECHO_OK = @echo -e $(LIGHT_GREEN)
ECHO_CMD = /bin/echo

define echo
	@$(ECHO_CMD) -e $(BOLD_CYAN)$(1)$(NC)
endef

define echo_error
	@$(ECHO_CMD) -e $(BOLD_RED)'! '$(1)$(NC)
endef

define build
	@echo -e $(BOLD_CYAN)'['$(1)'] Building '$(2)$(NC)
endef

define success
	@echo -e $(BOLD_GREEN)'+ '$(1)' [SUCCESS]'$(NC)
endef

define warn
	@$(ECHO_CMD) -e $(BOLD_PURPLE)'! '$(1)$(NC)
endef

define echo_help
	@$(ECHO_CMD) -e $(IPURPLE)'| '$(1)$(NC)
endef

define echo_help_b
	@$(ECHO_CMD) -e $(BIPURPLE)'  '$(1)$(NC)
endef

define echo_build
	@$(ECHO_CMD) -e $(BOLD_YELLOW)$(1)$(NC)
endef
