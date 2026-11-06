# Makefile for unit testing wifi_region_finder on host PC

# Compiler and flags
CC = gcc
CFLAGS = -DWIFI_RF_UNIT_TEST -Wall -Wextra -I.. -I../../../../lbm_lib/smtc_modem_api

# Source files
SRC = wrf_test.c ../wifi_region_finder.c
TARGET = wrf_test

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -f $(TARGET)
