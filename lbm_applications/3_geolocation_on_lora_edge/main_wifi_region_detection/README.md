# WiFi-Based Region Detection Example for LR1110/LR1120

This project demonstrates a region detection system based on nearby Wi-Fi access points, using Semtech's LR1110/LR1120 radios and the LoRa Basics Modem (LBM) library. The goal is to determine the most probable LoRaWAN region (e.g., EU868, US915, CN470) by analyzing Wi-Fi metadata such as country codes and SSIDs.

The project is composed of two parts:
- `wifi_region_finder` (wrf) module: a scoring-based analyzer of Wi-Fi access point data to infer the LoRaWAN region.
- `main_wifi_region_detection.c`: a sample application that performs the Wi-Fi scans using LBM and uses the wrf module to detect the region.

---

## Project Structure

### 1. `wifi_region_finder` Module

**Files**:
- `wifi_region_finder.h`
- `wifi_region_finder.c`

**Purpose**:
Provides region scoring logic based on Wi-Fi access point metadata (MAC address, SSID, country code). This module helps estimate the most likely LoRaWAN region.

> **Note:** This module does **not** perform Wi-Fi scans. Scanning is handled by the LoRa Basics Modem Wi-Fi geolocation service. This module only processes the results.

---

**Main API Functions**:

- `void wrf_init(void);`
  Initializes the internal state of the module.

- `void wrf_process_ap(const uint8_t* mac, const char* ssid, const char* cc, wrf_region_score_t* score);`
  Takes a single access point's MAC, SSID, and country code strings, and updates the region scores. Each MAC is considered only once per scan sequence.

- `void wrf_print_region_scores(const wrf_region_score_t* score);`
  Outputs the current score breakdown per region to a debug trace/log — useful for diagnostics.

- `int wrf_get_highest_score_region(const wrf_region_score_t* score, int threshold, smtc_modem_region_t* region, uint8_t* confidence);`
  Evaluates scores and selects the most probable region, if the score exceeds a given threshold. Also returns a confidence metric (0–100), calculated from internal scoring heuristics.

---

### How Region Detection Works (Inside `wifi_region_finder`)

The `wifi_region_finder` module implements a scoring-based approach to determine the most probable LoRaWAN region based on metadata from nearby Wi-Fi access points.

#### MAC Address De-duplication

Each access point (AP) is identified by its MAC address. The module ensures:
- Each MAC is only considered **once per scan session**
- Separate checks are applied for:
  - **SSID-based scoring**
  - **Country Code-based scoring**

This prevents any single AP from influencing the result multiple times.

---

#### Scoring Logic

Scores are accumulated for each region using two types of hints:

- **Country Code Matching**
If a known 2-letter Wi-Fi country code (e.g., `"FR"`, `"US"`, `"CN"`) is found and associated with a region, the region's score is incremented.

- **SSID Pattern Matching**
If the SSID contains known keywords typical for a region (e.g., `"xfinity"` for US), the region's score is incremented.

Both types of matches are independent and additive.

> 💡 Matching is done using `memcmp()` on non-null-terminated strings.

---

#### Confidence Score Calculation

Once all APs are processed, the `wrf_get_highest_score_region()` function determines the best candidate region:

1. **Top Score Ratio**
   - `top_score_ratio = (top_score * 100) / total_score`
   → How dominant is the top-scoring region?

2. **Top Score Margin**
   - `top_score_margin = (top_score - second_best) * 100 / top_score`
   → How much better is it compared to the second-best?

3. **Final Confidence**
   - `confidence = (top_score_ratio + top_score_margin) / 2`

If the top score is **below a user-defined threshold**, or the result is ambiguous, detection fails and `-1` is returned.

---

**Customization Required**:
This module is **an example implementation** and should be adapted to:
- Include or exclude specific LoRaWAN regions (e.g., AS923, AU915, etc.).
- Adjust the mapping of **country codes** to regions.
- Define **SSID patterns** commonly found in each region.

You are encouraged to extend the scoring logic with your region-specific ISP SSIDs or additional regulatory domains.

---

### 2. `main_wifi_region_detection.c`

**Purpose**:
Serves as an application-level example demonstrating how to:
1. Use the LBM API to perform a Wi-Fi scans.
2. Parse scan results and extract AP metadata.
3. Feed the results into `wifi_region_finder` to determine the most likely region.
4. Display the result and confidence level via UART/logging.

> **Note:** This application will indefinitely perform WiFi scans one after the other, and each time push the results
to the wrf module to update region scoring.

---

## Getting Started

1. Clone or import the code into your LBM project.
2. Customize the `wifi_region_finder.c` logic as needed for your target regions.
3. Build and flash the firmware to your LR11xx-enabled board.
4. Open a terminal to view the Wi-Fi scan and inferred region.

---

## Build

### With CMake

Ensure that there is no existing build directory from a previous build.
Use "rm -r build" if needed to clean it up.

``` bash
cmake -B build -G Ninja -DLBM_RADIO=lr1110 -DCMAKE_BUILD_TYPE=MinSizeRel -DLBM_CMAKE_CONFIG_AUTO=ON -DAPP=wifi_region_detection
```

Optionnally add `-DLBM_MODEM_TRACE=no` to the cmake command to disable LBM traces.

``` bash
ninja -C build
```

It is possible to flash the target with the following commands:

``` bash
ninja -C build flash
```

Note: `stlink-tools` needs to be installed.

For "copy" flashing, the following command can be used:

``` bash
ninja -C build flash_copy
```

Note: it expects the target to be mounted to `/media/$(USER)/NODE_L476RG`

### With make

The example can be built through GNU make command by doing the following:

```shell
$ make full_lr1110 MODEM_APP=EXAMPLE_WIFI_REGION_DETECTION
```

All the default compilation options used are available in the `../app_makefiles/app_options.mk` file.

Optionnally add `LBM_TRACE=no` to the make command to disable LBM traces.

---

## Output example

```shell

INFO: Event received: WIFI_SCAN_DONE
SCAN_DONE info:
-- number of results: 5
-- power consumption: 20765 nah
-- scan duration: 7199 ms
[ 14 CC 20 CB 53 56 ]	Channel:8	Type:1	RSSI:-57	Origin:FIXED	Country code:..	SSID:TP-LINK_CB5356                  
[ 64 70 02 D9 94 55 ]	Channel:6	Type:1	RSSI:-74	Origin:FIXED	Country code:US	SSID:briere                          
[ F4 CA E5 8B 49 A0 ]	Channel:11	Type:1	RSSI:-91	Origin:FIXED	Country code:..	SSID:freebox_briere                  
[ F4 CA E5 8B 49 A2 ]	Channel:11	Type:1	RSSI:-92	Origin:FIXED	Country code:..	SSID:FreeWifi_secure                 
[ 38 B5 C9 03 8A 60 ]	Channel:1	Type:1	RSSI:-94	Origin:FIXED	Country code:FR	SSID:Livebox-8A60                    

=== Region Scores (value, ratio%, margin%) ===
EU868 :  5,  83%,  80%
US915 :  1,  17%,  N/A
CN470 :  0,   0%,  N/A
AU915 :  0,   0%,  N/A
KR920 :  0,   0%,  N/A
IN865 :  0,   0%,  N/A
RU864 :  0,   0%,  N/A
AS923 :  0,   0%,  N/A
===========================================

INFO: -----------------------
INFO: Estimated region: EU868 with 81% confidence
INFO: -----------------------

```
