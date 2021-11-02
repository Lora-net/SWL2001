#!/bin/bash

mkdir release
cd release
mkdir src
mkdir tests

cp ../.gitlab-ci.yml .
cp ../../LICENSE.txt .

cat ../../README.md | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > ./README.md
cat ../../CHANGELOG.md | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > ./CHANGELOG.md

cat ../../src/CMakeLists.txt | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > src/CMakeLists.txt
cat ../../src/sx126x.c | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' -e 's/DS_SX1261-2_V1.2/DS_LLCC68_V1.0/g' > src/llcc68.c
cat ../../src/sx126x.h | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' -e 's/SX1262/LLCC68/g' -e 's/DS_SX1261-2_V1.2/DS_LLCC68_V1.0/g' > src/llcc68.h
cat ../../src/sx126x_regs.h | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' -e 's/DS_SX1261-2_V1.2/DS_LLCC68_V1.0/g' > src/llcc68_regs.h
cat ../../src/sx126x_hal.h | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > src/llcc68_hal.h

cat ../../tests/project.yml | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > tests/project.yml
cat ../../tests/run_tests.sh | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > tests/run_tests.sh
cat ../../tests/sx126x_toa.h | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > tests/llcc68_toa.h
cat ../../tests/test_sx126x_communication_status.c | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > tests/test_llcc68_communication_status.c
cat ../../tests/test_sx126x_dio_irq_control.c | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > tests/test_llcc68_dio_irq_control.c
cat ../../tests/test_sx126x_extra.c | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > tests/test_llcc68_extra.c
cat ../../tests/test_sx126x_miscellaneous.c | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > tests/test_llcc68_miscellaneous.c
cat ../../tests/test_sx126x_modulation_packet.c | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > tests/test_llcc68_modulation_packet.c
cat ../../tests/test_sx126x_operational_modes.c | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > tests/test_llcc68_operational_modes.c
cat ../../tests/test_sx126x_reg_buf_access.c | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > tests/test_llcc68_reg_buf_access.c

sed -i '/LLCC68_LORA_SF12/d' src/llcc68.h


# Update llcc68.c

input="src/llcc68.c"

line_start=$(awk '/llcc68_set_lora_mod_params/{ print NR; exit }' src/llcc68.c)
line_end=line_start

while IFS= read -r line; do
  ((line_end++))
  [[ $line = }* ]] && break
done < <(tail -n "+$line_start" $input)

sed -i "${line_start},${line_end}d" src/llcc68.c

((line_start-=2))
sed -i "${line_start}r ../llcc68_set_lora_mod_params.c" src/llcc68.c


# Update test_llcc68_modulation_packet.c

input="tests/test_llcc68_modulation_packet.c"

line_start=$(awk '/test_llcc68_set_lora_mod_params/{ print NR; exit }' $input)
line_end=line_start

while IFS= read -r line; do
  ((line_end++))
  [[ $line = }* ]] && break
done < <(tail -n "+$line_start" $input)

((line_end-=1))
sed -i "${line_start},${line_end}d" $input

((line_start-=1))
sed -i "${line_start}r ../llcc68_set_lora_mod_params_test.c" $input
