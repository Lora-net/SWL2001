from __future__ import print_function
import argparse
import math
import os
print('****** prog2c_header.py ******')

parser = argparse.ArgumentParser(prog='prog2c_header.py', description='Converts a .prog file into a .h file')
parser.add_argument("--progFile",help="input progFile",default="")
parser.add_argument("--hFile",help="output .h file", default="")
args = parser.parse_args()

print('Processing file ',args.progFile)

# find last line in prog file
a = 0
lastLine = 0
lineCount = 0
with open(args.progFile, "rb") as f:
    allProg = f.readlines()
allProgDecoded = []
for line in allProg:
    lineCount += 1
    line = line.decode("ascii")
    if len(line) < 10:
        continue
    u = int(line, 2)
    allProgDecoded.append(u)
    if u != 0xFFFFFFFF:
        lastLine = lineCount

print('Last line ',lastLine)

# write .h file
if args.hFile != "":
    print("Generating h file", args.hFile)
    lineCount = 0
    with open(args.hFile, "w") as f:
        f.write('const uint32_t lr11xx_firmware_image[] = { \n')
        for u in allProgDecoded:
            lineCount += 1
            if lineCount > lastLine:
                break
            f.write('%s,\n' % (hex(u)))
        f.write('};\n')
        f.write('\n')
        f.write('#define LR11XX_FIRMWARE_IMAGE_SIZE %s\n' % (lastLine))

print("*****************************")



