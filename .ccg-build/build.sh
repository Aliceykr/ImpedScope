#!/usr/bin/env bash
# Out-of-tree armclang build of the 9_30 firmware.
# Works around the Keil uvprojx pin to ARM-Compiler V6.24 (installed: 6.23),
# which blocks UV4.exe -b command-line builds.
#
# Usage: build.sh [outdir]
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_ROOT="${SRC_ROOT:-$ROOT}"
OUT="${1:-/tmp/930build}"
TOOL="E:/Keil_v5/ARM/ARMCLANG/bin"

CPU_FLAGS="-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard"
DEFINES="-DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H750xx"
INCLUDES="-I$SRC_ROOT/USE -I$SRC_ROOT/Core/Inc -I$SRC_ROOT/Drivers/STM32H7xx_HAL_Driver/Inc -I$SRC_ROOT/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I$SRC_ROOT/Drivers/CMSIS/Device/ST/STM32H7xx/Include -I$SRC_ROOT/Drivers/CMSIS/Include"

WARN_FLAGS="-Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers"

rm -rf "$OUT"; mkdir -p "$OUT/obj"

SOURCES=$(cd "$SRC_ROOT" && grep -o '<FilePath>[^<]*</FilePath>' MDK-ARM/9_30.uvprojx \
          | sed 's/<[^>]*>//g' | sed 's|^\.\./||')

ERRS=0
OBJS=""
for f in $SOURCES; do
  base=$(basename "$f")
  case "$f" in
    *.s)
      obj="$OUT/obj/${base%.s}.o"
      # startup_stm32h750xx.s has no ../ prefix in the uvprojx: it lives in MDK-ARM/
      # It uses legacy armasm (ARM Compiler 5) syntax, not GNU as.
      if [ -f "$SRC_ROOT/$f" ]; then srcf="$SRC_ROOT/$f"; else srcf="$SRC_ROOT/MDK-ARM/$f"; fi
      "$TOOL/armasm.exe" --cpu=Cortex-M7.fp.dp -g "$srcf" -o "$obj" 2>&1 | sed "s|^|[asm $base] |"
      rc=${PIPESTATUS[0]}
      ;;
    *.c)
      obj="$OUT/obj/${base%.c}.o"
      "$TOOL/armclang.exe" --target=arm-arm-none-eabi $CPU_FLAGS $DEFINES $INCLUDES $WARN_FLAGS -std=c99 -O1 -c "$SRC_ROOT/$f" -o "$obj" 2>&1 | sed "s|^|[$base] |"
      rc=${PIPESTATUS[0]}
      ;;
    *) continue ;;
  esac
  if [ "$rc" -ne 0 ]; then ERRS=$((ERRS+1)); echo "*** COMPILE FAILED: $f"; fi
  OBJS="$OBJS $obj"
done

echo "=============================================="
echo "compiled: $(echo $SOURCES | wc -w) files, failures: $ERRS"

if [ "$ERRS" -ne 0 ]; then
  echo "SKIPPING LINK due to compile failures"
  exit 1
fi

cat > "$OUT/scatter.sct" <<'EOF'
LR_IROM1 0x08000000 0x00020000  {
  ER_IROM1 0x08000000 0x00020000  {
   *.o (RESET, +First)
   *(InRoot$$Sections)
   .ANY (+RO)
  }
  RW_IRAM1 0x20000000 0x00020000  {
   .ANY (+RW +ZI)
  }
  RW_IRAM2 0x24000000 0x00080000  {
   .ANY (+RW +ZI)
  }
}
EOF

"$TOOL/armlink.exe" --cpu=Cortex-M7.fp.dp --scatter="$OUT/scatter.sct" --strict --summary_stderr --info summarysizes --entry=Reset_Handler $OBJS -o "$OUT/9_30.axf" 2>&1 | tee "$OUT/link.log"
LINK_RC=${PIPESTATUS[0]}

echo "=============================================="
if [ "$LINK_RC" -eq 0 ]; then echo "RESULT: BUILD OK (0 error)"; else echo "RESULT: LINK FAILED"; fi
exit $LINK_RC