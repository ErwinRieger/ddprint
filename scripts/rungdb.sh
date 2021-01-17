
echo ""
echo "Note: be aware of the watchdog, it may be enabled in the firmware."
echo ""
st-util &
sleep 1
arm-none-eabi-gdb -ix ../../ddprint_stm32/examples/stm32_gdbinit build/stm32duino.STM32F4.generic_f407v/firmware.ino.elf


