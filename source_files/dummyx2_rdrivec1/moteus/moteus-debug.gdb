# moteus firmware debug GDB init script
# Usage: gdb-multiarch -x moteus-debug.gdb bazel-bin/fw/moteus.elf

set mem inaccessible-by-default off
dir bazel-moteus
target extended-remote localhost:3333

# Stop and reset
monitor reset init
monitor halt

# Useful aliases
define reboot
  monitor reset init
  continue
end

define reflash
  monitor reset init
  monitor halt
  monitor flash write_image erase bazel-bin/fw/moteus.08000000.bin 0x8000000
  monitor flash write_image erase bazel-bin/fw/moteus.0800c000.bin 0x800c000
  monitor flash write_image erase bazel-bin/fw/moteus.08010000.bin 0x8010000
  monitor reset init
end
