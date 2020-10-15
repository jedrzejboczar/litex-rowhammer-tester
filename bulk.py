#!/usr/bin/env python3

from litex import RemoteClient
import time

wb = RemoteClient()
wb.open()

# --------------------------------------------------------------------

# Test patterns: 4 (words) x 32 (bits)
pattern = 0xffffffffffffffffffffffffffffffff
#pattern = 0x00000000000000000000000000000000
#pattern = 0xa5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5
#pattern = 0x5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a
#pattern = 0xaaaaaaaa55555555aaaaaaaa55555555
#pattern = 0xaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
#pattern = 0x55555555555555555555555555555555

# Disable bulk write
wb.regs.bulk_wr_enabled.write(0)

# Reset
wb.regs.bulk_wr_reset.write(1)
wb.regs.bulk_wr_reset.write(0)

# Configure
wb.regs.bulk_wr_address.write(0x00000000) # Without offset
wb.regs.bulk_wr_dataword.write(pattern)
# Arty-A7: 256 MiB DDR3 memory divided by 4 bytes per word and 4 words per write
wb.regs.bulk_wr_count.write(int(256 * 1024 * 1024 / 4 / 4) - 1)

# Enable bulk write
wb.regs.bulk_wr_enabled.write(1)
# Wait until done
while not wb.regs.bulk_wr_done.read():
    time.sleep(1 / 1e6) # 1us
# Stop bulk write
wb.regs.bulk_wr_enabled.write(0)

# Dump last words of the memory
#print('rowhammer: ' + str(["0x{:08x}".format(w) for w in wb.read(0x40000000 + 0x0 * 4, 12)]))
#for m in [(int(256 * 1024 * 1024 / 4) - 1 - 4), (int(256 * 1024 * 1024 / 4) - 1)]:
#    print('0x{:08x}: 0x{:08x}'.format(0x40000000 + m * 4, wb.read(0x40000000 + m * 4, 1)[0] ))

# --------------------------------------------------------------------
#off = 0x10
#wb.write(0x40000000 + off * 4 + 0, 0x0)
#wb.write(0x40000000 + off * 4 + 4, 0x0)
#wb.write(0x40000000 + off * 4 + 8, 0x0)
#wb.write(0x40000000 + off * 4 + 12, 0x0)
#print(':: ' + str(["0x{:08x}".format(w) for w in wb.read(0x40000000 + 0x0 * 4, 12)]))
wb.write(0x40000000 + (23 * 1024 * 1024), 0x0)
# --------------------------------------------------------------------

# Disable bulk read
wb.regs.bulk_rd_enabled.write(0)

# Reset
wb.regs.bulk_rd_reset.write(1)
wb.regs.bulk_rd_reset.write(0)

# Configure
wb.regs.bulk_rd_address.write(0x00000000) # Without offset
wb.regs.bulk_rd_dataword.write(pattern)
# Arty-A7: 256 MiB DDR3 memory divided by 4 bytes per word and 4 words per write
wb.regs.bulk_rd_count.write(int(256 * 1024 * 1024 / 4 / 4) - 1)

wb.regs.bulk_rd_enabled.write(1)
# Wait until done
while not wb.regs.bulk_rd_done.read():
    time.sleep(1 / 1e6) # 1us
# Stop bulk write
wb.regs.bulk_rd_enabled.write(0)
time.sleep(100 / 1e3)

# --------------------
print('pointer: 0x{:08x}'.format(wb.regs.bulk_rd_pointer.read()))
ptr = wb.regs.bulk_rd_pointer.read()
print("unmatching pattern somewhere around " + str(ptr * 4 * 4 / 1024 / 1024) + 'MiB')

wb.close()
