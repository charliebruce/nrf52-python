from time import sleep
import nrf52
import spi_flash
import pylink
from tqdm import tqdm

# Load the data file
with open("out.bin", "rb") as f:
    bin_data = f.read()

# 64 megabits - UK 40x prototypes
assert len(bin_data) == 64 * 1024 * 1024 / 8



jlink = pylink.JLink()
jlink.open()
jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
jlink.connect("NRF52832_xxAA")
jlink.reset()

sleep(3)


nrf = nrf52.nrf52(jlink)

print(f"Connected to {nrf.read_mac()}")
print(f"IC details: {nrf.read_details()}")

# CLK, MOSI, MISO
nrf.spim_init(20, 19, 21)

cs = 24

flash = spi_flash.spi_flash(nrf, cs)
flash.read_cr2()
firstBytes = flash.spi_flash_read_into_buffer(0, 128)

print(f"First bytes are: {firstBytes}")
print(f"File  bytes are: {bin_data[0:128]}")

# data = bytearray(range(100, 200))
# recv = nrf.spim_send(12, data) #cs

# flash.spi_flash_reset()
flash.jedec_verify()
sr1 = flash.spi_flash_status_reg_1_read()
sr2 = flash.spi_flash_status_reg_2_read()

print(f"SR1: {sr1}, SR2: {sr2}")
#flash.spi_flash_erase_chip()

# Every sector should be 0xFF now, so we can just write the audio data.

# Split the chunks into Sectors of 4KB
sector_nums = range(len(bin_data) // 4096)

# Write each sector
for s in tqdm(sector_nums):
    data = bin_data[4096*s : 4096*(s+1)]
    flash.spi_flash_write_sector(s, data)
    pass


print("DONE")


# print(f"Sent: {data}")
# print(f"Recv: {recv}")
