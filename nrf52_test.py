from time import sleep
import pylink
jlink = pylink.JLink()
jlink.open()
jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
jlink.connect("NRF52832_xxAA")
jlink.reset()

sleep(3)

import nrf52

nrf = nrf52.nrf52(jlink)

# CLK, MOSI, MISO
nrf.spim_init(15, 13, 7)

data = bytearray(range(100, 200))
recv = nrf.spim_send(12, data) #cs

print("DONE")

print(f"Sent: {data}")
print(f"Recv: {recv}")

sleep(10)