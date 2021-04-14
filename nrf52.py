

# Allow control of an nRF52832 by directly messing with registers
# This allows a SWD debugger to quickly prove board functionality
# This is just a small subset of the complete functionality
# It is designed to allow basic self-test and bringup functions only
# Should be applicable to others in the family, but may need to fix, for example, GPIO port 1
from time import sleep
import os

#
# GPIO
#
NRF52_GPIO_BASE = 0x50000000

NRF52_GPIO_REG_OUT      = NRF52_GPIO_BASE + 0x504
NRF52_GPIO_REG_OUTSET   = NRF52_GPIO_BASE + 0x508
NRF52_GPIO_REG_OUTCLR   = NRF52_GPIO_BASE + 0x50C
NRF52_GPIO_REG_IN       = NRF52_GPIO_BASE + 0x510
NRF52_GPIO_REG_DIR      = NRF52_GPIO_BASE + 0x514
NRF52_GPIO_REG_DIRSET   = NRF52_GPIO_BASE + 0x518
NRF52_GPIO_REG_DIRCLR   = NRF52_GPIO_BASE + 0x51C
def NRF52_GPIO_REG_PIN_CNF(pin):

    assert isinstance(pin, int), "Invalid type for pin - need int"
    assert pin < 32, "Need fixes before this code can handle GPIO P1"
    assert pin >= 0, "Invalid pin"

    return NRF52_GPIO_BASE + 0x700 + (0x04 * pin)

GPIO_DIR_INPUT = 0
GPIO_DIR_OUTPUT = 1

GPIO_BUFFER_CONNECT = 0
GPIO_BUFFER_DISCONNECT = 1

GPIO_PULL_NONE = 0
GPIO_PULL_DOWN = 1
GPIO_PULL_UP = 3

GPIO_DRIVE_S0S1 = 0
GPIO_DRIVE_H0S1 = 1
GPIO_DRIVE_S0H1 = 2
GPIO_DRIVE_H0H1 = 3
GPIO_DRIVE_D0S1 = 4
GPIO_DRIVE_D0H1 = 5
GPIO_DRIVE_S0D1 = 6
GPIO_DRIVE_H0D1 = 7

#
# Clock
#
NRF52_CLOCK_BASE = 0x40000000

NRF52_CLOCK_REG_HFCLKSTAT = NRF52_CLOCK_BASE + 0x40C



#
# SPI Master
# 
NRF52_SPIM0_BASE = 0x40003000
NRF52_SPIM1_BASE = 0x40004000
NRF52_SPIM2_BASE = 0x40023000

NRF52_SPIM_OFFSET_TASKS_START       = 0x010
NRF52_SPIM_OFFSET_TASKS_STOP        = 0x014
NRF52_SPIM_OFFSET_TASKS_SUSPEND     = 0x01C
NRF52_SPIM_OFFSET_TASKS_RESUME      = 0x020
NRF52_SPIM_OFFSET_EVENTS_STOPPED    = 0x104
NRF52_SPIM_OFFSET_EVENTS_ENDRX      = 0x110
NRF52_SPIM_OFFSET_EVENTS_END        = 0x118
NRF52_SPIM_OFFSET_EVENTS_ENDTX      = 0x120
NRF52_SPIM_OFFSET_EVENTS_STARTED    = 0x14C
NRF52_SPIM_OFFSET_ENABLE            = 0x500
NRF52_SPIM_OFFSET_SCK               = 0x508
NRF52_SPIM_OFFSET_MOSI              = 0x50C
NRF52_SPIM_OFFSET_MISO              = 0x510
NRF52_SPIM_OFFSET_FREQUENCY         = 0x524
NRF52_SPIM_OFFSET_RXD_PTR           = 0x534
NRF52_SPIM_OFFSET_RXD_MAXCNT        = 0x538
NRF52_SPIM_OFFSET_RXD_AMOUNT        = 0x53C
NRF52_SPIM_OFFSET_RXD_LIST          = 0x540
NRF52_SPIM_OFFSET_TXD_PTR           = 0x544
NRF52_SPIM_OFFSET_TXD_MAXCNT        = 0x548
NRF52_SPIM_OFFSET_TXD_AMOUNT        = 0x54C
NRF52_SPIM_OFFSET_TXD_LIST          = 0x550
NRF52_SPIM_OFFSET_CONFIG            = 0x554

NRF52_SPIM_FREQUENCY_8M = 0x80000000
NRF52_SPIM_FREQUENCY_4M = 0x40000000
NRF52_SPIM_FREQUENCY_2M = 0x20000000
NRF52_SPIM_FREQUENCY_1M = 0x10000000
NRF52_SPIM_FREQUENCY_500K = 0x08000000
NRF52_SPIM_FREQUENCY_250K = 0x04000000
NRF52_SPIM_FREQUENCY_125K = 0x02000000

#
# NVMC
#
NRF52_NVMC_BASE = 0x4001E000

NRF52_NVMC_REG_READY            = NRF52_NVMC_BASE + 0x400
NRF52_NVMC_REG_CONFIG           = NRF52_NVMC_BASE + 0x504
NRF52_NVMC_REG_ERASEALL         = NRF52_NVMC_BASE + 0x50C
NRF52_NVMC_REG_ERASEUICR        = NRF52_NVMC_BASE + 0x514

# 
# RAM
# 
NRF52_DATA_RAM_BASE = 0x20000000
NRF52_DATA_RAM_LEN = 0x00010000 # This is 64kB - xxAA only.

#
# FICR
#
NRF52_FICR_BASE = 0x10000000
NRF52_FICR_REG_DEVICEADDR0      = NRF52_FICR_BASE + 0x0A4
NRF52_FICR_REG_DEVICEADDR1      = NRF52_FICR_BASE + 0x0A8

NRF52_FICR_REG_INFO_PART        = NRF52_FICR_BASE + 0x100
NRF52_FICR_REG_INFO_VARIANT     = NRF52_FICR_BASE + 0x104
NRF52_FICR_REG_INFO_PACKAGE     = NRF52_FICR_BASE + 0x108
NRF52_FICR_REG_INFO_RAM         = NRF52_FICR_BASE + 0x10C
NRF52_FICR_REG_INFO_FLASH       = NRF52_FICR_BASE + 0x110

#
# UICR
#
NRF52_UICR_BASE = 0x10001000

NRF52_UICR_REG_NRFFW            = NRF52_UICR_BASE + 0x014
NRF52_UICR_REG_NRFHW            = NRF52_UICR_BASE + 0x050
NRF52_UICR_REG_CUSTOMER         = NRF52_UICR_BASE + 0x080
NRF52_UICR_REG_PSELRESET        = NRF52_UICR_BASE + 0x200
NRF52_UICR_REG_APPROTECT        = NRF52_UICR_BASE + 0x208
NRF52_UICR_REG_NFCPINS          = NRF52_UICR_BASE + 0x20C


class nrf52:

    def __init__(self, jlink):

        self.jlink = jlink

    # Specify full configuration of GPIO pin. Defaults match reset state.
    def gpio_cfg_full(self, pin, dir=GPIO_DIR_INPUT, input_buffer=GPIO_BUFFER_DISCONNECT, pull=GPIO_PULL_NONE, drive=GPIO_DRIVE_S0S1, sense=0):
            
        assert isinstance(pin, int), "Invalid type for pin - need int"
        assert pin < 32, "Need fixes before this code can handle GPIO P1"
        assert pin >= 0, "Invalid pin"        
        assert (pin not in [9, 10]) or self.nfcpins_are_gpios(), "Pin is set up for NFC, cannot be used for GPIO"
        
        # The PINCNF register handles everything. No need to write DIR, it's the same physical egister.
        cnf = (dir) + (input_buffer << 1) + (pull << 2) + (drive << 8) + (sense << 16)

        self.jlink.memory_write32(NRF52_GPIO_REG_PIN_CNF(pin), [cnf])


    def gpio_write(self, pin, state):

        assert isinstance(pin, int), "Invalid type for pin - need int"
        assert pin < 32, "Need fixes before this code can handle GPIO P1"
        assert pin >= 0, "Invalid pin"
        assert (pin not in [9, 10]) or self.nfcpins_are_gpios(), "Pin is set up for NFC, cannot be used for GPIO"


        if state:
            reg = NRF52_GPIO_REG_OUTSET
        else:
            reg = NRF52_GPIO_REG_OUTCLR

        self.jlink.memory_write32(reg, [(1 << pin)])


    def _gpios_read(self):

        return self.jlink.memory_read32(NRF52_GPIO_REG_IN, 1)[0]

    def gpio_read(self, pin, pullup_config="none"):

        assert isinstance(pin, int), "Invalid type for pin - need int"
        assert pin < 32, "Need fixes before this code can handle GPIO P1"
        assert pin >= 0, "Invalid pin"

        # Pin must have the input buffer connected in order to read the pin
        # Defaults to disconnected
        pin_cnf = NRF52_GPIO_REG_PIN_CNF(pin)
        if pullup_config == "none":
            value = 0
        elif pullup_config == "up":
            value = (1 << 8)
        elif pullup_config == "down":
            value = (3 << 8)
        else:
            raise ValueError("pullup_config must be one of none, down, or up")

        self.jlink.memory_write32(pin_cnf, [value])
        
        return (self._gpios_read() & (1 << pin)) != 0


    def hfclk_status(self):

        hfclkstat = self.jlink.memory_read32(NRF52_CLOCK_REG_HFCLKSTAT, 1)[0]
        src = "XTAL" if (hfclkstat & 1) != 0 else "RC"
        running = (hfclkstat & (1 << 16) != 0)
        print(f"HFCLK running: {running}, type: {src}")

    def nfcpins_are_gpios(self):

        return (self.jlink.memory_read32(NRF52_UICR_REG_NFCPINS, 1)[0] & 1) == 0

    def spim_init(self, clk, mosi, miso, rate=NRF52_SPIM_FREQUENCY_8M, spim=0, cpol=0, cpha=0):

        if spim != 0:
            raise ValueError("Only tested with SPIM0")
        if cpol != 0 or cpha != 0:
            raise ValueError("Only tested with CPOL=CPHA=0")

        if clk:
            assert isinstance(clk, int) and (clk <= 31) and (clk >= 0), "Bad CLK pin (NC/Port 1 not allowed at the moment)"
        else:
            clk = 0xFFFFFFFF # Disconnected if high bit set

        if mosi:
            assert isinstance(mosi, int) and (mosi <= 31) and (mosi >= 0), "Bad MOSI pin (NC/Port 1 not allowed at the moment)"
        else:
            mosi = 0xFFFFFFFF # Disconnected if high bit set

        if miso:
            assert isinstance(miso, int) and (miso <= 31) and (miso >= 0), "Bad MISO pin (NC/Port 1 not allowed at the moment)"
        else:
            miso = 0xFFFFFFFF # Disconnected if high bit set

        base = NRF52_SPIM0_BASE

        # Before configuring the SPI peripheral, we have to drive the GPIO direction and values as specified in order to guarantee correct behaviour

        # CLK is an output, must have its output value set to match CPOL
        # According to nrfx_spim.c we also need the input buffer connected for correct functionality, but this doesn't seem to change anything (at least, not in TX-only mode?)
        self.gpio_write(clk, (False if cpol == 0 else True))
        self.gpio_cfg_full(clk, dir=GPIO_DIR_OUTPUT, input_buffer=GPIO_BUFFER_CONNECT, drive=GPIO_DRIVE_H0H1)

        # MOSI is an output, must default to 0
        self.gpio_write(mosi, False)
        self.gpio_cfg_full(mosi, dir=GPIO_DIR_OUTPUT, drive=GPIO_DRIVE_H0H1)

        # MISO is an input (TODO: do we need the buffer to be connected or not?)
        self.gpio_cfg_full(miso, dir=GPIO_DIR_INPUT, input_buffer=GPIO_BUFFER_CONNECT)

        # Set up the pins.
        self.jlink.memory_write32(base + NRF52_SPIM_OFFSET_SCK, [clk])
        self.jlink.memory_write32(base + NRF52_SPIM_OFFSET_MOSI, [mosi])
        self.jlink.memory_write32(base + NRF52_SPIM_OFFSET_MISO, [miso])

        # Configure the peripheral
        self.jlink.memory_write32(base + NRF52_SPIM_OFFSET_FREQUENCY, [rate])
        self.jlink.memory_write32(base + NRF52_SPIM_OFFSET_CONFIG, [0x00000000]) # Bit 0: MSB first (0), Bit 1: CPHA Leading (0), Bit 2: CPOL Active High (0)

        # Enable it
        self.jlink.memory_write32(base + NRF52_SPIM_OFFSET_ENABLE, [0x00000007]) # Enable SPIM 


    def spim_transfer(self, cs, data_out, data_in_len, spim=0, discard_incoming=False):

        assert spim==0, "Bad SPIM"
        assert len(data_out) < 255, "Bad length"

        # Check that the data length and specified CS pin are OK
        assert isinstance(cs, int) and (cs <= 31) and (cs >= 0), "Bad CS pin (NC/Port 1 not allowed at the moment)"

        if len(data_out) <= 1 and data_in_len == 1:
            print(f"nRF52 has an anomaly, these lengths (TX: {len(data_out)} and RX: {data_in_len}) will result in an extra byte being clocked out.")
            assert False,"Anomaly 58 would be triggered"

        base = NRF52_SPIM0_BASE

        transfer_len = max(len(data_out), data_in_len)

        # Copy the outgoing data over into suitable RAM block
        self.jlink.memory_write8(NRF52_DATA_RAM_BASE, data_out)

        # Set up the RX and TX pointers/lengths
        self.jlink.memory_write32(base + NRF52_SPIM_OFFSET_TXD_PTR, [NRF52_DATA_RAM_BASE])
        self.jlink.memory_write32(base + NRF52_SPIM_OFFSET_TXD_MAXCNT, [len(data_out)])
        rxd_location = NRF52_DATA_RAM_BASE + NRF52_DATA_RAM_LEN//2
        self.jlink.memory_write32(base + NRF52_SPIM_OFFSET_RXD_PTR, [rxd_location]) 
        self.jlink.memory_write32(base + NRF52_SPIM_OFFSET_RXD_MAXCNT, [data_in_len])

        # Clear any pending END event by writing 0 to it
        self.jlink.memory_write32(base + NRF52_SPIM_OFFSET_EVENTS_END, [0])

        # Set CS low
        self.gpio_write(cs, False)
        sleep(0.005)

        # Send the START task
        self.jlink.memory_write32(base + NRF52_SPIM_OFFSET_TASKS_START, [1])

        # Spin until we hit the END event
        while(self.jlink.memory_read32(base + NRF52_SPIM_OFFSET_EVENTS_END, 1)[0] == 0):
            sleep(0.005)

        # Clear the END event
        self.jlink.memory_write32(base + NRF52_SPIM_OFFSET_EVENTS_END, [0])

        # Clear CS to indicate completion
        self.gpio_write(cs, True)

        # Read TX/RX nums
        sent = self.jlink.memory_read32(base + NRF52_SPIM_OFFSET_TXD_AMOUNT, 1)[0]
        received = self.jlink.memory_read32(base + NRF52_SPIM_OFFSET_RXD_AMOUNT, 1)[0]

        if discard_incoming:
            # print(f"Sent {sent} bytes, discarded incoming")
            return

        # Copy the incoming data out of RAM
        data_in = self.jlink.memory_read8(rxd_location, data_in_len)
        # print(f"Sent {sent} bytes, received {received} bytes ({data_in})")

        # Wipe the RAM (TODO: WTF - why is this needed?!) then return the data
        # We do this because otherwise it seems we just read the same value as 
        # the previous transaction - some kind of caching / DMA flush issue?
        self.jlink.memory_write32(rxd_location, [0xFF] * transfer_len)
        self.jlink.memory_write32(NRF52_DATA_RAM_BASE, [0xFF] * transfer_len)
        return data_in



    def write_firmware(self, fname):

        if os.path.isfile(fname):
            true_filename = fname
        else:
            true_filename = (os.path.sep).join(os.path.realpath(__file__).split(os.path.sep)[:-1]) + "/" + fname

        assert (os.path.isfile(true_filename)), "Failed to find the firmware file."
        
        # Program the firmware, checking UICR and main executable were written correctly

        # Flash and UICR erase
        self.jlink.reset(halt=True)
        
        # Use the NVMC to quickly erase everything
        print("Erasing nRF52...")
        assert(self.jlink.memory_write32(addr=NRF52_NVMC_REG_CONFIG, data=[0x00000002]) == 4) # enable writing/erasing (CONFIG=0x2 - erase enabled)
        sleep(0.5)
        assert(self.jlink.memory_write32(addr=NRF52_NVMC_REG_ERASEALL, data=[0x00000001]) == 4) # Erase Flash (ERASEALL = 0x1)- CPU is halted during this operation
        sleep(0.5) # Erase-all time specified as max 295.3ms
        assert(self.jlink.memory_write32(addr=NRF52_NVMC_REG_ERASEUICR, data=[0x00000001]) == 4) # Erase UICR (ERASEUICR = 0x1)
        sleep(0.5)

        print("Programming nRF52...")
        self.jlink.flash_file(true_filename, addr=0) 
        
        # TODO: Read back verification
        # TODO: Verify this works as expected with UICR (ie bootloader works)

        # Exit programming mode - write protect flash again
        assert(self.jlink.memory_write32(addr=NRF52_NVMC_REG_CONFIG, data=[0x00000000]) == 4) # disable writing/erasing (CONFIG=0x0 - read-only)
        sleep(0.1) 

        # Reset again so that the UICR changes are taken into consideration
        self.jlink.reset(halt=True)
        sleep(0.1)

    def run_firmware(self):
        
        self.jlink.restart()

    def read_mac(self):

        # MAC address spans two words.
        deviceaddr = self.jlink.memory_read32(NRF52_FICR_REG_DEVICEADDR0, 2)

        # Convert from 2x 32-bit words to 6x bytes
        addr_bytes = [
            ((0xFF & (deviceaddr[1] >> 8)) | 0b11000000), # two bits of the MAC are set to indicate that it is a Random Static Address
            (0xFF & (deviceaddr[1])),
            (0xFF & (deviceaddr[0] >> 24)),
            (0xFF & (deviceaddr[0] >> 16)),
            (0xFF & (deviceaddr[0] >> 8)),
            (0xFF & (deviceaddr[0])) 
        ]

        # Return string representation of the MAC address
        return ":".join([hex(x).upper().split("X")[1] for x in addr_bytes])


    def read_details(self):
        
        # Info.Part = 0x52832, encoded as integer
        info_part = hex(self.jlink.memory_read32(NRF52_FICR_REG_INFO_PART, 1)[0]).split("x")[1]

        # Info.Variant = 4-byte ASCII-encoded string with no \0, representing the part ("AAB0" for example)
        info_variant_data = self.jlink.memory_read32(NRF52_FICR_REG_INFO_VARIANT, 1)[0]
        s = ""
        for i in range(4):
            s = s + chr((info_variant_data >> 8*i) & 0xFF)
        info_variant = s[::-1]

        # Info.Package
        package_codes = {
        0x2000: "QF",       # QFN
        0x2001: "CH",       # 7x8 WLCSP
        0x2002: "CI",       # 7x8 WLCSP
        0x2005: "CK"        # 7x8 WLCSP, backside coating for light protection
        }
        info_package = package_codes[self.jlink.memory_read32(NRF52_FICR_REG_INFO_PACKAGE, 1)[0]]

        # Info.Ram = one of 0x10 (16 kByte RAM), 0x20 (32 kByte RAM), 0x40 (64 kByte RAM)
        ram = self.jlink.memory_read32(NRF52_FICR_REG_INFO_RAM, 1)[0]

        # Info.Flash = one of 0x80 (128 kByte FLASH), 0x100 (256 kByte FLASH), 0x200 (512 kByte FLASH)
        flash = self.jlink.memory_read32(NRF52_FICR_REG_INFO_FLASH, 1)[0]

        return f"nRF{info_part}-{info_package}{info_variant} ({ram}kB RAM, {flash}kB FLASH)"


