from time import sleep
import nrf52 as nrf5

WRITE_ENABLE = 0x06
WRITE_DISABLE = 0x04
READ_STATUS_REGISTER1 = 0x05       #(S7–S0)                                                                                                   *(2)
READ_STATUS_REGISTER2 = 0x07       #(S15-S8)                                                                                                  *(2)
WRITE_STATUS_REGISTER = 0x01       #(S7–S0)               (S15-S8)
PAGE_PROGRAM = 0x02                #A23–A16               A15–A8                  A7–A0                   (D7–D0)
QUAD_PAGE_PROGRAM = 0x32           #A23–A16               A15–A8                  A7–A0                   (D7–D0, …)                          *(3)
BLOCK_ERASE_64KB = 0xD8            #A23–A16               A15–A8                  A7–A0
BLOCK_ERASE_32KB = 0x52            #A23–A16               A15–A8                  A7–A0
SECTOR_ERASE_4KB = 0x20            #A23–A16               A15–A8                  A7–A0
CHIP_ERASE = 0xC7                  #No inputs or outputs                                                                                              Note: 0x60 should do the same.
ERASE_SUSPEND = 0x75               #No inputs or outputs
ERASE_RESUME = 0x7A                #No inputs or outputs
POWER_DOWN = 0xB9                  #No inputs or outputs
HIGH_PERFORMANCE_MODE = 0xA3       #dummy                 dummy                   dummy
MODE_BIT_RESET = 0xFF              #FFh                                                                                                       *(4)
HPM_DEVICE_ID = 0xAB               #dummy                 dummy                   dummy                   (ID7-ID0)                           *(5)    Note: also used for Release Power Down
MANUFACTURER_AND_DEVICE_ID = 0x90  #dummy                 dummy                   0x00                    (M7-M0)                 (ID7-ID0)
UNIQUE_ID = 0x4B                   #dummy                 dummy                   dummy                   dummy                   (ID63-ID0)
JEDEC_ID = 0x9F                    #(M7-M0)               (ID15-ID8)              (ID7-ID0)                                                           Note: (M7-M0) = Manufacturer, (ID15-ID8) = Memory Type, (ID7-ID0) = Capacity 

READ_DATA = 0x03

#  Reset commands - Cypress specific?
RESET_PT1 = 0x66
RESET_PT2 = 0x99

SR1_BUSY = (1 << 0)
SR1_WEL = (1 << 1)

SPI_FLASH_SECTOR_SIZE_BYTES = 4096

class spi_flash:

    def __init__(self, nrf52, cs):

        self.nrf52 = nrf52
        self.cs = cs

        self.nrf52.gpio_cfg_full(cs, dir=nrf5.GPIO_DIR_OUTPUT)
        self.nrf52.gpio_write(cs, True)
        sleep(0.05)

    def jedec_verify(self):
        # Read JEDEC data

        command = bytes([0x9F]) # Read JEDEC ID

        response = self.nrf52.spim_transfer(self.cs, command, 4)
        manuf_id = response[1]
        long_id = ((response[2] << 8) | response[3])

        assert(manuf_id == 0x01), f"Memory vendor {manuf_id} incorrect (expected Cypress: 0x01)"
        assert(long_id == 0x6017), f"Memory capacity or interface type {long_id} incorrect (expected 0x6017)"
        print("Memory ID OK")
        return

    def spi_flash_reset(self):

        command = bytes([RESET_PT1, RESET_PT2])
        self.nrf52.spim_transfer(self.cs, command, 1)
        self.nrf52.jlink.swd_sync()
        sleep(3)

    def spi_flash_status_reg_1_read(self):
    
        command = bytes([READ_STATUS_REGISTER1])

        response = self.nrf52.spim_transfer(self.cs, command, 2)
        #print(f"Status register 1: {response[1]}")
        return response[1]

    def spi_flash_status_reg_2_read(self):
    
        command = bytes([READ_STATUS_REGISTER2])

        response = self.nrf52.spim_transfer(self.cs, command, 2)
        # print(f"Status register 2: {response[1]}")
        return response[1]

    def read_cr2(self):

        command = bytes([0x15]) #RDCR2

        response = self.nrf52.spim_transfer(self.cs, command, 2)
        print(f"Config register 2: {response[1]}")
        return response[1]

    def spi_flash_write_enable(self):
    
        command = bytes([WRITE_ENABLE])

        self.nrf52.spim_transfer(self.cs, command, 0, discard_incoming=True) # RXD MAXCNT Must be 0 when 1 byte sent

        # Tiny delay ensures that CS has time to settle high before next transaction starts
        sleep(0.001);
        #print("After setting WEN")
        self.spi_flash_status_reg_1_read()

    def spi_flash_await_not_busy(self):

        #Poll BUSY flag until complete
        busy = True
        while(busy):
            busy = (self.spi_flash_status_reg_1_read() & SR1_BUSY) != 0;

        # print("No longer busy...")
        return True

    def spi_flash_erase_chip(self):
        print("Waiting for chip to be ready...")    
        self.spi_flash_await_not_busy()
        print("Erasing...")
        self.spi_flash_write_enable()
        command = bytes([CHIP_ERASE])
        self.nrf52.spim_transfer(self.cs, command, 0, discard_incoming=True);
        print("Erase in progress. This may take up to 150 seconds (Cypress 64MBit)")
        self.spi_flash_await_not_busy()
        print("Erase finished")
        
    #Erase a 4K page
    def spi_flash_erase_sector(self, sector_num):

        #Send the Write Enable command
        self.spi_flash_write_enable()

        #Winbond datasheet is unclear it wants the address of the first memory location within this sector (0-SPI_FLASH_CHIP_SIZE_BYTES, or the index of the sector (ie divide by SPI_FLASH_CHIP_SIZE_BYTES)
        #Other datasheets seem to imply it's the first memory location, and that the low 12 bits are unused, so we assume that here
        sector_address = sector_num << 12

        #Issue the Erase Sector command
        command = bytes([SECTOR_ERASE_4KB, (sector_address >> 16) & 0xFF, (sector_address >> 8) & 0xFF, sector_address & 0xFF])
        self.nrf52.spim_transfer(self.cs, command, 1);
        
        #Typical 45ms, max 400ms
        self.spi_flash_await_not_busy();



    #Write a 4K page into the Flash memory
    def spi_flash_write_sector(self, sector_num, data):

        # print(f"Writing data to sector number {sector_num}")

        start_addr = sector_num * SPI_FLASH_SECTOR_SIZE_BYTES

        # Split data into the largest chunks the SPIM can take
        for i in range(0, SPI_FLASH_SECTOR_SIZE_BYTES, 250)[:-1]:

            self.spi_flash_write_page(start_addr + i, data[i:i+250])

        # 4000 bytes written, now write the remaining 96 in the page
        self.spi_flash_write_page(start_addr + 4000, data[4000:])

        # print("Writing complete")


    #Writes can be done in chunks of 1-256 bytes, but due to SPI limitations on nRF52, the maximum length of an SPI transaction is 251 bytes.
    #If this represents a significant performance issue, we can work around, but this is simpler to understand.
    def spi_flash_write_page(self, address, data):

        assert len(data)<=250,"Data length unexpected"

        #Note that the page must have been erased previously (0xFF) otherwise the write will be invalid (bits can only be lowered by writing)

        #Send the Write Enable command
        self.spi_flash_write_enable();

        #Send the Program command and 24 bits of address
        #Note that the chip address can wrap around in certain circumstances!
        command = bytes([PAGE_PROGRAM, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF]) + bytes(data)
        #print(f"Programming with sequence {command}")

        #Send the command and data
        self.nrf52.spim_transfer(self.cs, command, 1);

        #Typical 0.7ms, max 3ms
        self.spi_flash_await_not_busy();


    def spi_flash_read_into_buffer(self, address, length):

        self.spi_flash_await_not_busy()

        assert length < 250

        command = bytes([READ_DATA, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF])

        #Send the command and read the data
        readBytes = self.nrf52.spim_transfer(self.cs, command, length + 4)[4:]

        return readBytes


