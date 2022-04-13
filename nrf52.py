# Allow control of an nRF52-series IC by directly modifying peripheral registers
# This allows a SWD debugger to quickly prove board functionality
# This is just a small subset of the complete functionality
# It is designed to allow basic self-test and bringup functions only
# Initially developed for and tested on the nRF52832, has been extended to mostly support the nRF52840.
# Some of the logic may still need to be fixed to support GPIO port 1 / other features only present on newer ICs.

from time import sleep
import os

#
# GPIO
#
NRF52_GPIO_BASE = {
    0: 0x50000000,
    1: 0x50000300
}

NRF52_GPIO_OFFSET_REG_OUT      = 0x504
NRF52_GPIO_OFFSET_REG_OUTSET   = 0x508
NRF52_GPIO_OFFSET_REG_OUTCLR   = 0x50C
NRF52_GPIO_OFFSET_REG_IN       = 0x510
NRF52_GPIO_OFFSET_REG_DIR      = 0x514
NRF52_GPIO_OFFSET_REG_DIRSET   = 0x518
NRF52_GPIO_OFFSET_REG_DIRCLR   = 0x51C


def NRF52_GPIO_REG_PIN_CNF(port, pin):

    assert isinstance(pin, int), "Invalid type for pin - need int"
    assert pin < 32, "Need fixes before this code can handle GPIO P1"
    assert pin >= 0, "Invalid pin"
    
    return NRF52_GPIO_BASE[port] + 0x700 + (0x04 * pin)

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
# SPI Master (shares resources with TWI Master)
# 
NRF52_SPIM0_BASE = 0x40003000
NRF52_SPIM1_BASE = 0x40004000
NRF52_SPIM2_BASE = 0x40023000
NRF52_SPIM3_BASE = 0x4002F000 # 32 MBit, nRF52840

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
# TWI Master (shares resources with SPI Master}
#
NRF52_TWIM0_BASE = 0x40003000
NRF52_TWIM1_BASE = 0x40004000

NRF52_TWIM_OFFSET_TASKS_STARTRX    = 0x000
NRF52_TWIM_OFFSET_TASKS_STARTTX    = 0x008
NRF52_TWIM_OFFSET_TASKS_STOP       = 0x014
NRF52_TWIM_OFFSET_TASKS_SUSPEND    = 0x01C
NRF52_TWIM_OFFSET_TASKS_RESUME     = 0x020
NRF52_TWIM_OFFSET_EVENTS_STOPPED   = 0x104
NRF52_TWIM_OFFSET_EVENTS_ERROR     = 0x124
NRF52_TWIM_OFFSET_EVENTS_SUSPENDED = 0x148
NRF52_TWIM_OFFSET_EVENTS_RXSTARTED = 0x14C
NRF52_TWIM_OFFSET_EVENTS_TXSTARTED = 0x150
NRF52_TWIM_OFFSET_EVENTS_LASTRX    = 0x15C
NRF52_TWIM_OFFSET_EVENTS_LASTTX    = 0x160
NRF52_TWIM_OFFSET_SHORTS           = 0x200
NRF52_TWIM_OFFSET_INTEN            = 0x300
NRF52_TWIM_OFFSET_INTENSET         = 0x304
NRF52_TWIM_OFFSET_INTENCLR         = 0x308
NRF52_TWIM_OFFSET_ERRORSRC         = 0x4C4
NRF52_TWIM_OFFSET_ENABLE           = 0x500
NRF52_TWIM_OFFSET_PSEL_SCL         = 0x508
NRF52_TWIM_OFFSET_PSEL_SDA         = 0x50C
NRF52_TWIM_OFFSET_FREQUENCY        = 0x524
NRF52_TWIM_OFFSET_RXD_PTR          = 0x534
NRF52_TWIM_OFFSET_RXD_MAXCNT       = 0x538
NRF52_TWIM_OFFSET_RXD_AMOUNT       = 0x53C
NRF52_TWIM_OFFSET_RXD_LIST         = 0x540
NRF52_TWIM_OFFSET_TXD_PTR          = 0x544
NRF52_TWIM_OFFSET_TXD_MAXCNT       = 0x548
NRF52_TWIM_OFFSET_TXD_AMOUNT       = 0x54C
NRF52_TWIM_OFFSET_TXD_LIST         = 0x550
NRF52_TWIM_OFFSET_ADDRESS          = 0x588

NRF52_TWIM_FREQUENCY_K100 = 0x01980000
NRF52_TWIM_FREQUENCY_K250 = 0x04000000
NRF52_TWIM_FREQUENCY_K400 = 0x06400000

#
# SAADC
#
NRF52_SAADC_BASE = 0x40007000

NRF52_SAADC_NUM_CHANNELS = 8

NRF52_SAADC_REG_TASKS_START             = NRF52_SAADC_BASE + 0x000
NRF52_SAADC_REG_TASKS_SAMPLE            = NRF52_SAADC_BASE + 0x004
NRF52_SAADC_REG_TASKS_STOP              = NRF52_SAADC_BASE + 0x008
NRF52_SAADC_REG_TASKS_CALIBRATEOFFSET   = NRF52_SAADC_BASE + 0x00C
NRF52_SAADC_REG_EVENTS_STARTED          = NRF52_SAADC_BASE + 0x100
NRF52_SAADC_REG_EVENTS_END              = NRF52_SAADC_BASE + 0x104
NRF52_SAADC_REG_EVENTS_DONE             = NRF52_SAADC_BASE + 0x108
NRF52_SAADC_REG_EVENTS_RESULTDONE       = NRF52_SAADC_BASE + 0x10C
NRF52_SAADC_REG_EVENTS_CALIBRATEDONE    = NRF52_SAADC_BASE + 0x110
NRF52_SAADC_REG_EVENTS_STOPPED          = NRF52_SAADC_BASE + 0x114

def NRF52_SAADC_REG_EVENTS_CH_LIMIT(channel, high_or_low):
    assert channel in range(NRF52_SAADC_NUM_CHANNELS), "Invalid ADC channel"
    assert high_or_low.lower() in ["h", "l", "high", "low"], "High or low must be one of 'h', 'l', 'high' or 'low'"

    is_high = True if high_or_low.lower() in ["h", "high"] else False

    return NRF52_SAADC_BASE + 0x118 + 8 * channel + (0 if is_high else 4)

# Check for bugs with a couple of randomly-chosen examples
assert NRF52_SAADC_REG_EVENTS_CH_LIMIT(6, "h") == NRF52_SAADC_BASE + 0x148, "Internal bug in SAADC register calculations"
assert NRF52_SAADC_REG_EVENTS_CH_LIMIT(1, "l") == NRF52_SAADC_BASE + 0x124, "Internal bug in SAADC register calculations"

NRF52_SAADC_REG_INTEN                   = NRF52_SAADC_BASE + 0x300
NRF52_SAADC_REG_INTENSET                = NRF52_SAADC_BASE + 0x304
NRF52_SAADC_REG_INTENCLR                = NRF52_SAADC_BASE + 0x308
NRF52_SAADC_REG_STATUS                  = NRF52_SAADC_BASE + 0x400
NRF52_SAADC_REG_ENABLE                  = NRF52_SAADC_BASE + 0x500


def NRF52_SAADC_REG_CH_CONFIG_BASE(channel):
    assert channel in range(NRF52_SAADC_NUM_CHANNELS), "Invalid ADC channel"

    return NRF52_SAADC_BASE + 0x510 + (16 * channel)
# Add to the above to get the register address
NRF52_SAADC_OFFSET_PSELP                = 0x0
NRF52_SAADC_OFFSET_PSELN                = 0x4
NRF52_SAADC_OFFSET_CONFIG               = 0x8
NRF52_SAADC_OFFSET_LIMIT                = 0xC

NRF52_SAADC_REG_RESOLUTION              = NRF52_SAADC_BASE + 0x5F0
NRF52_SAADC_REG_OVERSAMPLE              = NRF52_SAADC_BASE + 0x5F4
NRF52_SAADC_REG_SAMPLERATE              = NRF52_SAADC_BASE + 0x5F8
NRF52_SAADC_REG_RESULT_PTR              = NRF52_SAADC_BASE + 0x62C
NRF52_SAADC_REG_RESULT_MAXCNT           = NRF52_SAADC_BASE + 0x630
NRF52_SAADC_REG_RESULT_AMOUNT           = NRF52_SAADC_BASE + 0x634

# RESP and RESN bits
saadc_res = {
    "bypass": 0,
    "pulldown": 1,
    "pullup": 2,
    "vdd1_2": 3
}

# Convert gain to GAIN bits value
saadc_gain = {
    "gain1_6": 0,
    "gain1_5": 1,
    "gain1_4": 2,
    "gain1_3": 3,
    "gain1_2": 4,
    "gain1": 5,
    "gain2": 6,
    "gain4": 7
}

# Convert time in microseconds to TACQ bits value
saadc_acq_time = {
    3: 0,
    5: 1,
    10: 2,
    15: 3,
    20: 4,
    40: 5
}

saadc_inputs = {
    "NC": 0,
    "AnalogInput0": 1,
    "AnalogInput1": 2,
    "AnalogInput2": 3,
    "AnalogInput3": 4,
    "AnalogInput4": 5,
    "AnalogInput5": 6,
    "AnalogInput6": 7,
    "AnalogInput7": 8,
    "VDD": 9
}

# Resolution, in bits, to VAL for RESOLUTION
saadc_resolution = {
    8: 0,
    10: 1,
    12: 2,
    14: 3
}

# map from a saadc_inputs value to a GPIO pin number
saadc_input_to_gpio_pin = {
    "AnalogInput0": 2,
    "AnalogInput1": 3,
    "AnalogInput2": 4,
    "AnalogInput3": 5,
    "AnalogInput4": 28,
    "AnalogInput5": 29,
    "AnalogInput6": 30,
    "AnalogInput7": 31,
}




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
NRF52_UICR_REG_PSELRESET0       = NRF52_UICR_BASE + 0x200
NRF52_UICR_REG_PSELRESET1       = NRF52_UICR_BASE + 0x204
NRF52_UICR_REG_APPROTECT        = NRF52_UICR_BASE + 0x208
NRF52_UICR_REG_NFCPINS          = NRF52_UICR_BASE + 0x20C
NRF52_UICR_REG_DEBUGCTRL        = NRF52_UICR_BASE + 0x210
NRF52_UICR_REG_REGOUT0          = NRF52_UICR_BASE + 0x304


class nrf52:

    def __init__(self, jlink):

        self.jlink = jlink

    # Specify full configuration of GPIO pin. Defaults match reset state.
    def gpio_cfg_full(self, pin, direction=GPIO_DIR_INPUT, input_buffer=GPIO_BUFFER_DISCONNECT, pull=GPIO_PULL_NONE, drive=GPIO_DRIVE_S0S1, sense=0):
        
        assert pin >= 0, "Invalid pin"
        assert isinstance(pin, int), "Invalid type for pin - need int"
        assert (pin not in [9, 10]) or self.nfcpins_are_gpios(), "Pin is set up for NFC, cannot be used for GPIO"
        
        port = pin // 32
        pin = pin % 32
        
        if(direction == "input"):
                direction = GPIO_DIR_INPUT
        if(direction == "output"):
                direction = GPIO_DIR_OUTPUT
                
        if(input_buffer == "disconnect"):
                input_buffer = GPIO_BUFFER_DISCONNECT
        if(input_buffer == "connect"):
                input_buffer = GPIO_BUFFER_CONNECT
        
        if(pull == "up"):
                pull = GPIO_PULL_UP
        if(pull == "down"):
                pull = GPIO_PULL_DOWN
        if(pull == "none"):
                pull = GPIO_PULL_NONE
        
        # The PINCNF register handles everything. No need to write DIR, it's the same physical register.
        cnf = (direction) + (input_buffer << 1) + (pull << 2) + (drive << 8) + (sense << 16)

        self.jlink.memory_write32(NRF52_GPIO_REG_PIN_CNF(port, pin), [cnf])


    def gpio_write(self, pin, state):

        assert isinstance(pin, int), "Invalid type for pin - need int"
        assert pin >= 0, "Invalid pin"
        assert (pin not in [9, 10]) or self.nfcpins_are_gpios(), "Pin is set up for NFC, cannot be used for GPIO"

        port = pin // 32
        pin = pin % 32

        if state:
            reg = NRF52_GPIO_BASE[port] + NRF52_GPIO_OFFSET_REG_OUTSET
        else:
            reg = NRF52_GPIO_BASE[port] + NRF52_GPIO_OFFSET_REG_OUTCLR

        self.jlink.memory_write32(reg, [(1 << pin)])


    def _gpio_port_read(self, port):

        return self.jlink.memory_read32(NRF52_GPIO_BASE[port] + NRF52_GPIO_OFFSET_REG_IN, 1)[0]

    def gpio_read(self, pin):

        assert isinstance(pin, int), "Invalid type for pin - need int"
        assert pin >= 0, "Invalid pin"
        
        port = pin // 32
        pin = pin % 32
        
        return (self._gpio_port_read(port) & (1 << pin)) != 0

    def saadc_configure(self, resolution_bits=10, oversample_power=0, samplerate_cc=80, samplerate_mode=0):

        assert resolution_bits in saadc_resolution.keys(), f"Invalid ADC resolution {resolution_bits}, expected one of {saadc_resolution.keys()}"
        assert oversample_power in range(8+1), "Oversampling power only available in range 0 to 8"
        assert samplerate_cc in range(80, 2048), "Samplerate must be in the range 80 to 2047 inclusive"
        assert samplerate_mode in range(0, 1), "Samplerate mode must be 0 or 1"

        # Set up RESOLUTION
        self.jlink.memory_write32(NRF52_SAADC_REG_RESOLUTION, [saadc_resolution[resolution_bits]])

        # Set up OVERSAMPLE
        self.jlink.memory_write32(NRF52_SAADC_REG_OVERSAMPLE, [oversample_power])

        # Set up SAMPLERATE
        self.jlink.memory_write32(NRF52_SAADC_REG_SAMPLERATE, [samplerate_cc | (samplerate_mode << 12)])

        pass

    def saadc_channel_configure(self, channel_adc, input_p, input_n="NC", resistor_p="bypass", resistor_n="bypass", gain="gain1", internalReference=True, acqTime=10, differential=False, burst=False):

        resistor_p = resistor_p.lower()
        resistor_n = resistor_n.lower()
        gain = gain.lower()

        assert channel_adc in range(NRF52_SAADC_NUM_CHANNELS), f"Invalid ADC channel: {channel_p}, must be less than {NRF52_SAADC_NUM_CHANNELS}"
        assert input_p in saadc_inputs.keys(), f"Invalid input pin for positive input to ADC: {input_p}, expected one of {saadc_inputs.keys()}"
        assert input_n in saadc_inputs.keys(), f"Invalid input pin for negative input to ADC: {input_n}, expected one of {saadc_inputs.keys()}"
        assert resistor_p in saadc_res.keys(), f"Invalid ADC positive channel resistor: {resistor_p}, expected one of {saadc_res.keys()}"
        assert resistor_n in saadc_res.keys(), f"Invalid ADC negative channel resistor: {resistor_n}, expected one of {saadc_res.keys()}"
        assert gain in saadc_gain.keys(), f"Invalid gain {gain}, expected one of {saadc_gain.keys()}"
        assert acqTime in saadc_acq_time.keys(), f"Invalid acquisition time {acqTime}, expected one of {saadc_acq_time.keys()}"

        # Configure the positive input channel, CH[x].PSELP
        pselp = NRF52_SAADC_REG_CH_CONFIG_BASE(channel_adc) + NRF52_SAADC_OFFSET_PSELP
        value = saadc_inputs[input_p]
        self.jlink.memory_write32(pselp, [value])

        # Configure the negative input channel, CH[x].PSELN
        pseln = NRF52_SAADC_REG_CH_CONFIG_BASE(channel_adc) + NRF52_SAADC_OFFSET_PSELN
        value = saadc_inputs[input_n]
        self.jlink.memory_write32(pseln, [value])

        # CH[x].CONFIG
        config = NRF52_SAADC_REG_CH_CONFIG_BASE(channel_adc) + NRF52_SAADC_OFFSET_CONFIG
        value = (
            ((saadc_res[resistor_p]) << 0) | # RESP
            ((saadc_res[resistor_n]) << 4) | # RESN
            ((saadc_gain[gain]) << 8) | # GAIN
            ((0 if internalReference else 1) << 12) | # REFSEL
            ((saadc_acq_time[acqTime]) << 16) | # TACQ
            ((1 if differential else 0) << 20) | # MODE
            ((1 if burst else 0) << 24) # BURST
        )
        self.jlink.memory_write32(config, [value])

        # If using an external input (not NC or VCC), also configure the GPIO pin / buffer appropriately
        if input_p not in ["NC", "VDD"]:
            pin = saadc_input_to_gpio_pin[input_p]
            self.gpio_cfg_full(pin) # Default settings are OK for ADC

        if input_n not in ["NC", "VDD"]:
            pin = saadc_input_to_gpio_pin[input_n]
            self.gpio_cfg_full(pin) # Default settings are OK for ADC

    def saadc_channel_deconfigure(self, channel_adc):

        assert channel_adc in range(NRF52_SAADC_NUM_CHANNELS), f"Invalid ADC channel: {channel_p}, must be less than {NRF52_SAADC_NUM_CHANNELS}"

        # A channel is only considered enabled if PSELP is set, the rest of the configuration can be left.
        pselp = NRF52_SAADC_REG_CH_CONFIG_BASE(channel_adc) + NRF52_SAADC_OFFSET_PSELP
        value = saadc_inputs["NC"]
        self.jlink.memory_write32(pselp, [value])


    def saadc_calibrate(self):

        # Enable the SAADC
        self.jlink.memory_write32(NRF52_SAADC_REG_ENABLE, [1])

        # Clear any flags that might have been left over
        self.jlink.memory_write32(NRF52_SAADC_REG_EVENTS_CALIBRATEDONE, [0])

        # Fire the TASKS_CALIBRATEOFFSET task
        self.jlink.memory_write32(NRF52_SAADC_REG_TASKS_CALIBRATEOFFSET, [1])

        # Await the EVENTS_CALIBRATEDONE event
        while(self.jlink.memory_read32(NRF52_SAADC_REG_EVENTS_CALIBRATEDONE, 1)[0] == 0):
            sleep(0.005)

        # Clear the event again
        self.jlink.memory_write32(NRF52_SAADC_REG_EVENTS_CALIBRATEDONE, [0])

        # Disable the SAADC
        self.jlink.memory_write32(NRF52_SAADC_REG_ENABLE, [0])



    def saadc_measure_once(self, channel):

        assert channel in range(NRF52_SAADC_NUM_CHANNELS), f"Invalid ADC channel {channel}"

        # Enable the SAADC
        self.jlink.memory_write32(NRF52_SAADC_REG_ENABLE, [1])

        # One-shot conversion is configured by enabling only one of the available channels (by setting PSELP value on one channel only)
        # Upon a SAMPLE task, the ADC starts to sample the input voltage. The CH[n].CONFIG.TACQ controls the acquisition time.
        # A DONE event signals that one sample has been taken.
        # In this mode, the RESULTDONE event has the same meaning as DONE when no oversampling takes place.
        # Note that both events may occur before the actual value has been transferred into RAM by EasyDMA. For more information, see EasyDMA.

        # TODO: Ensure we're only configured with one PSELP value set (ie we will perform a single conversion only)

        # Configure the read operation - read into a temporary RAM buffer
        self.jlink.memory_write32(NRF52_SAADC_REG_RESULT_PTR, [NRF52_DATA_RAM_BASE])
        self.jlink.memory_write32(NRF52_SAADC_REG_RESULT_MAXCNT, [1])
        

        # Enable the correct channel
        # TODO: This
        # self.jlink.memory_write32(NRF52_SAA....)

        # Clear the event signals
        self.jlink.memory_write32(NRF52_SAADC_REG_EVENTS_STARTED, [0])
        self.jlink.memory_write32(NRF52_SAADC_REG_EVENTS_END, [0])

        # Start the conversion by firing the START task
        self.jlink.memory_write32(NRF52_SAADC_REG_TASKS_START, [1])
        
        # Await the STARTED event
        while(self.jlink.memory_read32(NRF52_SAADC_REG_EVENTS_STARTED, 1)[0] == 0):
            sleep(0.005)
            
        # Fire the SAMPLE task
        self.jlink.memory_write32(NRF52_SAADC_REG_TASKS_SAMPLE, [1])
        
        # Await the END event
        while(self.jlink.memory_read32(NRF52_SAADC_REG_EVENTS_END, 1)[0] == 0):
            sleep(0.005)

        # Clear the event signals. This doubles as a tiny delay to allow the DMA transfer to complete before reading the value.
        self.jlink.memory_write32(NRF52_SAADC_REG_EVENTS_STARTED, [0])
        self.jlink.memory_write32(NRF52_SAADC_REG_EVENTS_END, [0])

        # Read the result from the buffer. Samples are always recorded as 16-bit values, even if the resolution is lower
        value = self.jlink.memory_read16(NRF52_DATA_RAM_BASE, 1)[0]
        received = self.jlink.memory_read32(NRF52_SAADC_REG_RESULT_AMOUNT, 1)[0]
        
        assert received==1, "Error whilst reading from the ADC"

        # Disable the SAADC
        self.jlink.memory_write32(NRF52_SAADC_REG_ENABLE, [0])

        # Clear out our temp memory. Not sure why required.
        self.jlink.memory_write32(NRF52_DATA_RAM_BASE, [0xFF] * 1)

        return value

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

    def twim_read(self, scl, sda, addr, reg, length, freq=400, twim=0):
        
        assert twim==0, "Alternate TWIMs not yet supported"
        assert freq==400, "Alternate TWI frequencies not yet supported"
        assert addr <= 127 and addr >= 0, "Address out of range"
        
        base = NRF52_TWIM0_BASE
        frequency = NRF52_TWIM_FREQUENCY_K400
        
        # Set up the PSEL.SCL, PSEL.SDA, FREQUENCY and the ADDRESS[n] register
        self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_PSEL_SCL, [scl])
        self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_PSEL_SDA, [sda])
        self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_FREQUENCY, [frequency])
        self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_ADDRESS, [addr])
        
        # Address should be in the lowest 7 bits (the peripheral handles READ/WRITE for us)
        
        # Also configure S0D1 drive on both pins (only necessary for correct low-power performance?)
        
        # Also set Direction to Input (only necessary for correct low-power performance?)
                
        # We use the SHORTS system to connect tasks and events together.
        # This ensures that we don't need to rely on clock stretching
        # and works fine for the case where we simply write register address
        # then read out a set of data. It also means we don't need any code or interrupt handling
        # on the CPU.
        # Connect LASTTX event to STARTRX task (LASTTX_STARTRX - bit 7)
        # Connect LASTRX event to STOP task (LASTRX_STOP - bit 12)
        shorts = (1 << 7) | (1 << 12)
        self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_SHORTS, [shorts])

        
        # Set TX.MAXCNT=1, RX.MAXCNT=length
        self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_TXD_MAXCNT, [1])
        self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_RXD_MAXCNT, [length])
        
        # Set TX from DMA-capable RAM, RX to a separate block of DMA-capable RAM
        txd_location = NRF52_DATA_RAM_BASE
        rxd_location = NRF52_DATA_RAM_BASE + NRF52_DATA_RAM_LEN//2
        self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_TXD_PTR, [txd_location])
        self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_RXD_PTR, [rxd_location])
        
        # Write the I2C target's register to the first byte of TX memory, to be transmitted
        self.jlink.memory_write8(txd_location, [reg])
        
        # Clear out any old events
        self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_EVENTS_STOPPED, [0])
        self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_EVENTS_ERROR, [0])
        
        # Enable the peripheral
        self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_ENABLE, [6])
        sleep(0.005)
            
        # Trigger STARTTX task
        self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_TASKS_STARTTX, [1])
        
        # Poll for STOPPED and ERROR events
        stopped = 0
        err = 0
        while(err == 0 and stopped == 0):
            stopped = self.jlink.memory_read32(base + NRF52_TWIM_OFFSET_EVENTS_STOPPED, 1)[0]
            err = self.jlink.memory_read32(base + NRF52_TWIM_OFFSET_EVENTS_ERROR, 1)[0]
            sleep(0.005)
            
        # Ensure they're consistent (if the fault occurred betwen the first and second read, we might miss the err being set)
        stopped = self.jlink.memory_read32(base + NRF52_TWIM_OFFSET_EVENTS_STOPPED, 1)[0]
        err = self.jlink.memory_read32(base + NRF52_TWIM_OFFSET_EVENTS_ERROR, 1)[0]
        
        # Clear the events
        self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_EVENTS_STOPPED, [0])
        self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_EVENTS_ERROR, [0])
        
        # If an ERROR occurred, find the type of error, print diagnostic, and return None
        if(err != 0):
            errsrc = self.jlink.memory_read32(base + NRF52_TWIM_OFFSET_ERRORSRC, 1)[0]
            self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_ERRORSRC, [0])
            flags = ""
            if(errsrc & 0b1):
                flags += "OVERRUN "
            if(errsrc & 0b10):
                flags += "ANACK "
            if(errsrc & 0b100):
                flags += "DNACK "
                
            print(f"TWI ERROR event fired. Flags: {flags}")
            
            # Disable the peripheral
            self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_ENABLE, [0])

            return None
        
        # If the TXD_AMOUNT or RXD_AMOUNT does not match the expected length, something went wrong?
        # TODO: Handle errors
        sent = self.jlink.memory_read32(base + NRF52_TWIM_OFFSET_TXD_AMOUNT, 1)[0]
        received = self.jlink.memory_read32(base + NRF52_TWIM_OFFSET_RXD_AMOUNT, 1)[0]
        #print(f"TWI completed - sent {sent}, received {received}")
        
        # Read out the data
        data_in = self.jlink.memory_read8(rxd_location, length)
        
        # Clear the data (needed for some reason? Copied from SPI)
        self.jlink.memory_write32(rxd_location, [0xFF] * 256)
        self.jlink.memory_write32(txd_location, [0xFF] * 256)
        
        # Disable the peripheral
        self.jlink.memory_write32(base + NRF52_TWIM_OFFSET_ENABLE, [0])
        
        return data_in
        
        

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

    def verify_regout(self):

        vout_table = {
            0: "1.8v",
            1: "2.1v",
            2: "2.4v",
            3: "2.7v",
            4: "3.0v",
            5: "3.3v",
            7: "1.8v (default)"
        }

        vout = (self.jlink.memory_read32(NRF52_UICR_REG_REGOUT0, 1)[0]) & 0b111
        
        return vout_table[vout]


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
        return ":".join(["%02X" % x for x in addr_bytes])


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
        0x2004: "QI",       # 73-pin aQFN
        0x2005: "CK",       # 7x8 WLCSP, backside coating for light protection
        }
        info_package = package_codes[self.jlink.memory_read32(NRF52_FICR_REG_INFO_PACKAGE, 1)[0]]

        # Info.Ram = one of 0x10 (16 kByte RAM), 0x20 (32 kByte RAM), 0x40 (64 kByte RAM)
        ram = self.jlink.memory_read32(NRF52_FICR_REG_INFO_RAM, 1)[0]

        # Info.Flash = one of 0x80 (128 kByte FLASH), 0x100 (256 kByte FLASH), 0x200 (512 kByte FLASH)
        flash = self.jlink.memory_read32(NRF52_FICR_REG_INFO_FLASH, 1)[0]

        return f"nRF{info_part}-{info_package}{info_variant} ({ram}kB RAM, {flash}kB FLASH)"


