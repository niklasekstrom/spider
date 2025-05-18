/*
 * Written in April 2025 by Niklas Ekström.
 *
 * SPIder exposes two SPI interfaces to the Amiga through the clockport.
 */
#include "hardware/gpio.h"
#include "hardware/spi.h"

#include "pico/critical_section.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include <stdio.h>

#include "protocol.h"

// SemVer: Major.Minor.Patch.
#define FIRMWARE_MAJOR_VERSION  1
#define FIRMWARE_MINOR_VERSION  0
#define FIRMWARE_PATCH_VERSION  0

// Identification.
#define IDENT_SIZE              8
#define IDENT_BYTES             {0xff, 's', 'p', 'd', 'r', FIRMWARE_MAJOR_VERSION, FIRMWARE_MINOR_VERSION, FIRMWARE_PATCH_VERSION}

#define DEBOUNCE_DOUBLE_READ    1
#define DEBOUNCE_DELAY_AFTER    1

// On the RP2040, setting both pulls enables a "bus keep" function,
// i.e. a weak pull to whatever is current high/low state of GPIO.
#define CTRL_PULL_UP            true
#define CTRL_PULL_DOWN          true

#define TEN_KHZ                 10000
#define ONE_MHZ                 1000000

#define SYS_CLOCK_KHZ           200000

#define SPI_RESET_FREQUENCY     (400*1000)

//      Pin name    GPIO    Direction   Comment     Description
#define PIN_D(x)    (0+x)   // In/out
#define PIN_RESET   8       // Input    Active low
#define PIN_INT6    9       // Output   Open collector
#define PIN_A(x)    (10+x)  // Input
#define PIN_RD      14      // Input    Active low
#define PIN_WR      15      // Input    Active low
#define PIN_MISO    16      // Input    Pull-up
#define PIN_SS      17      // Output   Active low
#define PIN_SCK     18      // Output
#define PIN_MOSI    19      // Output
#define PIN_CDET    20      // Input    Pull-up     Card Detect

// Application core commands.
#define APC_NOP                 0
#define APC_RESET               1
#define APC_UPPER_BYTE          2
#define APC_RX_DISCARD          3
#define APC_TX_FEED             4
#define APC_SPI_FREQ            5
#define APC_SLAVE_SELECT        6

static uint8_t ident_data[IDENT_SIZE] = IDENT_BYTES;
static uint8_t ident_index;

static volatile uint8_t registers[16];

static uint8_t rx_fifo[256];
static uint8_t tx_fifo[256];

static bool running;
static uint8_t in_flight;
static uint8_t upper_byte;

static uint32_t rx_discard_bytes_left;
static uint32_t tx_feed_bytes_left;

// #############################################################################
// ##### Logger macros
// #############################################################################

#define LOGGER_TRACE 0
#define logger_trace(...) do { if (LOGGER_TRACE) printf(__VA_ARGS__); } while (0)

#define LOGGER_DEBUG 0
#define logger_debug(...) do { if (LOGGER_DEBUG) printf(__VA_ARGS__); } while (0)

#define LOGGER_INFO 1
#define logger_info(...) do { if (LOGGER_INFO) printf(__VA_ARGS__); } while (0)

#define LOGGER_WARNING 1
#define logger_warning(...) do { if (LOGGER_WARNING) printf(__VA_ARGS__); } while (0)

#define LOGGER_ERROR 1
#define logger_error(...) do { if (LOGGER_ERROR) printf(__VA_ARGS__); } while (0)

// #############################################################################
// ##### These functions execute on the app core
// #############################################################################

static inline void set_int_fired(uint8_t data)
{
    if (registers[REG_INT_FIRED])
        return;

    if ((registers[REG_INT_ARMED] & data) == 0)
        return;

    gpio_set_dir_out_masked(1 << PIN_INT6);
    registers[REG_INT_FIRED] = 1;
}

static void handle_reset()
{
    logger_info("RESET\n");

    registers[REG_INT_ARMED] = 0;
    registers[REG_INT_FIRED] = 0;
    gpio_set_dir_in_masked(1 << PIN_INT6);

    running = false;
    in_flight = 0;
    upper_byte = 0;

    rx_discard_bytes_left = 0;
    tx_feed_bytes_left = 0;

    registers[REG_RX_HEAD] = 0;
    registers[REG_RX_TAIL] = 0;
    registers[REG_TX_HEAD] = 0;
    registers[REG_TX_TAIL] = 0;

    spi_set_baudrate(spi0, SPI_RESET_FREQUENCY);
}

static void handle_spi_freq(uint32_t data)
{
    uint32_t freq = data < 128 ? (data * TEN_KHZ) : ((data - 128) * ONE_MHZ);

    logger_info("SPI_FREQ, freq=%d\n", freq);

    spi_set_baudrate(spi0, freq);
}

static void handle_slave_select(uint32_t data)
{
    gpio_put(PIN_SS, data == 1 ? 0 : 1);
}

static void handle_upper_byte(uint32_t data)
{
    upper_byte = (uint8_t)data;
}

static void handle_rx_discard(uint32_t data)
{
    uint32_t increment = (upper_byte << 8) | (data & 0xff);
    rx_discard_bytes_left += increment;
    upper_byte = 0;
}

static void handle_tx_feed(uint32_t data)
{
    uint32_t increment = (upper_byte << 8) | (data & 0xff);
    tx_feed_bytes_left += increment;
    upper_byte = 0;
}

static void handle_apc_cmd()
{
    if (multicore_fifo_rvalid())
    {
        uint32_t value = sio_hw->fifo_rd;
        uint32_t cmd = value & 0xff;
        uint32_t data = value >> 8;

        switch (cmd)
        {
            case APC_RESET: handle_reset(); break;
            case APC_UPPER_BYTE: handle_upper_byte(data); break;
            case APC_RX_DISCARD: handle_rx_discard(data); break;
            case APC_TX_FEED: handle_tx_feed(data); break;
            case APC_SPI_FREQ: handle_spi_freq(data); break;
            case APC_SLAVE_SELECT: handle_slave_select(data); break;
            default: break;
        }

        running = cmd != APC_RESET;
    }
}

static void handle_gpio_interrupts()
{
    uint8_t card_detect = (gpio_get_all() & (1 << PIN_CDET)) == 0 ? 1 : 0;

    if (card_detect != registers[REG_CARD_DETECT])
    {
        registers[REG_CARD_DETECT] = card_detect;
        set_int_fired(IRQ_CD_CHANGED);
    }
}

static void handle_fifo_data()
{
    uint8_t rx_fifo_tail = registers[REG_RX_TAIL];

    // Read as many bytes as possible from SPI-RX.
    while (spi_is_readable(spi0))
    {
        if (rx_discard_bytes_left)
        {
            (void) spi_get_hw(spi0)->dr;
            rx_discard_bytes_left--;
        }
        else
        {
            uint8_t next_rx_fifo_tail = rx_fifo_tail + 1;
            if (next_rx_fifo_tail == registers[REG_RX_HEAD])
                break; // No space in RX-FIFO.

            rx_fifo[rx_fifo_tail] = (uint8_t) spi_get_hw(spi0)->dr;

            rx_fifo_tail = next_rx_fifo_tail;
            registers[REG_RX_TAIL] = next_rx_fifo_tail;
        }

        in_flight--;
    }

    uint8_t tx_fifo_head = registers[REG_TX_HEAD];

    // Write as many bytes as possible to SPI-TX.
    while (in_flight != 8 && spi_is_writable(spi0))
    {
        if (tx_fifo_head != registers[REG_TX_TAIL])
        {
            spi_get_hw(spi0)->dr = (uint32_t) tx_fifo[tx_fifo_head];

            tx_fifo_head++;
            registers[REG_TX_HEAD] = tx_fifo_head;
        }
        else if (tx_feed_bytes_left)
        {
            spi_get_hw(spi0)->dr = (uint32_t) 0xff;

            tx_feed_bytes_left--;
        }
        else
            break; // No bytes to write.

        in_flight++;
    }
}

static void drain_spi_rx()
{
    while (spi_is_readable(spi0))
    {
        (void) spi_get_hw(spi0)->dr;
    }
}

static void __not_in_flash_func(app_main)()
{
    while (1)
    {
        handle_apc_cmd();
        handle_gpio_interrupts();
        if (!running)
            drain_spi_rx();
        else
            handle_fifo_data();
    }
}

// #############################################################################
// ##### These functions execute on the bitbang core
// #############################################################################

static void __not_in_flash_func(bitbang_main)()
{
    uint32_t pins;

state_idle:
    while (1)
    {
        pins = gpio_get_all();
        if ((pins & (1 << PIN_WR)) == 0)
        {
#if DEBOUNCE_DOUBLE_READ
            pins = gpio_get_all();
            if ((pins & (1 << PIN_WR)) == 0)
#endif // DEBOUNCE_DOUBLE_READ
                goto state_write;
        }
        else if ((pins & (1 << PIN_RD)) == 0)
        {
#if DEBOUNCE_DOUBLE_READ
            pins = gpio_get_all();
            if ((pins & (1 << PIN_RD)) == 0)
#endif // DEBOUNCE_DOUBLE_READ
                goto state_read;
        }
        else if ((pins & (1 << PIN_RESET)) == 0)
        {
#if DEBOUNCE_DOUBLE_READ
            pins = gpio_get_all();
            if ((pins & (1 << PIN_RESET)) == 0)
#endif // DEBOUNCE_DOUBLE_READ
                goto state_reset;
        }
    }

state_read:
    {
        uint32_t address = (pins >> PIN_A(0)) & 0xf;

        if (address == REG_FIFO)
        {
            uint8_t rx_fifo_head = registers[REG_RX_HEAD];
            uint8_t data = rx_fifo[rx_fifo_head];

            gpio_set_dir_out_masked(0xff);
            gpio_set_mask(data);

            rx_fifo_head++;
            registers[REG_RX_HEAD] = rx_fifo_head;
        }
        else
        {
            uint8_t data = registers[address];

            gpio_set_dir_out_masked(0xff);
            gpio_set_mask(data);

            if (address == REG_IDENT)
            {
                ident_index = (ident_index + 1) & (IDENT_SIZE - 1);
                registers[REG_IDENT] = ident_data[ident_index];
            }
        }

        while (1)
        {
            pins = gpio_get_all();
            if ((pins & (1 << PIN_RD)) != 0)
            {
                gpio_set_dir_in_masked(0xff);
                gpio_clr_mask(0xff);

#if DEBOUNCE_DELAY_AFTER
                pins = gpio_get_all();
                pins = gpio_get_all();
#endif // DEBOUNCE_DELAY_AFTER
                goto state_idle;
            }
        }
    }

state_write:
    {
        uint32_t address = (pins >> PIN_A(0)) & 0xf;
        uint8_t data = pins & 0xff;

        if ((address & 8) == 0)
        {
            if ((address & 4) == 0)
            {
                if ((address & 2) == 0)
                {
                    if ((address & 1) == 0) // 0
                    {
                    }
                    else // 1
                    {
                    }
                }
                else
                {
                    if ((address & 1) == 0) // 2 = REG_UPPER_LENGTH
                    {
                        sio_hw->fifo_wr = (data << 8) | APC_UPPER_BYTE;
                    }
                    else // 3
                    {
                    }
                }
            }
            else
            {
                if ((address & 2) == 0)
                {
                    if ((address & 1) == 0) // 4
                    {
                    }
                    else // 5
                    {
                    }
                }
                else
                {
                    if ((address & 1) == 0) // 6
                    {
                    }
                    else // 7
                    {
                    }
                }
            }
        }
        else
        {
            if ((address & 4) == 0)
            {
                if ((address & 2) == 0)
                {
                    if ((address & 1) == 0) // 8 = REG_RX_DISCARD
                    {
                        sio_hw->fifo_wr = (data << 8) | APC_RX_DISCARD;
                    }
                    else // 9 = REG_TX_FEED
                    {
                        sio_hw->fifo_wr = (data << 8) | APC_TX_FEED;
                    }
                }
                else
                {
                    if ((address & 1) == 0) // 10 = REG_SPI_FREQ
                    {
                        sio_hw->fifo_wr = (data << 8) | APC_SPI_FREQ;
                    }
                    else // 11 = REG_SLAVE_SELECT
                    {
                        sio_hw->fifo_wr = (data << 8) | APC_SLAVE_SELECT;
                    }
                }
            }
            else
            {
                if ((address & 2) == 0)
                {
                    if ((address & 1) == 0) // 12 = REG_INT_FIRED
                    {
                        registers[REG_INT_FIRED] = 0;
                        gpio_set_dir_in_masked(1 << PIN_INT6);
                    }
                    else // 13 = REG_INT_ARMED
                    {
                        registers[REG_INT_ARMED] = data;
                    }
                }
                else
                {
                    if ((address & 1) == 0) // 14 = REG_FIFO
                    {
                        uint8_t tx_fifo_tail = registers[REG_TX_TAIL];
                        tx_fifo[tx_fifo_tail] = data;
                        tx_fifo_tail++;
                        registers[REG_TX_TAIL] = tx_fifo_tail;
                    }
                    else // 15 = REG_IDENT
                    {
                    }
                }
            }
        }

        while (1)
        {
            pins = gpio_get_all();
            if ((pins & (1 << PIN_WR)) != 0)
            {
#if DEBOUNCE_DELAY_AFTER
                pins = gpio_get_all();
                pins = gpio_get_all();
#endif // DEBOUNCE_DELAY_AFTER
                goto state_idle;
            }
        }
    }

state_reset:
    {
        // The only thing that is necessary to do here is to disable interrupts.
        // Later the Amiga will send a command to reset the app core.
        registers[REG_INT_ARMED] = 0;
        registers[REG_INT_FIRED] = 0;
        gpio_set_dir_in_masked(1 << PIN_INT6);

        sio_hw->fifo_wr = APC_RESET;

        while (1)
        {
            pins = gpio_get_all();
            if ((pins & (1 << PIN_RESET)) != 0)
            {
#if DEBOUNCE_DELAY_AFTER
                pins = gpio_get_all();
                pins = gpio_get_all();
#endif // DEBOUNCE_DELAY_AFTER
                goto state_idle;
            }
        }
    }
}

// #############################################################################
// ##### main is executed on core 0
// #############################################################################

int __not_in_flash_func(main)()
{
    stdio_init_all();

    // Configure gpio pins.

    spi_init(spi0, SPI_RESET_FREQUENCY);

    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_pull_up(PIN_MISO);

    gpio_init(PIN_SS);
    gpio_put(PIN_SS, 1);
    gpio_set_dir(PIN_SS, GPIO_OUT);

    gpio_init(PIN_CDET);
    gpio_pull_up(PIN_CDET);

    uint init_pins[] = {PIN_INT6, PIN_RD, PIN_WR, PIN_RESET};

    // Initialize D[7:0] and A[3:0].
    for (int i = 0; i < 16; i++)
    {
        gpio_init(i);
        gpio_disable_pulls(i);
    }

    // Initialize the control signals.
    for (int i = 0; i < 4; i++)
    {
        uint gpio = init_pins[i];
        gpio_init(gpio);
        if (gpio == PIN_INT6)
            gpio_disable_pulls(gpio);
        else
            gpio_set_pulls(gpio, CTRL_PULL_UP, CTRL_PULL_DOWN);
    }

    // Initialize state variables.
    running = false;
    in_flight = 0;
    upper_byte = 0;

    rx_discard_bytes_left = 0;
    tx_feed_bytes_left = 0;

    for (int i = 0; i < sizeof(registers); i++)
        registers[i] = 0;

    registers[REG_CARD_DETECT] = (gpio_get_all() & (1 << PIN_CDET)) == 0 ? 1 : 0;

    registers[REG_IDENT] = ident_data[0];
    ident_index = 0;

    // Turn up core frequency.
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);

    // Set SPI frequency based on new clock frequency.
    spi_set_baudrate(spi0, SPI_RESET_FREQUENCY);

    // Launch core inner loops.
    multicore_launch_core1(bitbang_main);
    app_main();
}
