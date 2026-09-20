/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/** \file
 * \brief
 * ESC hardware layer for the LAN9252 over Linux spidev, with optional
 * reset and interrupt lines driven through the gpiochip character device.
 *
 * Nothing here is Raspberry Pi specific: spidev and the GPIO chardev UAPI are
 * generic Linux interfaces, so this HAL runs on any Linux host wired to a
 * LAN9252 in SPI mode.
 */

#ifndef __esc_hw__
#define __esc_hw__

#include <stdint.h>

/** Hardware configuration, passed through esc_cfg_t.user_arg.
 *
 * This replaces the string token parsing used by the older bcm2835 HAL, which
 * could not carry a device path: it split on '.' and '-', which shreds
 * "/dev/spidev1.0". Point user_arg at one of these instead:
 *
 *    static const esc_hw_cfg_t hw = {
 *       .spidev        = "/dev/spidev1.0",
 *       .spi_speed_hz  = 20000000,
 *       .spi_mode      = 0,
 *       .gpiochip      = "/dev/gpiochip0",
 *       .irq_line      = 17,
 *       .reset_line    = 18,
 *       .op_timeout_ms = 100,
 *    };
 *    static esc_cfg_t config = { .user_arg = (void *)&hw, ... };
 */
typedef struct
{
   /** spidev node, e.g. "/dev/spidev1.0". The node is the chip select. */
   const char *spidev;
   /** SPI clock in Hz. The LAN9252 tops out at 30 MHz in plain serial mode. */
   uint32_t    spi_speed_hz;
   /** SPI mode; the LAN9252 serial interface uses mode 0. */
   uint8_t     spi_mode;
   /** GPIO chardev, e.g. "/dev/gpiochip0". NULL disables both GPIO lines. */
   const char *gpiochip;
   /** Offset of the line wired to the LAN9252 IRQ pin, or -1 if unused.
    *  Consumed in Phase 3; ignored while running polled. */
   int         irq_line;
   /** Offset of the line wired to the LAN9252 reset pin, or -1 if unused. */
   int         reset_line;
   /** Deadline applied to each hardware wait loop, in milliseconds.
    *  Every busy-wait in this HAL is bounded by it; 0 selects a 100 ms default. */
   uint32_t    op_timeout_ms;
} esc_hw_cfg_t;

void ESC_interrupt_enable (uint32_t mask);
void ESC_interrupt_disable (uint32_t mask);

/** True once a hardware wait loop has timed out.
 *
 * The HAL latches this and turns ESC_read and ESC_write into no-ops, so a
 * wedged chip drops the stack out of OP instead of spinning a core forever.
 */
int  ESC_hw_faulted (void);

/** Read a LAN9252 *system* register directly over SPI.
 *
 * System registers (BYTE_TEST 0x0064, ID_REV 0x0050, HW_CFG 0x0074, the reset
 * and PRAM control registers) live in the LAN9252's own address space and are
 * read with a plain serial-read command. They are NOT reachable through
 * ESC_read, which performs the EtherCAT CSR indirection via 0x0300/0x0304 and
 * therefore addresses the EtherCAT core's register space instead.
 *
 * Intended for bring-up diagnostics.
 */
uint32_t ESC_hw_sys_read32 (uint16_t address);

/** Write a LAN9252 system register directly over SPI. See ESC_hw_sys_read32. */
void ESC_hw_sys_write32 (uint16_t address, uint32_t value);

#endif
