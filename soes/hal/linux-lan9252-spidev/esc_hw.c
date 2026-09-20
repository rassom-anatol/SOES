/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/** \file
 * \brief
 * ESC hardware layer functions for the LAN9252 over Linux spidev.
 *
 * Ported from the bcm2835-based Raspberry Pi HAL. The LAN9252 register
 * protocol is unchanged; the transport is now spidev ioctls and the optional
 * reset line is driven through the GPIO character device, which removes the
 * root/\c /dev/mem requirement, the fixed clock dividers and the Pi-model
 * dependence of the original.
 */

#include "esc.h"
#include "esc_hw.h"

#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>

#define BIT(x)                   (1U << (x))

#define ESC_CMD_SERIAL_WRITE     0x02
#define ESC_CMD_SERIAL_READ      0x03

#define ESC_CMD_RESET_CTL        0x01F8      /* reset register */
#define ESC_CMD_HW_CFG           0x0074      /* hardware configuration register */
#define ESC_CMD_BYTE_TEST        0x0064      /* byte order test register */
#define ESC_CMD_ID_REV           0x0050      /* chip ID and revision */
#define ESC_CMD_IRQ_CFG          0x0054      /* interrupt configuration */
#define ESC_CMD_INT_EN           0x005C      /* interrupt enable */

/* Reset the EtherCAT core only. The original HAL had
 * (ESC_RESET_DIGITAL & ESC_RESET_ETHERCAT), which evaluates to zero: it wrote
 * 0 to the reset register and its wait loop exited on the first iteration, so
 * no reset ever happened. A full digital reset is also the wrong tool here,
 * since it re-latches the SPI mode strapping.
 */
#define ESC_RESET_CTRL_RST       BIT(6)
#define ESC_HW_CFG_READY         0x08000000
#define ESC_BYTE_TEST_OK         0x87654321

#define ESC_PRAM_RD_FIFO_REG     0x0000
#define ESC_PRAM_WR_FIFO_REG     0x0020
#define ESC_PRAM_RD_ADDR_LEN_REG 0x0308
#define ESC_PRAM_RD_CMD_REG      0x030C
#define ESC_PRAM_WR_ADDR_LEN_REG 0x0310
#define ESC_PRAM_WR_CMD_REG      0x0314

#define ESC_PRAM_CMD_BUSY        0x80000000
#define ESC_PRAM_CMD_ABORT       0x40000000
#define ESC_PRAM_CMD_AVAIL       0x00000001
#define ESC_PRAM_CMD_CNT(x)      (((x) >> 8) & 0x1F)
#define ESC_PRAM_SIZE(x)         ((uint32_t)(x) << 16)
#define ESC_PRAM_ADDR(x)         ((uint32_t)(x) << 0)

/* ESC_PRAM_CMD_CNT is five bits, so the FIFO reports at most 31 words. The
 * transfer is a 3 byte command header plus four bytes per word; sizing the
 * buffer for 32 removes the realloc the original performed on every batch in
 * the cyclic path, which no real-time path should contain.
 */
#define ESC_PRAM_FIFO_MAX        32
#define ESC_PRAM_BUF_SIZE        (3 + 4 * ESC_PRAM_FIFO_MAX)

#define ESC_CSR_DATA_REG         0x0300
#define ESC_CSR_CMD_REG          0x0304

#define ESC_CSR_CMD_BUSY         0x80000000
#define ESC_CSR_CMD_READ         (0x80000000 | 0x40000000)
#define ESC_CSR_CMD_WRITE        0x80000000
#define ESC_CSR_CMD_SIZE(x)      ((x) << 16)

#define DEFAULT_TIMEOUT_MS       100

static int      spi_fd   = -1;
static int      reset_fd = -1;
static uint32_t timeout_ms = DEFAULT_TIMEOUT_MS;

/* Latched once any hardware wait loop exceeds its deadline. Every busy-wait
 * below is bounded, and on expiry ESC_read/ESC_write stop touching the bus so
 * the stack falls out of OP rather than livelocking a core.
 */
static int hw_fault = 0;

int ESC_hw_faulted (void)
{
   return hw_fault;
}

/* Forward declarations; defined with the SPI primitives below. */
static uint32_t lan9252_read_32 (uint16_t address);
static void lan9252_write_32 (uint16_t address, uint32_t val);

uint32_t ESC_hw_sys_read32 (uint16_t address)
{
   if (hw_fault || spi_fd < 0)
   {
      return 0;
   }
   return lan9252_read_32 (address);
}

void ESC_hw_sys_write32 (uint16_t address, uint32_t value)
{
   if (hw_fault || spi_fd < 0)
   {
      return;
   }
   lan9252_write_32 (address, value);
}

/* ---------------------------------------------------------------- deadlines */

static void deadline_set (struct timespec * d, uint32_t ms)
{
   clock_gettime (CLOCK_MONOTONIC, d);
   d->tv_nsec += (long)(ms % 1000u) * 1000000L;
   d->tv_sec  += (time_t)(ms / 1000u);
   if (d->tv_nsec >= 1000000000L)
   {
      d->tv_nsec -= 1000000000L;
      d->tv_sec  += 1;
   }
}

static int deadline_passed (const struct timespec * d)
{
   struct timespec now;
   clock_gettime (CLOCK_MONOTONIC, &now);
   if (now.tv_sec != d->tv_sec)
   {
      return now.tv_sec > d->tv_sec;
   }
   return now.tv_nsec > d->tv_nsec;
}

/* Report a wait that ran out of time. Latches hw_fault so the caller and the
 * stack both stop trusting the bus.
 */
static void hw_timeout (const char * what)
{
   if (!hw_fault)
   {
      DPRINT ("lan9252: timeout waiting for %s, disabling ESC access\n", what);
      hw_fault = 1;
   }
}

/* ---------------------------------------------------------------- spi access */

/* One full-duplex transfer, in place: spidev supports tx_buf == rx_buf. */
static int spi_xfer (uint8_t * buf, uint32_t len)
{
   struct spi_ioc_transfer xfer;

   memset (&xfer, 0, sizeof (xfer));
   xfer.tx_buf = (unsigned long)buf;
   xfer.rx_buf = (unsigned long)buf;
   xfer.len    = len;

   if (ioctl (spi_fd, SPI_IOC_MESSAGE (1), &xfer) < 0)
   {
      hw_timeout ("spi transfer");
      return -1;
   }
   return 0;
}

static void lan9252_write_32 (uint16_t address, uint32_t val)
{
   uint8_t data[7];

   data[0] = ESC_CMD_SERIAL_WRITE;
   data[1] = (uint8_t)((address >> 8) & 0xFF);
   data[2] = (uint8_t)(address & 0xFF);
   data[3] = (uint8_t)(val & 0xFF);
   data[4] = (uint8_t)((val >> 8) & 0xFF);
   data[5] = (uint8_t)((val >> 16) & 0xFF);
   data[6] = (uint8_t)((val >> 24) & 0xFF);

   (void)spi_xfer (data, sizeof (data));
}

static uint32_t lan9252_read_32 (uint16_t address)
{
   uint8_t data[7];

   memset (data, 0, sizeof (data));
   data[0] = ESC_CMD_SERIAL_READ;
   data[1] = (uint8_t)((address >> 8) & 0xFF);
   data[2] = (uint8_t)(address & 0xFF);

   if (spi_xfer (data, sizeof (data)) < 0)
   {
      return 0;
   }

   return (((uint32_t)data[6] << 24) |
           ((uint32_t)data[5] << 16) |
           ((uint32_t)data[4] << 8) |
            (uint32_t)data[3]);
}

/* Poll a register until (value & mask) matches want, or the deadline passes.
 * Returns the last value read; sets hw_fault on expiry.
 */
static uint32_t wait_until (uint16_t reg, uint32_t mask, uint32_t want,
                            const char * what)
{
   struct timespec deadline;
   uint32_t value;

   deadline_set (&deadline, timeout_ms);
   for (;;)
   {
      value = lan9252_read_32 (reg);
      if (hw_fault)
      {
         return value;
      }
      if ((value & mask) == want)
      {
         return value;
      }
      if (deadline_passed (&deadline))
      {
         hw_timeout (what);
         return value;
      }
   }
}

/* ---------------------------------------------------------------- CSR access */

static void ESC_read_csr (uint16_t address, void *buf, uint16_t len)
{
   uint32_t value;

   value = (ESC_CSR_CMD_READ | ESC_CSR_CMD_SIZE (len) | address);
   lan9252_write_32 (ESC_CSR_CMD_REG, value);

   (void)wait_until (ESC_CSR_CMD_REG, ESC_CSR_CMD_BUSY, 0, "CSR read");
   if (hw_fault)
   {
      memset (buf, 0, len);
      return;
   }

   value = lan9252_read_32 (ESC_CSR_DATA_REG);
   memcpy (buf, (uint8_t *)&value, len);
}

static void ESC_write_csr (uint16_t address, void *buf, uint16_t len)
{
   uint32_t value = 0;

   memcpy ((uint8_t *)&value, buf, len);
   lan9252_write_32 (ESC_CSR_DATA_REG, value);
   value = (ESC_CSR_CMD_WRITE | ESC_CSR_CMD_SIZE (len) | address);
   lan9252_write_32 (ESC_CSR_CMD_REG, value);

   (void)wait_until (ESC_CSR_CMD_REG, ESC_CSR_CMD_BUSY, 0, "CSR write");
}

/* --------------------------------------------------------------- PRAM access */

/* Wait until the FIFO reports data available and at least fifo_range words
 * ready. Returns the command register value, or 0 on timeout.
 */
static uint32_t pram_wait_fifo (uint16_t reg, uint8_t fifo_range,
                                const char * what)
{
   struct timespec deadline;
   uint32_t value;

   deadline_set (&deadline, timeout_ms);
   for (;;)
   {
      value = lan9252_read_32 (reg);
      if (hw_fault)
      {
         return 0;
      }
      if ((value & ESC_PRAM_CMD_AVAIL) &&
          (ESC_PRAM_CMD_CNT (value) >= fifo_range))
      {
         return value;
      }
      if (deadline_passed (&deadline))
      {
         hw_timeout (what);
         return 0;
      }
   }
}

/* Number of 32 bit FIFO words needed to cover len bytes from first_byte_position. */
static uint8_t pram_fifo_range (uint16_t len, uint16_t byte_offset,
                                uint8_t first_byte_position)
{
   uint16_t quotient, remainder;

   if (byte_offset > 0)
   {
      quotient  = (uint16_t)(len / 4);
      remainder = (uint16_t)(len - quotient * 4);
   }
   else
   {
      quotient  = (uint16_t)((len + first_byte_position) / 4);
      remainder = (uint16_t)((len + first_byte_position) - quotient * 4);
   }
   if (remainder != 0)
   {
      quotient++;
   }
   return (uint8_t)MIN (quotient, 16);
}

static void ESC_read_pram (uint16_t address, void *buf, uint16_t len)
{
   uint32_t value;
   uint8_t * temp_buf = buf;
   uint16_t byte_offset = 0;
   uint8_t fifo_cnt, fifo_size, fifo_range, first_byte_position, temp_len;
   uint8_t buffer[ESC_PRAM_BUF_SIZE];
   int i;
   uint32_t size;

   lan9252_write_32 (ESC_PRAM_RD_CMD_REG, ESC_PRAM_CMD_ABORT);
   (void)wait_until (ESC_PRAM_RD_CMD_REG, ESC_PRAM_CMD_BUSY, 0, "PRAM read abort");
   if (hw_fault)
   {
      return;
   }

   lan9252_write_32 (ESC_PRAM_RD_ADDR_LEN_REG,
                     (ESC_PRAM_SIZE (len) | ESC_PRAM_ADDR (address)));
   lan9252_write_32 (ESC_PRAM_RD_CMD_REG, ESC_PRAM_CMD_BUSY);

   /* Find out first byte position and adjust the copy from that
    * according to LAN9252 datasheet and MicroChip SDK code
    */
   first_byte_position = (uint8_t)(address & 0x03);

   while (len > 0)
   {
      fifo_range = pram_fifo_range (len, byte_offset, first_byte_position);

      value = pram_wait_fifo (ESC_PRAM_RD_CMD_REG, fifo_range, "PRAM read FIFO");
      if (hw_fault)
      {
         return;
      }

      fifo_size = (uint8_t)ESC_PRAM_CMD_CNT (value);
      if (fifo_size > ESC_PRAM_FIFO_MAX)
      {
         fifo_size = ESC_PRAM_FIFO_MAX;
      }
      size = 3u + 4u * fifo_size;

      fifo_cnt = fifo_size;

      memset (buffer, 0, size);
      buffer[0] = ESC_CMD_SERIAL_READ;
      buffer[1] = (uint8_t)((ESC_PRAM_RD_FIFO_REG >> 8) & 0xFF);
      buffer[2] = (uint8_t)(ESC_PRAM_RD_FIFO_REG & 0xFF);

      if (spi_xfer (buffer, size) < 0)
      {
         return;
      }

      i = 3;
      while (fifo_cnt > 0 && len > 0)
      {
         value = (uint32_t)buffer[i] | ((uint32_t)buffer[i + 1] << 8) |
                 ((uint32_t)buffer[i + 2] << 16) | ((uint32_t)buffer[i + 3] << 24);

         if (byte_offset > 0)
         {
            temp_len = (uint8_t)((len > 4) ? 4 : len);
            memcpy (temp_buf + byte_offset, &value, temp_len);
         }
         else
         {
            temp_len = (uint8_t)((len > (4 - first_byte_position)) ?
                                 (4 - first_byte_position) : len);
            memcpy (temp_buf, ((uint8_t *)&value + first_byte_position), temp_len);
         }

         i += 4;
         fifo_cnt--;
         len = (uint16_t)(len - temp_len);
         byte_offset = (uint16_t)(byte_offset + temp_len);
      }
   }
}

static void ESC_write_pram (uint16_t address, void *buf, uint16_t len)
{
   uint32_t value;
   uint8_t * temp_buf = buf;
   uint16_t byte_offset = 0;
   uint8_t fifo_cnt, fifo_size, fifo_range, first_byte_position, temp_len;
   uint8_t buffer[ESC_PRAM_BUF_SIZE];
   int i;
   uint32_t size;

   lan9252_write_32 (ESC_PRAM_WR_CMD_REG, ESC_PRAM_CMD_ABORT);
   (void)wait_until (ESC_PRAM_WR_CMD_REG, ESC_PRAM_CMD_BUSY, 0, "PRAM write abort");
   if (hw_fault)
   {
      return;
   }

   lan9252_write_32 (ESC_PRAM_WR_ADDR_LEN_REG,
                     (ESC_PRAM_SIZE (len) | ESC_PRAM_ADDR (address)));
   lan9252_write_32 (ESC_PRAM_WR_CMD_REG, ESC_PRAM_CMD_BUSY);

   first_byte_position = (uint8_t)(address & 0x03);

   while (len > 0)
   {
      fifo_range = pram_fifo_range (len, byte_offset, first_byte_position);

      value = pram_wait_fifo (ESC_PRAM_WR_CMD_REG, fifo_range, "PRAM write FIFO");
      if (hw_fault)
      {
         return;
      }

      fifo_size = (uint8_t)ESC_PRAM_CMD_CNT (value);
      if (fifo_size > ESC_PRAM_FIFO_MAX)
      {
         fifo_size = ESC_PRAM_FIFO_MAX;
      }
      size = 3u + 4u * fifo_size;

      fifo_cnt = fifo_size;

      memset (buffer, 0, size);
      buffer[0] = ESC_CMD_SERIAL_WRITE;
      buffer[1] = (uint8_t)((ESC_PRAM_WR_FIFO_REG >> 8) & 0xFF);
      buffer[2] = (uint8_t)(ESC_PRAM_WR_FIFO_REG & 0xFF);

      i = 3;
      while (fifo_cnt > 0 && len > 0)
      {
         value = 0;
         if (byte_offset > 0)
         {
            temp_len = (uint8_t)((len > 4) ? 4 : len);
            memcpy (&value, (temp_buf + byte_offset), temp_len);
         }
         else
         {
            temp_len = (uint8_t)((len > (4 - first_byte_position)) ?
                                 (4 - first_byte_position) : len);
            memcpy (((uint8_t *)&value + first_byte_position), temp_buf, temp_len);
         }

         buffer[i]     = (uint8_t)(value & 0xFF);
         buffer[i + 1] = (uint8_t)((value >> 8) & 0xFF);
         buffer[i + 2] = (uint8_t)((value >> 16) & 0xFF);
         buffer[i + 3] = (uint8_t)((value >> 24) & 0xFF);

         i += 4;
         fifo_cnt--;
         len = (uint16_t)(len - temp_len);
         byte_offset = (uint16_t)(byte_offset + temp_len);
      }

      if (spi_xfer (buffer, size) < 0)
      {
         return;
      }
   }
}

/* --------------------------------------------------------- stack entry points */

/* Split a CSR access into transfers the LAN9252 accepts, per datasheet
 * Table 12-14 (EtherCAT CSR address vs size) and the Microchip SDK.
 */
static uint16_t csr_chunk_size (uint16_t address, uint16_t len)
{
   uint16_t size = (len > 4) ? 4 : len;

   if (address & BIT (0))
   {
      /* odd address: single byte only */
      size = 1;
   }
   else if (address & BIT (1))
   {
      size = (size & BIT (0)) ? 1 : 2;
   }
   else if (size == 3)
   {
      size = 1;
   }
   return size;
}

/** ESC read function used by the Slave stack.
 *
 * @param[in]   address     = address of ESC register to read
 * @param[out]  buf         = pointer to buffer to read in
 * @param[in]   len         = number of bytes to read
 */
void ESC_read (uint16_t address, void *buf, uint16_t len)
{
   if (hw_fault || spi_fd < 0)
   {
      memset (buf, 0, len);
      return;
   }

   if (address >= 0x1000)
   {
      ESC_read_pram (address, buf, len);
   }
   else
   {
      uint8_t *temp_buf = (uint8_t *)buf;

      while (len > 0)
      {
         uint16_t size = csr_chunk_size (address, len);
         ESC_read_csr (address, temp_buf, size);

         len = (uint16_t)(len - size);
         temp_buf += size;
         address = (uint16_t)(address + size);
      }
   }

   /* To mimic the ET1100 always providing AlEvent on every read or write */
   ESC_read_csr (ESCREG_ALEVENT, (void *)&ESCvar.ALevent, sizeof (ESCvar.ALevent));
   ESCvar.ALevent = etohs (ESCvar.ALevent);
}

/** ESC write function used by the Slave stack.
 *
 * @param[in]   address     = address of ESC register to write
 * @param[out]  buf         = pointer to buffer to write from
 * @param[in]   len         = number of bytes to write
 */
void ESC_write (uint16_t address, void *buf, uint16_t len)
{
   if (hw_fault || spi_fd < 0)
   {
      return;
   }

   if (address >= 0x1000)
   {
      ESC_write_pram (address, buf, len);
   }
   else
   {
      uint8_t *temp_buf = (uint8_t *)buf;

      while (len > 0)
      {
         uint16_t size = csr_chunk_size (address, len);
         ESC_write_csr (address, temp_buf, size);

         len = (uint16_t)(len - size);
         temp_buf += size;
         address = (uint16_t)(address + size);
      }
   }

   /* To mimic the ET1x00 always providing AlEvent on every read or write */
   ESC_read_csr (ESCREG_ALEVENT, (void *)&ESCvar.ALevent, sizeof (ESCvar.ALevent));
   ESCvar.ALevent = etohs (ESCvar.ALevent);
}

/* ------------------------------------------------------------------ GPIO reset */

/* Request one output line on the gpiochip and return its line-request fd. */
static int gpio_request_output (const char * chip, int offset, uint8_t value)
{
   struct gpio_v2_line_request req;
   int chip_fd;

   chip_fd = open (chip, O_RDONLY | O_CLOEXEC);
   if (chip_fd < 0)
   {
      DPRINT ("lan9252: cannot open %s\n", chip);
      return -1;
   }

   memset (&req, 0, sizeof (req));
   req.offsets[0]         = (uint32_t)offset;
   req.num_lines          = 1;
   req.config.flags       = GPIO_V2_LINE_FLAG_OUTPUT;
   req.config.num_attrs   = 1;
   req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
   req.config.attrs[0].attr.values = value ? 1u : 0u;
   req.config.attrs[0].mask = 1u;
   strncpy (req.consumer, "lan9252", sizeof (req.consumer) - 1);

   if (ioctl (chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0)
   {
      DPRINT ("lan9252: cannot request %s line %d\n", chip, offset);
      close (chip_fd);
      return -1;
   }

   close (chip_fd);
   return req.fd;
}

static void gpio_set (int fd, uint8_t value)
{
   struct gpio_v2_line_values vals;

   if (fd < 0)
   {
      return;
   }
   memset (&vals, 0, sizeof (vals));
   vals.mask = 1u;
   vals.bits = value ? 1u : 0u;
   (void)ioctl (fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals);
}

/* Pulse the reset line low, if one is wired. The LAN9252 can also be reset
 * over SPI, which ESC_init does unconditionally; a hardware line is stronger
 * and is used first when available.
 */
static void hw_reset_pulse (void)
{
   if (reset_fd < 0)
   {
      return;
   }
   gpio_set (reset_fd, 0);
   usleep (200);
   gpio_set (reset_fd, 1);
   usleep (1000);
}

/* ----------------------------------------------------------------------- init */

void ESC_reset (void)
{
   hw_reset_pulse ();
}

void ESC_init (const esc_cfg_t * config)
{
   const esc_hw_cfg_t * hw = (const esc_hw_cfg_t *)config->user_arg;
   uint8_t  bits = 8;
   uint8_t  mode;
   uint32_t speed;
   uint32_t value;

   hw_fault = 0;

   if (hw == NULL || hw->spidev == NULL)
   {
      DPRINT ("lan9252: user_arg must point at an esc_hw_cfg_t with a spidev path\n");
      hw_fault = 1;
      return;
   }

   timeout_ms = (hw->op_timeout_ms != 0) ? hw->op_timeout_ms : DEFAULT_TIMEOUT_MS;
   mode  = hw->spi_mode;
   speed = hw->spi_speed_hz;

   spi_fd = open (hw->spidev, O_RDWR | O_CLOEXEC);
   if (spi_fd < 0)
   {
      DPRINT ("lan9252: cannot open %s\n", hw->spidev);
      hw_fault = 1;
      return;
   }

   if (ioctl (spi_fd, SPI_IOC_WR_MODE, &mode) < 0 ||
       ioctl (spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
       ioctl (spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
   {
      DPRINT ("lan9252: cannot configure %s\n", hw->spidev);
      close (spi_fd);
      spi_fd = -1;
      hw_fault = 1;
      return;
   }

   /* Optional hardware reset line, released high before talking SPI. */
   if (hw->gpiochip != NULL && hw->reset_line >= 0)
   {
      reset_fd = gpio_request_output (hw->gpiochip, hw->reset_line, 1);
      hw_reset_pulse ();
   }

   /* Reset the EtherCAT core over SPI and wait for the bit to self-clear.
    * The original HAL polled the CSR command register here instead of the
    * reset register, so it never observed the reset completing.
    */
   lan9252_write_32 (ESC_CMD_RESET_CTL, ESC_RESET_CTRL_RST);
   (void)wait_until (ESC_CMD_RESET_CTL, ESC_RESET_CTRL_RST, 0, "core reset");

   /* Each wait below gets its own deadline. The original shared one counter
    * across all three, so an early slow step consumed the entire budget and
    * the later loops exited immediately on an unvalidated value.
    */
   (void)wait_until (ESC_CMD_BYTE_TEST, 0xFFFFFFFFu, ESC_BYTE_TEST_OK, "byte test");
   (void)wait_until (ESC_CMD_HW_CFG, ESC_HW_CFG_READY, ESC_HW_CFG_READY, "hw ready");

   if (hw_fault)
   {
      DPRINT ("lan9252: initialisation failed on %s\n", hw->spidev);
      return;
   }

   value = lan9252_read_32 (ESC_CMD_ID_REV);
   DPRINT ("lan9252: detected chip %x rev %u on %s at %u Hz\n",
           (unsigned)((value >> 16) & 0xFFFF), (unsigned)(value & 0xFFFF),
           hw->spidev, (unsigned)speed);

   value = (ESCREG_ALEVENT_CONTROL |
            ESCREG_ALEVENT_SMCHANGE |
            ESCREG_ALEVENT_SM0 |
            ESCREG_ALEVENT_SM1);
   ESC_ALeventmaskwrite (value);
}

/* ------------------------------------------------------------------ interrupts */

void ESC_interrupt_enable (uint32_t mask)
{
   uint32_t user_int_mask = ESCREG_ALEVENT_DC_SYNC0 |
                            ESCREG_ALEVENT_SM2 |
                            ESCREG_ALEVENT_SM3;
   if (mask & user_int_mask)
   {
      ESC_ALeventmaskwrite (ESC_ALeventmaskread () | (mask & user_int_mask));
   }

   /* LAN9252 IRQ pin as push-pull active high, then enable the interrupt.
    * Nothing observes the line until the Phase 3 gpiochip edge wait exists.
    */
   lan9252_write_32 (ESC_CMD_IRQ_CFG, 0x00000111);
   lan9252_write_32 (ESC_CMD_INT_EN, 0x00000001);
}

void ESC_interrupt_disable (uint32_t mask)
{
   uint32_t user_int_mask = ESCREG_ALEVENT_DC_SYNC0 |
                            ESCREG_ALEVENT_SM2 |
                            ESCREG_ALEVENT_SM3;

   if (mask & user_int_mask)
   {
      ESC_ALeventmaskwrite (ESC_ALeventmaskread () & ~(mask & user_int_mask));
   }

   lan9252_write_32 (ESC_CMD_INT_EN, 0x00000000);
}
