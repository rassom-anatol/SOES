/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/** \file
 * \brief
 * Measure the cost of a spidev transaction, by clock rate and message shape.
 *
 * Written to size the cyclic budget of a motion controller that talks to an
 * EtherCAT slave controller on one SPI bus and a motor controller on another.
 * Two questions it answers:
 *
 *   1. What does one register access actually cost at a given clock? Wire time
 *      is easy to compute; the syscall and driver overhead around it is not,
 *      and on short frames it dominates.
 *
 *   2. Is a multi-transfer message cheap? Some devices need a pause between
 *      address and data -- the TMC4671 wants 500 ns for its 8 MHz read mode --
 *      which in spidev means two transfers in one SPI_IOC_MESSAGE with
 *      delay_usecs. On the BCM2711 AUX controller that form measured markedly
 *      slower than separate messages, because its chip select is a GPIO the
 *      driver toggles. The main spi-bcm2835 controller uses a hardware chip
 *      select, so the result may differ; this measures it rather than assuming.
 *
 * The device on the other end does not affect the timing: an unattached bus
 * clocks out the same bits in the same time and simply reads back zeros. Only
 * correctness needs a real device.
 *
 * Build:  gcc -O2 -Wall -o spidev_bench spidev_bench.c
 * Usage:  ./spidev_bench [device] [iterations]
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define MAX_ITERS 200000

static int cmp_u64 (const void * a, const void * b)
{
   uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
   return (x < y) ? -1 : ((x > y) ? 1 : 0);
}

static uint64_t now_ns (void)
{
   struct timespec t;
   clock_gettime (CLOCK_MONOTONIC, &t);
   return (uint64_t)t.tv_sec * 1000000000ull + (uint64_t)t.tv_nsec;
}

/* Run one shape n times and report the distribution.
 *
 * Reports p99 as well as the median because a cyclic control loop is limited
 * by its worst case, not its typical case: a tail that appears once in a
 * hundred cycles still misses one cycle in a hundred.
 */
static void run (int fd, const char * label, uint32_t hz,
                 struct spi_ioc_transfer * xfer, int n_xfer, uint32_t iters,
                 uint64_t * out)
{
   static uint64_t samples[MAX_ITERS];
   uint64_t sum = 0, t0;
   uint32_t i;
   int failed = 0;

   if (ioctl (fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0)
   {
      printf ("  %-28s cannot set %u Hz\n", label, hz);
      return;
   }

   for (i = 0; i < iters; i++)
   {
      t0 = now_ns ();
      if (ioctl (fd, SPI_IOC_MESSAGE (n_xfer), xfer) < 0)
      {
         failed = 1;
         break;
      }
      samples[i] = now_ns () - t0;
      sum += samples[i];
   }

   if (failed)
   {
      printf ("  %-28s transfer failed: %s\n", label, strerror (errno));
      return;
   }

   qsort (samples, iters, sizeof (uint64_t), cmp_u64);
   printf ("  %-28s min %6.1f  median %6.1f  mean %6.1f  p99 %6.1f  max %7.1f us\n",
           label,
           (double)samples[0] / 1000.0,
           (double)samples[iters / 2] / 1000.0,
           (double)sum / (double)iters / 1000.0,
           (double)samples[(iters * 99) / 100] / 1000.0,
           (double)samples[iters - 1] / 1000.0);

   if (out != NULL)
   {
      *out = samples[iters / 2];
   }
}

int main (int argc, char * argv[])
{
   const char * dev = (argc > 1) ? argv[1] : "/dev/spidev0.0";
   uint32_t iters = (argc > 2) ? (uint32_t)strtoul (argv[2], NULL, 0) : 20000;
   const uint32_t clocks[] = { 1000000, 2000000, 4000000, 8000000 };
   uint8_t buf[8];
   int fd, c;
   uint8_t mode = 3;          /* the TMC4671 uses mode 3 */
   uint8_t bits = 8;

   if (iters > MAX_ITERS) iters = MAX_ITERS;

   fd = open (dev, O_RDWR);
   if (fd < 0)
   {
      printf ("cannot open %s\n", dev);
      return 1;
   }
   if (ioctl (fd, SPI_IOC_WR_MODE, &mode) < 0 ||
       ioctl (fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0)
   {
      printf ("cannot configure %s\n", dev);
      close (fd);
      return 1;
   }

   printf ("spidev benchmark on %s, mode %u, %u iterations per shape\n\n",
           dev, mode, iters);

   for (c = 0; c < (int)(sizeof (clocks) / sizeof (clocks[0])); c++)
   {
      struct spi_ioc_transfer one[1], split[2];
      char label[64];
      uint32_t hz = clocks[c];

      printf ("%u MHz  (40-bit datagram = %.1f us of wire time)\n",
              hz / 1000000u, 40.0 * 1e6 / (double)hz);

      /* Single 5-byte datagram: the TMC4671's normal register access. */
      memset (one, 0, sizeof (one));
      memset (buf, 0, sizeof (buf));
      one[0].tx_buf = (unsigned long)buf;
      one[0].rx_buf = (unsigned long)buf;
      one[0].len    = 5;
      snprintf (label, sizeof (label), "5-byte single transfer");
      run (fd, label, hz, one, 1, iters, NULL);

      /* Address byte, 500 ns pause, then 4 data bytes: the shape required by
       * the TMC4671's 8 MHz read mode. delay_usecs is microsecond granular,
       * so 1 us is the smallest pause expressible and is >= the 500 ns needed. */
      memset (split, 0, sizeof (split));
      split[0].tx_buf      = (unsigned long)buf;
      split[0].rx_buf      = (unsigned long)buf;
      split[0].len         = 1;
      split[0].delay_usecs = 1;
      split[0].cs_change   = 0;
      split[1].tx_buf      = (unsigned long)(buf + 1);
      split[1].rx_buf      = (unsigned long)(buf + 1);
      split[1].len         = 4;
      snprintf (label, sizeof (label), "1+4 split, 1us pause");
      run (fd, label, hz, split, 2, iters, NULL);

      /* 7 bytes, for comparison with the LAN9252 CSR frame on the AUX bus. */
      memset (one, 0, sizeof (one));
      one[0].tx_buf = (unsigned long)buf;
      one[0].rx_buf = (unsigned long)buf;
      one[0].len    = 7;
      snprintf (label, sizeof (label), "7-byte single transfer");
      run (fd, label, hz, one, 1, iters, NULL);

      printf ("\n");
   }

   close (fd);
   return 0;
}
