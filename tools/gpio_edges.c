/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/** \file
 * \brief
 * Count edges and sample the level on GPIO lines, with nothing else running.
 *
 * Written to tell signal from noise during bring-up. A line that is wired but
 * not yet driven floats, and a floating input on a flying lead next to a
 * clocking SPI bus can produce tens of thousands of spurious edges a second --
 * which looks exactly like a working signal if all you do is count.
 *
 * This deliberately touches no SPI, so any edges it sees are not coupling from
 * the bus this project drives. It also samples the level under each bias,
 * which can prove a line is floating but cannot prove the opposite -- see the
 * note in the level probe.
 *
 * Build:  gcc -O2 -Wall -o gpio_edges gpio_edges.c
 * Usage:  ./gpio_edges <gpiochip> <seconds> <line> [line ...]
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>

#define MAX_LINES 8

static int cmp_u64 (const void * a, const void * b)
{
   uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
   return (x < y) ? -1 : ((x > y) ? 1 : 0);
}

#define MAX_IV 200000

struct line
{
   int      offset;
   int      fd;
   uint64_t edges;
   uint64_t first_ns;
   uint64_t last_ns;
   uint64_t prev_ns;
   uint64_t iv[MAX_IV];     /* inter-edge intervals, kernel timestamped */
   uint32_t niv;
};

/* Request a line for both-edge events with a pull-down, matching how the HAL
 * watches IRQ and SYNC0. */
static int open_edges (const char * chip, int offset)
{
   struct gpio_v2_line_request req;
   int chip_fd, ret;

   chip_fd = open (chip, O_RDONLY | O_CLOEXEC);
   if (chip_fd < 0) return -1;

   memset (&req, 0, sizeof (req));
   req.offsets[0]   = (uint32_t)offset;
   req.num_lines    = 1;
   req.config.flags = GPIO_V2_LINE_FLAG_INPUT |
                      GPIO_V2_LINE_FLAG_EDGE_RISING |
                      GPIO_V2_LINE_FLAG_EDGE_FALLING |
                      GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN;
   strncpy (req.consumer, "gpio_edges", sizeof (req.consumer) - 1);

   ret = ioctl (chip_fd, GPIO_V2_GET_LINE_IOCTL, &req);
   close (chip_fd);
   return (ret < 0) ? -1 : req.fd;
}

/* Read the present level of a line, requested separately as a plain input. */
static int read_level (const char * chip, int offset, int pulldown)
{
   struct gpio_v2_line_request req;
   struct gpio_v2_line_values vals;
   int chip_fd, v = -1;

   chip_fd = open (chip, O_RDONLY | O_CLOEXEC);
   if (chip_fd < 0) return -1;

   memset (&req, 0, sizeof (req));
   req.offsets[0]   = (uint32_t)offset;
   req.num_lines    = 1;
   req.config.flags = GPIO_V2_LINE_FLAG_INPUT |
                      (pulldown ? GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN
                                : GPIO_V2_LINE_FLAG_BIAS_PULL_UP);
   strncpy (req.consumer, "gpio_edges", sizeof (req.consumer) - 1);

   if (ioctl (chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) == 0)
   {
      memset (&vals, 0, sizeof (vals));
      vals.mask = 1;
      if (ioctl (req.fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &vals) == 0)
      {
         v = (int)(vals.bits & 1u);
      }
      close (req.fd);
   }
   close (chip_fd);
   return v;
}

int main (int argc, char * argv[])
{
   const char * chip = (argc > 1) ? argv[1] : "/dev/gpiochip0";
   int seconds = (argc > 2) ? atoi (argv[2]) : 3;
   static struct line lines[MAX_LINES];
   struct pollfd pfd[MAX_LINES];
   int n = 0, i;
   struct timespec deadline, now;

   if (argc < 4)
   {
      printf ("usage: %s <gpiochip> <seconds> <line> [line ...]\n", argv[0]);
      return 2;
   }

   /* Probe levels first. A line can only be requested once, so this must
    * happen before the edge requests claim them -- otherwise every probe
    * fails with EBUSY and a naive comparison of two failures looks like
    * agreement. */
   printf ("levels before counting (no SPI traffic):\n");
   for (i = 3; i < argc; i++)
   {
      int off = atoi (argv[i]);
      int lo = read_level (chip, off, 1);
      int hi = read_level (chip, off, 0);
      if (lo < 0 || hi < 0)
      {
         printf ("  line %2d: could not read level (%s)\n", off, strerror (errno));
      }
      else
      {
         /* A line that follows the bias is certainly floating. The converse
          * does NOT hold: the internal bias is only about 50 kOhm, too weak to
          * drag a line floating near mid-rail across the logic threshold, so
          * an undriven line can read the same under both biases and look
          * driven. Confirmed on hardware -- a SYNC0 pin measured at 1.5 V on a
          * scope read 1 under both biases here. Treat agreement as
          * inconclusive and reach for a scope. */
         printf ("  line %2d: pull-down reads %d, pull-up reads %d  -> %s\n",
                 off, lo, hi,
                 (lo != hi) ? "floating (follows the bias)"
                            : "inconclusive: driven, or floating near mid-rail");
      }
   }
   printf ("\n");

   for (i = 3; i < argc && n < MAX_LINES; i++)
   {
      memset (&lines[n], 0, sizeof (lines[n]));
      lines[n].offset = atoi (argv[i]);
      lines[n].fd = open_edges (chip, lines[n].offset);
      if (lines[n].fd < 0)
      {
         printf ("line %d: cannot request (%s)\n", lines[n].offset,
                 strerror (errno));
         continue;
      }
      n++;
   }
   if (n == 0) return 1;

   printf ("\ncounting edges for %d s with the SPI bus idle...\n", seconds);
   clock_gettime (CLOCK_MONOTONIC, &deadline);
   deadline.tv_sec += seconds;

   for (;;)
   {
      struct gpio_v2_line_event ev[16];
      int rc;

      clock_gettime (CLOCK_MONOTONIC, &now);
      if (now.tv_sec > deadline.tv_sec ||
          (now.tv_sec == deadline.tv_sec && now.tv_nsec >= deadline.tv_nsec))
      {
         break;
      }

      for (i = 0; i < n; i++)
      {
         pfd[i].fd = lines[i].fd;
         pfd[i].events = POLLIN;
         pfd[i].revents = 0;
      }

      rc = poll (pfd, (nfds_t)n, 200);
      if (rc <= 0) continue;

      for (i = 0; i < n; i++)
      {
         if (pfd[i].revents & POLLIN)
         {
            ssize_t got = read (lines[i].fd, ev, sizeof (ev));
            int count = (got > 0) ? (int)(got / (ssize_t)sizeof (ev[0])) : 0;
            if (count > 0)
            {
               int k;
               if (lines[i].first_ns == 0)
               {
                  lines[i].first_ns = ev[0].timestamp_ns;
               }
               for (k = 0; k < count; k++)
               {
                  if (lines[i].prev_ns != 0 && lines[i].niv < MAX_IV)
                  {
                     lines[i].iv[lines[i].niv++] =
                        ev[k].timestamp_ns - lines[i].prev_ns;
                  }
                  lines[i].prev_ns = ev[k].timestamp_ns;
               }
               lines[i].last_ns = ev[count - 1].timestamp_ns;
               lines[i].edges += (uint64_t)count;
            }
         }
      }
   }

   printf ("\nresults:\n");
   for (i = 0; i < n; i++)
   {
      double span = (lines[i].last_ns > lines[i].first_ns)
                    ? (double)(lines[i].last_ns - lines[i].first_ns) / 1e9 : 0.0;
      printf ("  line %2d: %llu edges", lines[i].offset,
              (unsigned long long)lines[i].edges);
      if (span > 0.0)
      {
         printf ("  (%.0f/s over %.2f s)", (double)lines[i].edges / span, span);
      }
      if (lines[i].niv > 16)
      {
         uint64_t *v = lines[i].iv;
         uint32_t m = lines[i].niv;
         uint64_t sum = 0, j;
         for (j = 0; j < m; j++) sum += v[j];
         qsort (v, m, sizeof (uint64_t), cmp_u64);
         printf ("    interval  min %.1f  median %.1f  mean %.1f  "
                 "p99 %.1f  p99.9 %.1f  max %.1f us\n",
                 (double)v[0] / 1000.0,
                 (double)v[m / 2] / 1000.0,
                 (double)sum / (double)m / 1000.0,
                 (double)v[(m * 99) / 100] / 1000.0,
                 (double)v[(uint32_t)((uint64_t)m * 999 / 1000)] / 1000.0,
                 (double)v[m - 1] / 1000.0);
         printf ("    jitter    p99 %+.1f us, worst %+.1f us from the median\n",
                 ((double)v[(m * 99) / 100] - (double)v[m / 2]) / 1000.0,
                 ((double)v[m - 1] - (double)v[m / 2]) / 1000.0);
      }
      printf ("\n");
      close (lines[i].fd);
   }
   return 0;
}
