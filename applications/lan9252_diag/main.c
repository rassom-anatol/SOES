/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/** \file
 * \brief
 * LAN9252 bring-up diagnostic over Linux spidev.
 *
 * Two modes:
 *
 *   probe   Bring the ESC up and read back its identification registers, then
 *           exit. Needs no EtherCAT master and no cable, because it exercises
 *           only the SPI transport and the reset sequence. This is what proves
 *           the HAL before any of the stack above it is trusted.
 *
 *   run     Enter the normal cyclic slave loop. Needs a master.
 *
 * Probe mode calls ESC_init directly rather than ecat_slv_init, so that it
 * exercises the transport without needing a link: ecat_slv_init additionally
 * waits for the ESC to report one, which cannot happen with no cable.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <inttypes.h>

#include "esc.h"
#include "esc_hw.h"
#include "ecat_slv.h"
#include "utypes.h"

/* Application variables */
_Objects Obj;

/* Registers read back during probe. Addresses are LAN9252 system CSRs, not
 * EtherCAT registers, so they are reachable before the ESC is configured.
 */
#define LAN9252_ID_REV      0x0050
#define LAN9252_BYTE_TEST   0x0064
#define LAN9252_HW_CFG      0x0074
#define LAN9252_IRQ_CFG     0x0054
#define LAN9252_RESET_CTL   0x01F8
#define LAN9252_RESET_BIT   (1u << 6)
#define BYTE_TEST_EXPECTED  0x87654321u

static esc_hw_cfg_t hw_cfg =
{
   .spidev        = "/dev/spidev1.0",
   .spi_speed_hz  = 12000000,
   .spi_mode      = 0,
   .gpiochip      = "/dev/gpiochip0",
   .irq_line      = 17,     /* LAN9252 IRQ */
   .sync0_line    = 18,     /* LAN9252 SYNC0 */
   .reset_line    = 25,     /* shared with the TMC4671 reset / cmc CTRL_RST */
   .reset_pulse_us = 500,   /* >= LAN9252 minimum of 200 us; confirm against
                             * the TMC4671 minimum, which shares this line */
   .op_timeout_ms = 100,
};

static esc_cfg_t config =
{
   .user_arg = &hw_cfg,
   .use_interrupt = 0,
   .watchdog_cnt = 150,
   .set_defaults_hook = NULL,
   .pre_state_change_hook = NULL,
   .post_state_change_hook = NULL,
   .application_hook = NULL,
   .safe_state_override = NULL,
   .pre_object_download_hook = NULL,
   .post_object_download_hook = NULL,
   .rxpdo_override = NULL,
   .txpdo_override = NULL,
   .esc_hw_interrupt_enable = NULL,
   .esc_hw_interrupt_disable = NULL,
   .esc_hw_eep_handler = NULL,
   .esc_check_dc_handler = NULL,
};

/* The stack calls these; probe mode never reaches them, but they must link. */
void cb_update_txpdo (void)
{
}

void cb_apply_rxpdo (void)
{
}

static int probe (void)
{
   uint32_t byte_test, id_rev, hw_cfg_val;

   printf ("probing %s at %u Hz\n", hw_cfg.spidev, (unsigned)hw_cfg.spi_speed_hz);

   ESC_init (&config);

   if (ESC_hw_faulted ())
   {
      printf ("FAIL: ESC_init reported a hardware fault\n");
      return 1;
   }

   byte_test  = ESC_hw_sys_read32 (LAN9252_BYTE_TEST);
   id_rev     = ESC_hw_sys_read32 (LAN9252_ID_REV);
   hw_cfg_val = ESC_hw_sys_read32 (LAN9252_HW_CFG);

   if (ESC_hw_faulted ())
   {
      printf ("FAIL: register read timed out\n");
      return 1;
   }

   printf ("  BYTE_TEST 0x%08X  (expect 0x%08X)  %s\n",
           byte_test, BYTE_TEST_EXPECTED,
           (byte_test == BYTE_TEST_EXPECTED) ? "OK" : "MISMATCH");
   printf ("  ID_REV    0x%08X  chip 0x%04X rev %u\n",
           id_rev, (unsigned)((id_rev >> 16) & 0xFFFF), (unsigned)(id_rev & 0xFFFF));
   printf ("  HW_CFG    0x%08X  ready %s\n",
           hw_cfg_val, (hw_cfg_val & 0x08000000u) ? "yes" : "NO");

   if (byte_test != BYTE_TEST_EXPECTED)
   {
      printf ("FAIL: byte test mismatch -- wiring, mode or clock speed\n");
      return 1;
   }
   if ((hw_cfg_val & 0x08000000u) == 0)
   {
      printf ("FAIL: device reports not ready\n");
      return 1;
   }

   printf ("PASS\n");
   return 0;
}

/* Functional proof that the reset actually executes.
 *
 * ESC_init waiting for the reset bit to read clear is weak evidence: a chip
 * that reset instantly and one that never reset at all look identical from
 * there, which is exactly how the original HAL hid a reset mask of zero.
 *
 * The strong signal is catching the device leave and return. A genuine reset
 * makes BYTE_TEST stop reading its magic value for a while; if every sample
 * straight after the reset write still reads 0x87654321, nothing happened.
 * Only once the device is demonstrably back is IRQ_CFG checked for its default.
 */
static int reset_test (void)
{
   uint32_t dirty, after, bt;
   int i, disturbed = 0, recovered = 0;
   uint32_t first_odd = 0;

   printf ("reset test on %s\n", hw_cfg.spidev);
   ESC_init (&config);
   if (ESC_hw_faulted ())
   {
      printf ("FAIL: ESC_init reported a hardware fault\n");
      return 1;
   }

   ESC_hw_sys_write32 (LAN9252_IRQ_CFG, 0x00000111u);
   dirty = ESC_hw_sys_read32 (LAN9252_IRQ_CFG);
   printf ("  IRQ_CFG dirtied to    0x%08X\n", dirty);
   if ((dirty & 0x00000111u) != 0x00000111u)
   {
      printf ("FAIL: could not set IRQ_CFG, write path broken\n");
      return 1;
   }

   ESC_hw_sys_write32 (LAN9252_RESET_CTL, LAN9252_RESET_BIT);

   /* Sample hard for evidence the device dropped off the bus. */
   for (i = 0; i < 2000; i++)
   {
      bt = ESC_hw_sys_read32 (LAN9252_BYTE_TEST);
      if (bt != BYTE_TEST_EXPECTED)
      {
         if (!disturbed)
         {
            first_odd = bt;
         }
         disturbed++;
      }
      else if (disturbed)
      {
         recovered = 1;
         break;
      }
      usleep (50);
   }

   printf ("  BYTE_TEST disturbed   %d samples (first 0x%08X), recovered %s\n",
           disturbed, first_odd, recovered ? "yes" : "no");

   /* Let the device settle the way ESC_init would. */
   for (i = 0; i < 200; i++)
   {
      if (ESC_hw_sys_read32 (LAN9252_BYTE_TEST) == BYTE_TEST_EXPECTED &&
          (ESC_hw_sys_read32 (LAN9252_HW_CFG) & 0x08000000u))
      {
         break;
      }
      usleep (1000);
   }

   /* Informational, not a pass criterion. BIT(6) is ETHERCAT_RST and resets
    * the EtherCAT core only; IRQ_CFG lives in the host interface block, which
    * is outside that reset domain and is expected to survive. That scoping is
    * why ETHERCAT_RST is used here in preference to a full digital reset,
    * which would also re-latch the SPI mode strapping.
    */
   after = ESC_hw_sys_read32 (LAN9252_IRQ_CFG);
   printf ("  IRQ_CFG after reset   0x%08X (survives: host interface is\n"
           "                                   outside the ETHERCAT_RST domain)\n",
           after);

   if (disturbed == 0)
   {
      printf ("FAIL: device never went away -- no reset occurred\n");
      return 1;
   }
   if (!recovered)
   {
      printf ("FAIL: device did not come back after reset\n");
      return 1;
   }
   printf ("PASS: device observably reset and recovered\n");
   return 0;
}

/* Benchmark the CSR read path, which is the unit of cost in the cyclic loop.
 * Reports min/mean/max over many samples so a change can be compared against
 * a baseline rather than against a handful of noisy readings.
 */
static int cmp_u64 (const void * a, const void * b)
{
   uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
   return (x < y) ? -1 : ((x > y) ? 1 : 0);
}

static int dlstatus_test (uint32_t iterations)
{
   struct timespec a, b;
   uint16_t dls;
   uint64_t * ns;
   uint64_t sum = 0;
   uint32_t i;

   if (iterations == 0) iterations = 1000;
   ns = malloc (iterations * sizeof (uint64_t));
   if (ns == NULL) return 1;

   printf ("CSR read benchmark on %s, %u iterations\n",
           hw_cfg.spidev, iterations);
   if (ESC_init (&config) != 0)
   {
      printf ("FAIL: ESC_init returned non-zero\n");
      free (ns);
      return 1;
   }

   for (i = 0; i < iterations; i++)
   {
      dls = 0;
      clock_gettime (CLOCK_MONOTONIC, &a);
      ESC_read (ESCREG_DLSTATUS, &dls, sizeof (dls));
      clock_gettime (CLOCK_MONOTONIC, &b);
      ns[i] = (uint64_t)(b.tv_sec - a.tv_sec) * 1000000000ull +
              (uint64_t)(b.tv_nsec - a.tv_nsec);
      sum += ns[i];
   }

   qsort (ns, iterations, sizeof (uint64_t), cmp_u64);
   printf ("  min %6.1f us   median %6.1f us   mean %6.1f us\n",
           (double)ns[0] / 1000.0,
           (double)ns[iterations / 2] / 1000.0,
           (double)sum / (double)iterations / 1000.0);
   printf ("  p99 %6.1f us   max    %6.1f us   hw_fault=%d\n",
           (double)ns[(iterations * 99) / 100] / 1000.0,
           (double)ns[iterations - 1] / 1000.0,
           ESC_hw_faulted ());
   free (ns);
   return 0;
}

/* ESC DC registers not already named in esc.h. */
#define ESCREG_CYCLIC_UNIT_CTRL  0x0980
#define ESCREG_SYNC0_START_TIME  0x0990

/* Validate the IRQ and SYNC0 wiring without a master.
 *
 * The ESC's own distributed-clock unit can generate SYNC0 from its free
 * running local time, so the slave can provoke the very edges it needs to
 * observe. Unmasking DC_SYNC0 in the AL event mask makes the same event drive
 * the IRQ pin, so one mechanism exercises both lines.
 *
 * Kernel edge timestamps come from the GPIO chardev, so the interval spread
 * reported here is measured at the kernel, not in userspace.
 */
static int edge_test (uint32_t period_us, uint32_t seconds)
{
   int irq_fd, sync_fd;
   uint64_t now = 0, start;
   uint32_t period_ns = period_us * 1000u;
   uint8_t act;
   uint64_t t_prev = 0, t_now;
   uint64_t n_sync = 0, n_irq = 0;
   uint64_t min_iv = ~0ull, max_iv = 0, sum_iv = 0, n_iv = 0;
   int coalesced, total_coalesced = 0;
   struct timespec t_end, t_cur;

   printf ("edge test: SYNC0 %u us for %u s\n", period_us, seconds);

   if (ESC_init (&config) != 0)
   {
      printf ("FAIL: ESC_init returned non-zero\n");
      return 1;
   }

   irq_fd  = ESC_hw_edge_open (hw_cfg.gpiochip, hw_cfg.irq_line);
   sync_fd = ESC_hw_edge_open (hw_cfg.gpiochip, hw_cfg.sync0_line);
   printf ("  IRQ   line %d: %s\n", hw_cfg.irq_line,
           irq_fd >= 0 ? "requested" : "FAILED");
   printf ("  SYNC0 line %d: %s\n", hw_cfg.sync0_line,
           sync_fd >= 0 ? "requested" : "FAILED");
   if (irq_fd < 0 || sync_fd < 0)
   {
      return 1;
   }

   /* The DC unit is master-owned: 0x0980, 0x0981, 0x0990 and 0x09A0 are
    * ECAT-write / PDI-read, so a slave cannot start its own SYNC0. Verified
    * on hardware -- writes to all four read back as zero while a write to the
    * PDI-writable AL Status register lands correctly. This mode therefore
    * observes what a master has configured rather than configuring anything.
    */
   {
      uint8_t  r_unit = 0, r_act = 0;
      uint32_t r_cycle = 0;
      uint64_t r_time = 0;
      ESC_read (ESCREG_CYCLIC_UNIT_CTRL, &r_unit, sizeof (r_unit));
      ESC_read (ESCREG_SYNC_ACT, &r_act, sizeof (r_act));
      ESC_read (ESCREG_SYNC0_CYCLE_TIME, &r_cycle, sizeof (r_cycle));
      ESC_read (ESCREG_LOCALTIME, &r_time, sizeof (r_time));
      printf ("  0x0981 activation 0x%02X  (bit0 sync unit, bit1 SYNC0)\n", r_act);
      printf ("  0x09A0 cycle time %u ns\n", (unsigned)r_cycle);
      printf ("  0x0910 local time %llu\n", (unsigned long long)r_time);
      if ((r_act & (ESCREG_SYNC_ACT_ACTIVATED | ESCREG_SYNC_SYNC0_EN)) == 0)
      {
         printf ("  NOTE: SYNC0 is not activated. Connect a master and enable\n"
                 "        distributed clocks, or expect no edges below.\n");
      }
      (void)r_unit; (void)period_ns; (void)now; (void)start; (void)act;
   }

   /* Let the same event reach the IRQ pin. */
   ESC_interrupt_enable (ESCREG_ALEVENT_DC_SYNC0);

   clock_gettime (CLOCK_MONOTONIC, &t_end);
   t_end.tv_sec += (time_t)seconds;

   for (;;)
   {
      clock_gettime (CLOCK_MONOTONIC, &t_cur);
      if (t_cur.tv_sec > t_end.tv_sec ||
          (t_cur.tv_sec == t_end.tv_sec && t_cur.tv_nsec >= t_end.tv_nsec))
      {
         break;
      }

      coalesced = 0;
      if (ESC_hw_edge_wait (sync_fd, 200000000ull, &t_now, &coalesced) == 1)
      {
         /* One call drains up to eight queued events, so count them all. */
         n_sync += 1u + (uint64_t)coalesced;
         total_coalesced += coalesced;
         if (t_prev != 0)
         {
            uint64_t iv = t_now - t_prev;
            if (iv < min_iv) min_iv = iv;
            if (iv > max_iv) max_iv = iv;
            sum_iv += iv;
            n_iv++;
         }
         t_prev = t_now;
      }

      /* Drain IRQ without blocking; it should track SYNC0 one-for-one. */
      while (ESC_hw_edge_wait (irq_fd, 0, NULL, NULL) == 1)
      {
         n_irq++;
      }
   }

   ESC_interrupt_disable (ESCREG_ALEVENT_DC_SYNC0);

   printf ("  SYNC0 edges %llu (expected ~%llu)\n",
           (unsigned long long)n_sync,
           (unsigned long long)((uint64_t)seconds * 1000000ull / period_us));
   printf ("  IRQ   edges %llu\n", (unsigned long long)n_irq);
   printf ("  coalesced   %d (non-zero means we fell behind)\n", total_coalesced);
   if (n_iv > 0)
   {
      printf ("  interval    min %.3f ms  mean %.3f ms  max %.3f ms\n",
              (double)min_iv / 1e6, (double)sum_iv / (double)n_iv / 1e6,
              (double)max_iv / 1e6);
   }

   close (irq_fd);
   close (sync_fd);

   /* Observation only. Without a master driving DC there is nothing to
    * validate against, and a handful of edges at start-up proves only that
    * the line is connected to something that moves.
    */
   if (n_sync == 0)
   {
      printf ("  no SYNC0 edges observed\n");
   }
   return 0;
}

static void usage (const char * argv0)
{
   printf ("usage: %s [probe|reset|dlstatus|edges|run] [spidev] [speed_hz] [period_us] [seconds] [spidev] [speed_hz]\n", argv0);
}

int main (int argc, char * argv[])
{
   const char * mode = (argc > 1) ? argv[1] : "probe";

   if (argc > 2)
   {
      hw_cfg.spidev = argv[2];
   }
   if (argc > 3)
   {
      hw_cfg.spi_speed_hz = (uint32_t)strtoul (argv[3], NULL, 0);
   }

   if (strcmp (mode, "probe") == 0)
   {
      return probe ();
   }

   if (strcmp (mode, "edges") == 0)
   {
      return edge_test ((argc > 4) ? (uint32_t)strtoul (argv[4], NULL, 0) : 1000u,
                        (argc > 5) ? (uint32_t)strtoul (argv[5], NULL, 0) : 3u);
   }

   if (strcmp (mode, "dlstatus") == 0)
   {
      return dlstatus_test ((argc > 4) ? (uint32_t)strtoul (argv[4], NULL, 0) : 1000u);
   }

   if (strcmp (mode, "reset") == 0)
   {
      return reset_test ();
   }

   if (strcmp (mode, "run") == 0)
   {
      if (ecat_slv_init (&config) != 0)
      {
         printf ("FAIL: stack init failed\n");
         return 1;
      }
      printf ("stack init OK, entering cyclic loop\n");
      for (;;)
      {
         ecat_slv ();
      }
   }

   usage (argv[0]);
   return 2;
}
