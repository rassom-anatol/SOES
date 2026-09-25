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
#include <pthread.h>
#include <sched.h>

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

static void cb_state_change (uint8_t * as, uint8_t * an);

/* Observed process data and callback counts, reported by the run loop. */
static int sync_pin_cfg = -1;   /* value to try writing to 0x0151 */
static int sync_thread_prio = -1;  /* >=0 spawns the SYNC0 wake-latency thread */
static pthread_t sync_tid;
static int sync_thread_running = 0;

/* Wake latency samples, written by the sync thread and read by the reporter.
 * Deliberately lock-free and approximate: a torn read costs one wrong sample
 * in a statistics run, and a mutex here would be the very interference the
 * measurement is trying to characterise.
 */
#define LAT_MAX 100000
static volatile uint64_t wl_ns[LAT_MAX];
static volatile uint32_t wl_n = 0;
static volatile uint64_t wl_timeouts = 0;
static volatile int      sync_thread_stop = 0;

/* Block on the SYNC0 edge and record how long after the kernel timestamped it
 * this thread actually resumed.
 *
 * This is the figure that decides SyncErrorCounterLimit and whether a 1 ms
 * cycle is safe: the signal itself is already known to be stable to +-4 us
 * (roadmap 3.3.0), so what remains is scheduling. It cannot be measured from
 * the cyclic thread, because a polling loop discovers the edge whenever it
 * next looks rather than when it arrived.
 */
static void * sync_latency_thread (void * arg)
{
   int fd = *(int *)arg;
   struct timespec now;

   if (sync_thread_prio > 0)
   {
      struct sched_param sp;
      memset (&sp, 0, sizeof (sp));
      sp.sched_priority = sync_thread_prio;
      if (pthread_setschedparam (pthread_self (), SCHED_FIFO, &sp) != 0)
      {
         printf ("sync thread: SCHED_FIFO %d refused, running SCHED_OTHER\n",
                 sync_thread_prio);
      }
      else
      {
         printf ("sync thread: SCHED_FIFO %d\n", sync_thread_prio);
      }
   }

   while (!sync_thread_stop)
   {
      uint64_t t_edge = 0, t_now;
      int coalesced = 0;
      int rc = ESC_hw_edge_wait (fd, 50000000ull, &t_edge, &coalesced);

      if (rc == 0) { wl_timeouts++; continue; }
      if (rc < 0)  { break; }

      clock_gettime (CLOCK_MONOTONIC, &now);
      t_now = (uint64_t)now.tv_sec * 1000000000ull + (uint64_t)now.tv_nsec;

      if (t_now > t_edge && wl_n < LAT_MAX)
      {
         wl_ns[wl_n] = t_now - t_edge;
         wl_n = wl_n + 1;
      }
   }
   return NULL;
}
static volatile uint8_t  rx_mirror = 0;
static volatile uint64_t rx_calls = 0;
static volatile uint64_t tx_calls = 0;

static esc_hw_cfg_t hw_cfg =
{
   .spidev        = "/dev/spidev1.0",
   .spi_speed_hz  = 25000000,  /* measured knee; see roadmap 3.4 */
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
   /* The software counter stays as a fallback for a master that disables the
    * hardware watchdog. 150 polls of a loop that runs as fast as it can is not
    * a meaningful timeout, which is the reason to prefer the hardware one:
    * 0x0440 is reset by the master's own SM2 writes, so it measures elapsed
    * time rather than iterations of whatever loop happens to be calling us. */
   .watchdog_cnt = 150,
   .use_hw_watchdog = 1,
   .set_defaults_hook = NULL,
   .pre_state_change_hook = NULL,
   .post_state_change_hook = cb_state_change,
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

static const char * al_name (uint8_t st)
{
   switch (st & 0x0F)
   {
      case ESCinit:   return "INIT";
      case ESCpreop:  return "PREOP";
      case ESCboot:   return "BOOT";
      case ESCsafeop: return "SAFEOP";
      case ESCop:     return "OP";
      default:        return "?";
   }
}

/* Report every AL state transition and any error the stack raises.
 *
 * Without these the slave is silent about the one thing that matters when a
 * master cannot bring it up: which transition was refused and why. The AL
 * status code is the actual diagnosis -- SMRESULT_ERRSM2/ERRSM3 point at a
 * SyncManager mismatch, anything else at the stack above it.
 */
static void cb_state_change (uint8_t * as, uint8_t * an)
{
   printf ("AL: %s -> %s%s  ALstatus 0x%04X  ALerror 0x%04X\n",
           al_name ((uint8_t)(ESCvar.ALstatus & 0x0F)),
           al_name (*an),
           (*an & ESCerror) ? " (ERROR)" : "",
           ESCvar.ALstatus, ESCvar.ALerror);
   (void)as;
}

/* Process data callbacks.
 *
 * The TxPDO mirrors the RxPDO, so whatever a master writes to the LED objects
 * comes straight back on the Button objects. That makes one observation in the
 * master prove both directions of the process image at once, and it exercises
 * exactly the two callbacks renamed in Phase 2.
 */
void cb_apply_rxpdo (void)
{
   rx_mirror = (uint8_t)((Obj.LEDs.LED0 ? 0x01 : 0) | (Obj.LEDs.LED1 ? 0x02 : 0) |
                         (Obj.LEDs.LED2 ? 0x04 : 0) | (Obj.LEDs.LED3 ? 0x08 : 0) |
                         (Obj.LEDs.LED4 ? 0x10 : 0) | (Obj.LEDs.LED5 ? 0x20 : 0));
   rx_calls++;
}

void cb_update_txpdo (void)
{
   Obj.Buttons.Button0 = Obj.LEDs.LED0;
   Obj.Buttons.Button1 = Obj.LEDs.LED1;
   Obj.Buttons.Button2 = Obj.LEDs.LED2;
   Obj.Buttons.Button3 = Obj.LEDs.LED3;
   Obj.Buttons.Button4 = Obj.LEDs.LED4;
   Obj.Buttons.Button5 = Obj.LEDs.LED5;
   tx_calls++;
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
      /* "noedge" releases the IRQ and SYNC0 lines so an external tool can
       * watch them while this process drives the stack. A line can only be
       * requested once, so the two cannot observe it simultaneously. */
      if (argc > 4 && strcmp (argv[4], "noedge") == 0)
      {
         hw_cfg.irq_line = -1;
         hw_cfg.sync0_line = -1;
      }
      /* Optional: attempt to set the Sync/Latch PDI configuration (0x0151)
       * at run time. Normally this is loaded from the SII EEPROM and is not
       * writable, but if the PDI can write it the correct value can be found
       * on a scope without flashing anything. Readback tells us which. */
      if (argc > 4 && strcmp (argv[4], "syncthread") == 0)
      {
         /* The stack keeps the cyclic loop running so the slave stays in OP
          * and the master keeps generating SYNC0; a second thread owns the
          * SYNC0 line and measures wake latency. The IRQ line is released. */
         hw_cfg.irq_line = -1;
         sync_thread_prio = (argc > 5) ? (int)strtoul (argv[5], NULL, 0) : 0;
      }
      else if (argc > 5)
      {
         sync_pin_cfg = (int)strtoul (argv[5], NULL, 0);
      }
      if (ecat_slv_init (&config) != 0)
      {
         printf ("FAIL: stack init failed\n");
         return 1;
      }
      printf ("stack init OK, entering cyclic loop\n");
      {
         /* Time each ecat_slv() call. This is the whole cyclic cost --
          * AL event read, RxPDO fetch, callbacks, TxPDO write -- and is the
          * figure the 1 ms SYNC0 budget in the roadmap depends on. Reported
          * as a distribution because the tail is what decides feasibility.
          */
         enum { NS = 20000 };
         static uint64_t samples[NS];
         static uint64_t spi_samples[NS];
         static uint64_t xfer_samples[NS];
         static uint64_t iv[NS];            /* SYNC0 inter-edge intervals */
         uint32_t n = 0, nv = 0;
         uint64_t n_sync = 0, n_irq = 0, t_prev = 0, t_edge;
         uint64_t lat_sum = 0, lat_max = 0, lat_n = 0;
         int irq_fd, sync_fd, coalesced;
         struct timespec a, b, last;

         /* Watch the DC and interrupt lines from inside the cyclic loop. A
          * separate probe process cannot do this: the master only configures
          * DC once the slave is in OP, which requires this stack to be the one
          * holding the SPI bus.
          */
         irq_fd  = ESC_hw_edge_open (hw_cfg.gpiochip, hw_cfg.irq_line);
         sync_fd = ESC_hw_edge_open (hw_cfg.gpiochip, hw_cfg.sync0_line);
         if (sync_pin_cfg >= 0)
         {
            uint8_t w = (uint8_t)sync_pin_cfg, rb = 0;
            ESC_write (0x0151, &w, sizeof (w));
            ESC_read (0x0151, &rb, sizeof (rb));
            printf ("0x0151: wrote %02X, reads back %02X -> %s\n", w, rb,
                    (rb == w) ? "PDI-WRITABLE" : "not writable from the PDI");
         }
         printf ("edge lines: IRQ %d %s, SYNC0 %d %s\n",
                 hw_cfg.irq_line, irq_fd >= 0 ? "ok" : "FAILED",
                 hw_cfg.sync0_line, sync_fd >= 0 ? "ok" : "FAILED");

         if (sync_thread_prio >= 0 && sync_fd >= 0)
         {
            static int tfd;
            tfd = sync_fd;
            if (pthread_create (&sync_tid, NULL, sync_latency_thread, &tfd) == 0)
            {
               sync_thread_running = 1;
               printf ("sync thread: measuring SYNC0 wake latency\n");
            }
            else
            {
               printf ("sync thread: pthread_create failed\n");
            }
         }

         clock_gettime (CLOCK_MONOTONIC, &last);
         for (;;)
         {
            uint64_t s0 = ESC_hw_spi_ns (), c0 = ESC_hw_spi_count ();
            clock_gettime (CLOCK_MONOTONIC, &a);
            ecat_slv ();
            clock_gettime (CLOCK_MONOTONIC, &b);
            if (n < NS)
            {
               spi_samples[n]  = ESC_hw_spi_ns () - s0;
               xfer_samples[n] = ESC_hw_spi_count () - c0;
               samples[n++] = (uint64_t)(b.tv_sec - a.tv_sec) * 1000000000ull +
                              (uint64_t)(b.tv_nsec - a.tv_nsec);
            }

            /* Watch the DC activation register every cycle and report any
             * change. A five second sample cannot distinguish "the master
             * never wrote it" from "the master wrote it and something cleared
             * it", and those have different causes. */
            {
               static uint8_t dc_prev = 0xFF;
               static int dc_first = 1;
               uint8_t dc_now = 0;
               ESC_read (ESCREG_SYNC_ACT, &dc_now, sizeof (dc_now));
               if (dc_first || dc_now != dc_prev)
               {
                  uint32_t c = 0;
                  ESC_read (ESCREG_SYNC0_CYCLE_TIME, &c, sizeof (c));
                  printf ("0x0981 %02X -> %02X  (cycle %u ns, AL %04X)\n",
                          dc_first ? 0 : dc_prev, dc_now, (unsigned)c,
                          ESCvar.ALstatus);
                  dc_prev = dc_now;
                  dc_first = 0;
               }
            }

            /* Drain both lines without blocking. Intervals come from the
             * kernel's event timestamps, so they measure the signal itself
             * rather than when this loop got around to looking.
             */
            while (!sync_thread_running &&
                   ESC_hw_edge_wait (sync_fd, 0, &t_edge, &coalesced) == 1)
            {
               uint64_t nowns = (uint64_t)b.tv_sec * 1000000000ull +
                                (uint64_t)b.tv_nsec;
               n_sync += 1u + (uint64_t)coalesced;
               if (t_prev != 0 && nv < NS)
               {
                  iv[nv++] = t_edge - t_prev;
               }
               t_prev = t_edge;
               /* Upper bound on wake latency: this is a polling loop, so it
                * includes however long the loop took to come back round. A
                * true figure needs the blocking design of Phase 3. */
               if (nowns > t_edge)
               {
                  uint64_t l = nowns - t_edge;
                  lat_sum += l; lat_n++;
                  if (l > lat_max) lat_max = l;
               }
            }
            while (ESC_hw_edge_wait (irq_fd, 0, NULL, NULL) == 1)
            {
               n_irq++;
            }

            if (b.tv_sec - last.tv_sec >= 5)
            {
               uint64_t sum = 0;
               uint32_t i;
               if (n > 0)
               {
                  uint64_t spisum = 0, xfersum = 0;
                  uint64_t * srt = malloc (n * sizeof (uint64_t));
                  if (srt != NULL)
                  {
                     memcpy (srt, samples, n * sizeof (uint64_t));
                     for (i = 0; i < n; i++)
                     {
                        sum += srt[i];
                        spisum += spi_samples[i];
                        xfersum += xfer_samples[i];
                     }
                     qsort (srt, n, sizeof (uint64_t), cmp_u64);
                     printf ("ecat_slv n=%u  min %.1f  median %.1f  mean %.1f  "
                             "p99 %.1f  max %.1f us | rx=%llu tx=%llu leds=0x%02X\n",
                             n, (double)srt[0] / 1000.0,
                             (double)srt[n / 2] / 1000.0,
                             (double)sum / (double)n / 1000.0,
                             (double)srt[(n * 99) / 100] / 1000.0,
                             (double)srt[n - 1] / 1000.0,
                             (unsigned long long)rx_calls,
                             (unsigned long long)tx_calls,
                             rx_mirror);
                     printf ("   SPI: mean %.1f us over %.1f transfers per cycle"
                             " -> %.0f%% of cycle time is NOT SPI\n",
                             (double)spisum / (double)n / 1000.0,
                             (double)xfersum / (double)n,
                             100.0 * (1.0 - (double)spisum / (double)sum));
                     {
                        /* What the master actually wrote to the DC unit.
                         * 0x0981 bit0 = cyclic unit enabled, bit1 = SYNC0
                         * generation. If these read zero the master never
                         * activated it, and no amount of probing the pin will
                         * show a signal. */
                        uint8_t  r_unit = 0, r_act = 0;
                        uint32_t r_cyc = 0;
                        uint64_t r_start = 0, r_time = 0;
                        ESC_read (0x0980, &r_unit, sizeof (r_unit));
                        ESC_read (ESCREG_SYNC_ACT, &r_act, sizeof (r_act));
                        ESC_read (ESCREG_SYNC0_CYCLE_TIME, &r_cyc, sizeof (r_cyc));
                        ESC_read (0x0990, &r_start, sizeof (r_start));
                        ESC_read (ESCREG_LOCALTIME, &r_time, sizeof (r_time));
                        printf ("   DC: 0x0980=%02X 0x0981=%02X (%s) "
                                "cycle=%u ns start=%llu now=%llu\n",
                                r_unit, r_act,
                                (r_act & 0x02) ? "SYNC0 ENABLED" : "sync0 off",
                                (unsigned)r_cyc,
                                (unsigned long long)r_start,
                                (unsigned long long)r_time);
                        {
                           /* Whether the SYNC0 *pin* is enabled as an output is
                            * separate from whether the sync unit runs, and comes
                            * from the SII EEPROM rather than from the master.
                            * 0x0151 is the Sync/Latch PDI configuration, 0x0982
                            * the pulse length (0 means level-until-acknowledged
                            * rather than a pulse), 0x098E the SYNC0 status. */
                           uint8_t  pdi = 0, sl = 0, st0 = 0;
                           uint16_t plen = 0;
                           ESC_read (0x0140, &pdi, sizeof (pdi));
                           ESC_read (0x0151, &sl, sizeof (sl));
                           ESC_read (0x0982, &plen, sizeof (plen));
                           ESC_read (0x098E, &st0, sizeof (st0));
                           printf ("   PIN: 0x0140 PDIctl=%02X  0x0151 SyncLatchCfg=%02X"
                                   "  0x0982 pulselen=%u  0x098E sync0stat=%02X\n",
                                   pdi, sl, (unsigned)plen, st0);
                        }
                        {
                           /* Who is holding the state machine back. AL control
                            * is written by the master and AL status by us, so
                            * control==1 means the master is not asking for a
                            * transition, while control>status means we refused
                            * one and the code says why. DL status distinguishes
                            * that from having no link to be asked over: bit 0
                            * is PDI operational, bits 4/5 the two ports. */
                           uint16_t alctl = 0, alsts = 0, alcode = 0, dls = 0;
                           uint16_t wd = 0, wdp = 0, wdd = 0;
                           ESC_read (ESCREG_ALCONTROL, &alctl, sizeof (alctl));
                           ESC_read (ESCREG_ALSTATUS, &alsts, sizeof (alsts));
                           ESC_read (ESCREG_ALERROR, &alcode, sizeof (alcode));
                           ESC_read (ESCREG_DLSTATUS, &dls, sizeof (dls));
                           ESC_read (0x0400, &wdd, sizeof (wdd));
                           ESC_read (0x0420, &wdp, sizeof (wdp));
                           ESC_read (0x0440, &wd, sizeof (wd));
                           printf ("   AL: ctl=%04X sts=%04X err=%04X "
                                   "0x0110 DL=%04X (link %s) | WD div=%u "
                                   "pdt=%u 0x0440=%04X\n",
                                   alctl, alsts, alcode, dls,
                                   (dls & 0x0030) ? "up" : "DOWN",
                                   (unsigned)wdd, (unsigned)wdp, wd);
                        }
                     }
                     if (sync_thread_running && wl_n > 16)
                     {
                        uint32_t m = (uint32_t)wl_n, k;
                        uint64_t *w = malloc (m * sizeof (uint64_t));
                        if (w != NULL)
                        {
                           uint64_t wsum = 0;
                           for (k = 0; k < m; k++) { w[k] = wl_ns[k]; wsum += w[k]; }
                           qsort (w, m, sizeof (uint64_t), cmp_u64);
                           printf ("   WAKE n=%u  min %.1f  median %.1f  mean %.1f  "
                                   "p99 %.1f  p99.9 %.1f  max %.1f us  (timeouts %llu)\n",
                                   m, (double)w[0]/1000.0, (double)w[m/2]/1000.0,
                                   (double)wsum/(double)m/1000.0,
                                   (double)w[(m*99)/100]/1000.0,
                                   (double)w[(uint32_t)((uint64_t)m*999/1000)]/1000.0,
                                   (double)w[m-1]/1000.0,
                                   (unsigned long long)wl_timeouts);
                           free (w);
                        }
                        wl_n = 0;
                     }
                     if (n_sync > 0)
                     {
                        printf ("   SYNC0 %llu edges, IRQ %llu",
                                (unsigned long long)n_sync,
                                (unsigned long long)n_irq);
                        if (nv > 8)
                        {
                           qsort (iv, nv, sizeof (uint64_t), cmp_u64);
                           printf (" | interval min %.1f median %.1f p99 %.1f max %.1f us",
                                   (double)iv[0] / 1000.0,
                                   (double)iv[nv / 2] / 1000.0,
                                   (double)iv[(nv * 99) / 100] / 1000.0,
                                   (double)iv[nv - 1] / 1000.0);
                        }
                        if (lat_n > 0)
                        {
                           printf (" | observe-delay mean %.1f max %.1f us (polling, upper bound)",
                                   (double)lat_sum / (double)lat_n / 1000.0,
                                   (double)lat_max / 1000.0);
                        }
                        printf ("\n");
                     }
                     else
                     {
                        printf ("   SYNC0 no edges (enable distributed clocks in the master)\n");
                     }
                     n_sync = 0; n_irq = 0; nv = 0;
                     lat_sum = 0; lat_max = 0; wl_n = 0;
                     free (srt);
                  }
               }
               n = 0;
               last = b;
            }
         }
      }
   }

   usage (argv[0]);
   return 2;
}
