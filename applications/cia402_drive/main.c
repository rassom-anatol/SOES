/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/** \file
 * \brief
 * CiA402 dictionary bring-up against a real master.
 *
 * This is deliberately not a drive. There is no CiA402 state machine here and
 * there is not meant to be one: it lives in the consuming motion controller,
 * where it can be unit tested against the profile without any hardware at all
 * (docs/cia402-roadmap.md, Phase 5). Putting a second copy here would mean two
 * implementations of the one thing in this project most worth having exactly
 * one of.
 *
 * What this application is for is everything *below* that state machine, which
 * cannot be tested without a master on the wire:
 *
 *   - the generated SyncManager layout survives PREOP->SAFEOP, i.e. the ESI the
 *     master configured from and the constants compiled in here agree, which
 *     ESC_checkSM23 enforces and reports only as an AL status code;
 *   - the PDO mapping words describe the process image the master actually
 *     transfers, in the right order and with the right padding;
 *   - the ESI data types match the object list, which is the fix in
 *     docs/stack-review.md section 2.1 and the one defect there that failed
 *     silently. Mirroring each setpoint into its matching actual value is what
 *     makes it visible: write a negative TargetPosition and a correct ESI
 *     brings back a negative PositionActual, while the unsigned declaration
 *     that shipped before brings back something near 4.29e9.
 *
 * The mirror is the whole application. Every fault and status word reads zero,
 * because there is no gate driver and no motor attached to report on, and
 * inventing values for them would make the one thing this is for -- believing
 * the numbers -- impossible.
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

/* Transport configuration. Same wiring as the diagnostic application: the
 * LAN9252 is on SPI1 with its IRQ, SYNC0 and reset on plain GPIO. See
 * docs/cia402-roadmap.md Phase 0 for why these pins and not others.
 */
static esc_hw_cfg_t hw_cfg =
{
   .spidev         = "/dev/spidev1.0",
   .spi_speed_hz   = 25000000,  /* measured knee; see roadmap 3.4 */
   .spi_mode       = 0,
   .gpiochip       = "/dev/gpiochip0",
   .irq_line       = 17,
   .sync0_line     = 18,
   .reset_line     = 25,     /* shared with the TMC4671 reset / cmc CTRL_RST */
   .reset_pulse_us = 500,
   .op_timeout_ms  = 100,
};

static volatile uint64_t rx_calls = 0;
static volatile uint64_t tx_calls = 0;

/* Mailbox responses this slave has sent.
 *
 * Counted from transitions of ESCvar.mbxbackup, which ESC_writembx sets, rather
 * than from the SM0 and SM1 bits of the AL event register. Counting the event
 * bits does not work and is worth recording as a trap: ESC_mbxprocess reads the
 * mailbox and thereby clears the SM0 event during the same ecat_slv() call, so
 * by the time an application samples ESCvar.ALevent the evidence is gone and
 * busy mailbox traffic reads as a flat zero. That reading once made a working
 * mailbox look completely silent.
 *
 * For what actually arrives, build with ESC_DEBUG: ESC_coeprocess logs every
 * CoE service before interpreting it, which is on the handling path and so
 * cannot be defeated this way.
 */
static uint64_t mbx_responses = 0;

/* SYNC0 period the master configured, from dc_checker. The cyclic loop bounds
 * its wait at twice this, which is what identifies a stopped sync unit. */
static uint32_t sync0_period_ns = 0;

/* Measurement controls. `force_freerun` keeps the loop on the free-running
 * path even when the master has activated DC, so the two loop structures can
 * be compared in one session against one master rather than across runs --
 * comparing measurements taken minutes apart under different conditions is how
 * the per-transfer cost came to be misattributed in the first place.
 * `limit_override` raises the sync error counter limit so that the slave stays
 * in OP long enough to be measured instead of dropping to SAFEOP mid-sample.
 */
static int force_freerun = 0;
static uint32_t limit_override = 0;

/* Scheduler accounting for this thread, straight from the kernel.
 *   field 1: nanoseconds spent running on a CPU
 *   field 2: nanoseconds spent on the run queue, runnable but not running
 *   field 3: number of times it was given the CPU
 * Field 2 is the one that says whether the scheduler is stealing time, which
 * is otherwise indistinguishable from the work simply being slow.
 */
static void read_schedstat (uint64_t * run, uint64_t * wait, uint64_t * slices)
{
   FILE * fh = fopen ("/proc/self/schedstat", "r");

   *run = *wait = *slices = 0;
   if (fh != NULL)
   {
      if (fscanf (fh, "%" SCNu64 " %" SCNu64 " %" SCNu64, run, wait, slices) != 3)
      {
         *run = *wait = *slices = 0;
      }
      fclose (fh);
   }
}

/* The cyclic loop blocks on the SYNC0 pin, so it owns that line and the wake
 * count is the edge count -- see ESC_hw_wait_sync0 for why the pin rather than
 * the AL event register. */

static void cb_state_change (uint8_t * as, uint8_t * an);

/** esc_cfg_t.esc_check_dc_handler: vet the DC configuration the master wrote.
 *
 * ESC_checkDC delegates the entire decision here once the sync unit is active,
 * and returns ALERR_DCINVALIDSYNCCFG if no handler is registered -- so without
 * this function DC cannot be entered at all.
 *
 * Validating rather than merely accepting is the point. The AL status code set
 * distinguishes an unsupported sync configuration from a cycle time the device
 * cannot sustain, and answering with the specific one turns a master
 * misconfiguration from "the drive refuses and will not say why" into a
 * diagnosis. The floor comes from CMC_MIN_CYCLE_NS, which the generator
 * exports from the same object, 0x1C32:05, that advertises it -- so what is
 * enforced here and what a master reads are one number.
 */
static uint16_t dc_checker (void)
{
   uint32_t sync0_cycle = 0;
   uint8_t activation = 0;

   ESC_read (ESCREG_SYNC_ACT, &activation, sizeof (activation));
   ESC_read (ESCREG_SYNC0_CYCLE_TIME, &sync0_cycle, sizeof (sync0_cycle));
   sync0_cycle = etohl (sync0_cycle);

   if ((activation & ESCREG_SYNC_ACT_ACTIVATED) == 0)
   {
      /* The cyclic unit is on but SYNC0 generation is not, so nothing will
       * ever wake the loop. */
      return ALERR_DCINVALIDSYNCCFG;
   }

   if (sync0_cycle == 0)
   {
      return ALERR_DCSYNC0CYCLETIME;
   }

   if (sync0_cycle < CMC_MIN_CYCLE_NS)
   {
      printf ("DC: master asked for %u ns, floor is %u ns\n",
              (unsigned)sync0_cycle, (unsigned)CMC_MIN_CYCLE_NS);
      return ALERR_DCSYNC0CYCLETIME;
   }

   ESCvar.dcsync = 1;
   ESCvar.synccounterlimit = (limit_override != 0)
                             ? (uint16_t)limit_override
                             : Obj.ErrorSettings.SyncErrorCounterLimit;

   /* Report back what was accepted, so 0x1C32/0x1C33 describe the running
    * configuration rather than whatever was last written to them. */
   Obj.SM2Sync.CycleTime = sync0_cycle;
   Obj.SM3Sync.CycleTime = sync0_cycle;
   sync0_period_ns = sync0_cycle;

   printf ("DC: SYNC0 at %u ns, sync error limit %u\n",
           (unsigned)sync0_cycle, (unsigned)ESCvar.synccounterlimit);
   return 0;
}

/** Master outputs have arrived: SM2 has been read and unpacked into Obj.
 *
 * Runs before the TxPDO is packed in the same DIG_process call, so a value
 * written here reaches the master in the same cycle.
 */
void cb_apply_rxpdo (void)
{
   unsigned n;

   /* Every loop over axes is bounded by CMC_AXIS_COUNT, which the generator
    * emits into utypes.h. Nothing here names an axis by number, which is what
    * lets the same source build against the one, two and four axis variants --
    * the rule this application exists partly to demonstrate, since the
    * consuming motion controller has to follow it too. */
   for (n = 0; n < CMC_AXIS_COUNT; n++)
   {
      _Axis * a = &Obj.axis[n];

      a->PositionActual          = a->TargetPosition;
      a->VelocityActual          = a->TargetVelocity;
      a->TorqueActual            = a->TargetTorque;
      a->ModesOfOperationDisplay = a->ModesOfOperation;

      /* Zero by construction while the mirror is exact. It is mapped rather
       * than omitted so that the master's interpretation of a signed 32-bit
       * entry that legitimately goes negative is exercised by the position
       * mirror above. */
      a->FollowingErrorActual = a->TargetPosition - a->PositionActual;
   }

   rx_calls++;
}

/** Slave inputs are about to be packed into SM3 for the master to read. */
void cb_update_txpdo (void)
{
   unsigned n;

   for (n = 0; n < CMC_AXIS_COUNT; n++)
   {
      _Axis * a = &Obj.axis[n];

      /* Switch on disabled: the honest CiA402 state for a device with no power
       * stage. It is a constant here rather than a computed value precisely
       * because the state machine that should compute it is not in this
       * repository -- a plausible-looking statusword from a stub would be
       * worse than an obviously static one. */
      a->Statusword = 0x0040;

      /* No gate driver and no motor to report on. */
      a->ErrorCode        = 0;
      a->GateDriverFaults = 0;
      a->DriveStatusFlags = 0;
   }

   /* Mirror the stack's sync accounting into the dictionary. The stack owns
    * the counts because it observes the events; these objects are how a master
    * sees them, and they are only worth carrying if they are current. */
   Obj.SM2Sync.SMEventMissed = ESCvar.smeventmissed;
   Obj.SM3Sync.SMEventMissed = ESCvar.smeventmissed;
   Obj.SM2Sync.SyncError     = ESCvar.syncerror;
   Obj.SM3Sync.SyncError     = ESCvar.syncerror;

   tx_calls++;
}

/** Called from ESC_stopoutputs when the stack leaves an output state, whether
 * because the master asked or because the watchdog expired. With no drive
 * attached the safe state is simply to forget the setpoints, so that a stale
 * target cannot be mirrored back and read as if it were still commanded.
 */
static void app_safe_state (void)
{
   unsigned n;

   for (n = 0; n < CMC_AXIS_COUNT; n++)
   {
      _Axis * a = &Obj.axis[n];

      a->TargetPosition = 0;
      a->TargetVelocity = 0;
      a->TargetTorque   = 0;
      a->PositionActual = 0;
      a->VelocityActual = 0;
      a->TorqueActual   = 0;
   }

   printf ("safe state: setpoints cleared on %u axis/axes\n",
           (unsigned)CMC_AXIS_COUNT);
}

static esc_cfg_t config =
{
   .user_arg = &hw_cfg,
   /* Not an optimisation: esc.c returns early when this is 0, before
    * ESC_checkDC runs and before the SYNC0 bit is ever added to the AL event
    * mask, so it is the precondition for DC existing at all. */
   .use_interrupt = 1,
   /* Fallback only, for a master that leaves the hardware watchdog disabled.
    * See the watchdog handling in soes/ecat_slv.c. */
   .watchdog_cnt = 150,
   .use_hw_watchdog = 1,
   .set_defaults_hook = NULL,
   .pre_state_change_hook = NULL,
   .post_state_change_hook = cb_state_change,
   .application_hook = NULL,
   .safe_state_override = app_safe_state,
   .pre_object_download_hook = NULL,
   .post_object_download_hook = NULL,
   .rxpdo_override = NULL,
   .txpdo_override = NULL,
   /* Without these the AL event mask never gains the SM2, SM3 and SYNC0 bits
    * and the LAN9252's interrupt output is never switched on, so its IRQ pin
    * stays idle and every wait in the cyclic loop times out. The symptom is
    * the DC liveness check firing on a bus that is in fact perfectly healthy. */
   .esc_hw_interrupt_enable = ESC_interrupt_enable,
   .esc_hw_interrupt_disable = ESC_interrupt_disable,
   .esc_hw_eep_handler = NULL,
   .esc_check_dc_handler = dc_checker,
   /* The SYNC0 pin, not the IRQ pin: this ESC pulses SYNC0 correctly but never
    * sets its AL event bit, so a loop waiting on the IRQ pin never sees a
    * cycle boundary. See ESC_hw_wait_sync0. */
   .esc_hw_wait = ESC_hw_wait_sync0,
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

static void cb_state_change (uint8_t * as, uint8_t * an)
{
   printf ("AL: %s -> %s%s  ALerror 0x%04X\n",
           al_name (*as), al_name (*an),
           (*an & ESCerror) ? " (ERROR)" : "",
           ESCvar.ALerror);
   fflush (stdout);
}

static int cmp_u64 (const void * a, const void * b)
{
   uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
   return (x > y) - (x < y);
}

int main (int argc, char * argv[])
{
   enum { NS = 20000 };
   static uint64_t samples[NS];
   static uint64_t xfers[NS];
   static uint64_t spi_work[NS];
   static uint64_t per_xfer[NS];
   uint64_t sched_run = 0, sched_wait = 0, sched_slices = 0;
   uint64_t run0 = 0, wait0 = 0, slices0 = 0;
   /* Cycles actually executed in this reporting interval. Distinct from n,
    * which stops at the size of the sample arrays: dividing a five-second
    * scheduler delta by a capped sample count overstates the per-cycle cost by
    * whatever factor the cap discarded. */
   uint64_t cycles = 0;
   uint32_t n = 0;
   struct timespec a, b, last;

   if (argc > 1)
   {
      hw_cfg.spidev = argv[1];
   }
   if (argc > 2)
   {
      hw_cfg.spi_speed_hz = (uint32_t)strtoul (argv[2], NULL, 0);
   }
   if (argc > 3 && strcmp (argv[3], "freerun") == 0)
   {
      force_freerun = 1;
   }
   if (argc > 4)
   {
      limit_override = (uint32_t)strtoul (argv[4], NULL, 0);
   }

   printf ("cia402_drive: %u axis/axes on %s at %u Hz\n",
           (unsigned)CMC_AXIS_COUNT, hw_cfg.spidev,
           (unsigned)hw_cfg.spi_speed_hz);

   if (ecat_slv_init (&config) != 0)
   {
      printf ("FAIL: stack init failed\n");
      return 1;
   }
   printf ("stack init OK, entering cyclic loop\n");

   read_schedstat (&run0, &wait0, &slices0);
   clock_gettime (CLOCK_MONOTONIC, &last);
   for (;;)
   {
      uint64_t c0 = ESC_hw_spi_count ();
      uint64_t s0 = ESC_hw_spi_ns ();

      clock_gettime (CLOCK_MONOTONIC, &a);
      if ((ESCvar.dcsync > 0) && (sync0_period_ns > 0) && !force_freerun)
      {
         /* Bounded at twice the period the master configured: expiring with DC
          * active is what identifies a stopped sync unit. */
         ecat_slv_run_dc ((uint64_t)sync0_period_ns * 2ull);
      }
      else
      {
         ecat_slv ();
      }
      clock_gettime (CLOCK_MONOTONIC, &b);

      {
         static uint8_t backup_prev = 0;

         if (ESCvar.mbxbackup != backup_prev)
         {
            if (ESCvar.mbxbackup != 0)
            {
               mbx_responses++;
            }
            backup_prev = ESCvar.mbxbackup;
         }
      }

      cycles++;
      if (n < NS)
      {
         xfers[n] = ESC_hw_spi_count () - c0;
         spi_work[n] = ESC_hw_spi_ns () - s0;
         /* Cost of one transfer, derived per cycle so that it can be reported
          * as a distribution rather than as one ratio of two averages. */
         per_xfer[n] = (xfers[n] > 0) ? (spi_work[n] / xfers[n]) : 0;
         samples[n++] = (uint64_t)(b.tv_sec - a.tv_sec) * 1000000000ull +
                        (uint64_t)(b.tv_nsec - a.tv_nsec);
      }

      if (b.tv_sec - last.tv_sec >= 5)
      {
         uint16_t alctl = 0, alsts = 0, alerr = 0;
         uint8_t smc2 = 0;
         uint16_t wd = 0;

         ESC_read (ESCREG_ALCONTROL, &alctl, sizeof (alctl));
         ESC_read (ESCREG_ALSTATUS, &alsts, sizeof (alsts));
         ESC_read (ESCREG_ALERROR, &alerr, sizeof (alerr));
         ESC_read (ESCREG_SM2 + 4, &smc2, sizeof (smc2));
         ESC_read (ESCREG_WDSTATUS, &wd, sizeof (wd));

         if (n > 0)
         {
            uint64_t sum = 0, xsum = 0, ssum = 0;
            uint32_t i;
            uint64_t * srt = malloc (n * sizeof (uint64_t));

            if (srt != NULL)
            {
               memcpy (srt, samples, n * sizeof (uint64_t));
               for (i = 0; i < n; i++)
               {
                  sum += srt[i];
                  xsum += xfers[i];
                  ssum += spi_work[i];
               }
               qsort (srt, n, sizeof (uint64_t), cmp_u64);
               /* Medians, not means: the wall distribution has a tail running
                * to ten milliseconds, and a mean over that says nothing about
                * what a typical cycle costs. */
               {
                  uint64_t * sx = malloc (n * sizeof (uint64_t));
                  uint64_t * st = malloc (n * sizeof (uint64_t));
                  uint64_t * sp = malloc (n * sizeof (uint64_t));

                  if (sx != NULL && st != NULL && sp != NULL)
                  {
                     memcpy (sx, spi_work, n * sizeof (uint64_t));
                     memcpy (st, xfers, n * sizeof (uint64_t));
                     memcpy (sp, per_xfer, n * sizeof (uint64_t));
                     qsort (sx, n, sizeof (uint64_t), cmp_u64);
                     qsort (st, n, sizeof (uint64_t), cmp_u64);
                     qsort (sp, n, sizeof (uint64_t), cmp_u64);
                     printf ("   SPI: per cycle median %.1f  p99 %.1f us"
                             " | transfers median %llu"
                             " | per transfer median %.1f  p99 %.1f us\n",
                             (double)sx[n / 2] / 1000.0,
                             (double)sx[(n * 99) / 100] / 1000.0,
                             (unsigned long long)st[n / 2],
                             (double)sp[n / 2] / 1000.0,
                             (double)sp[(n * 99) / 100] / 1000.0);
                  }
                  free (sx); free (st); free (sp);
               }
               {
                  uint64_t r, w, sl;

                  read_schedstat (&r, &w, &sl);
                  sched_run = r - run0;
                  sched_wait = w - wait0;
                  sched_slices = sl - slices0;
                  run0 = r; wait0 = w; slices0 = sl;
                  printf ("   SCHED: cycles=%llu (sampled %u)  onCPU %.1f"
                          " us/cycle  runqueue-wait %.2f us/cycle"
                          "  slices=%llu%s\n",
                          (unsigned long long)cycles, n,
                          (double)sched_run / (double)cycles / 1000.0,
                          (double)sched_wait / (double)cycles / 1000.0,
                          (unsigned long long)sched_slices,
                          force_freerun ? "  [FREERUN forced]" : "");
               }
               /* Under DC the wall figure is the SYNC0 period, because the
                * call blocks waiting for it; the SPI figure is the work. Both
                * are printed so neither is mistaken for the other. */
               printf ("cycle n=%u  wall median %.1f  p99 %.1f  max %.1f us"
                       " | spi mean %.1f us over %.1f transfers%s\n",
                       n, (double)srt[n / 2] / 1000.0,
                       (double)srt[(n * 99) / 100] / 1000.0,
                       (double)srt[n - 1] / 1000.0,
                       (double)ssum / (double)n / 1000.0,
                       (double)xsum / (double)n,
                       (ESCvar.dcsync > 0) ? "  [DC]" : "  [free-run]");
               free (srt);
            }
         }

         {
            uint32_t w = 0, sm = 0, s0 = 0, mb = 0, id = 0;

            ecat_slv_dc_counters (&w, &sm, &s0, &mb, &id);
            printf ("   EVT: wakes=%u  sm=%u  sync0=%u  mbx=%u  idle=%u\n",
                    w, sm, s0, mb, id);
         }
         {
            /* 0x0204 AL Event Mask selects which events may drive the IRQ pin.
             * 0x0151 is the Sync/Latch PDI configuration, loaded from SII
             * EEPROM word 1 and not writable from the PDI: as well as making
             * the pin an output it controls whether a SYNC0 pulse is mapped
             * into the AL Event Request register at all. A pulse that is
             * visible on a scope but absent from 0x0220 points here. */
            uint32_t almask = 0;
            uint8_t slcfg = 0, syncact = 0, sync0stat = 0;

            ESC_read (ESCREG_ALEVENTMASK, &almask, sizeof (almask));
            ESC_read (0x0151, &slcfg, sizeof (slcfg));
            ESC_read (ESCREG_SYNC_ACT, &syncact, sizeof (syncact));
            ESC_read (ESCREG_SYNC0_STATUS, &sync0stat, sizeof (sync0stat));
            printf ("   SYNC: 0x0204 mask=%08X (sync0 %s)"
                    "  0x0151=%02X  0x0981=%02X  0x098E=%02X\n",
                    (unsigned)etohl (almask),
                    (etohl (almask) & ESCREG_ALEVENT_DC_SYNC0) ? "unmasked"
                                                              : "MASKED",
                    slcfg, syncact, sync0stat);
         }
         printf ("   DC: dcsync=%u sync0=%u ns  synccounter=%d limit=%u"
                 "  missed=%u syncerror=%u\n",
                 ESCvar.dcsync, (unsigned)sync0_period_ns,
                 (int)ESCvar.synccounter,
                 (unsigned)ESCvar.synccounterlimit,
                 (unsigned)ESCvar.smeventmissed, ESCvar.syncerror);

         printf ("   AL: ctl=%04X sts=%04X err=%04X | SM2 ctl=%02X "
                 "(wd trigger %s) 0x0440=%04X | rx=%llu tx=%llu\n",
                 alctl, alsts, alerr, smc2, (smc2 & 0x40) ? "ON" : "off", wd,
                 (unsigned long long)rx_calls, (unsigned long long)tx_calls);

         {
            /* Whether the ESC came up at all, as distinct from whether a master
             * is talking to it. 0x0110 bits 4/5 are the two ports' links and
             * bit 0 is PDI operational; 0x0502 reports EEPROM loading, whose
             * error bits are the first thing to check after writing an SII,
             * because an image the ESC will not load leaves the device on the
             * wire but not configurable. */
            uint16_t dls = 0, eep = 0;
            uint8_t pdi = 0;

            ESC_read (ESCREG_DLSTATUS, &dls, sizeof (dls));
            ESC_read (ESCREG_EECONTSTAT, &eep, sizeof (eep));
            ESC_read (0x0140, &pdi, sizeof (pdi));
            eep = etohs (eep);
            printf ("   ESC: 0x0110 DL=%04X (link %s, PDI %s)"
                    "  0x0140 PDIctl=%02X  0x0502 EEP=%04X%s%s%s%s\n",
                    etohs (dls), (etohs (dls) & 0x0030) ? "up" : "DOWN",
                    (etohs (dls) & 0x0001) ? "operational" : "NOT OPERATIONAL",
                    pdi, eep,
                    (eep & 0x0800) ? " CHECKSUM-ERROR" : "",
                    (eep & 0x1000) ? " DEVICE-INFO-ERROR" : "",
                    (eep & 0x2000) ? " CMD-ERROR" : "",
                    (eep & 0x4000) ? " WRITE-ERROR" : "");
         }

         {
            uint8_t c0m = 0, a0m = 0, c1m = 0, a1m = 0;
            uint16_t l0 = 0, l1 = 0, p0 = 0, p1 = 0;

            ESC_read (ESCREG_SM0, &p0, sizeof (p0));
            ESC_read (ESCREG_SM0 + 2, &l0, sizeof (l0));
            ESC_read (ESCREG_SM0 + 4, &c0m, sizeof (c0m));
            ESC_read (ESCREG_SM0 + 6, &a0m, sizeof (a0m));
            ESC_read (ESCREG_SM1, &p1, sizeof (p1));
            ESC_read (ESCREG_SM1 + 2, &l1, sizeof (l1));
            ESC_read (ESCREG_SM1 + 4, &c1m, sizeof (c1m));
            ESC_read (ESCREG_SM1 + 6, &a1m, sizeof (a1m));
            printf ("   MBX: run=%u xoe=%u outpost=%u backup=%u"
                    " | SM0 %04X len %u ctl %02X act %02X"
                    " | SM1 %04X len %u ctl %02X act %02X"
                    " | responses=%llu\n",
                    ESCvar.MBXrun, ESCvar.xoe, ESCvar.mbxoutpost,
                    ESCvar.mbxbackup,
                    etohs (p0), (unsigned)etohs (l0), c0m, a0m,
                    etohs (p1), (unsigned)etohs (l1), c1m, a1m,
                    (unsigned long long)mbx_responses);
         }

         /* The mirror, as the master should see it, one line per axis. Signed
          * values are printed as signed on purpose: this is the readout for
          * stack-review 2.1, and an ESI that still declares these unsigned
          * shows up as the master writing a number this slave never sees as
          * negative. */
         {
            unsigned k;

            for (k = 0; k < CMC_AXIS_COUNT; k++)
            {
               _Axis * ax = &Obj.axis[k];

               printf ("   PDO[%u]: mode %d->%d  pos %" PRId32 "->%" PRId32
                       "  vel %" PRId32 "->%" PRId32
                       "  torque %" PRId16 "->%" PRId16
                       "  ctrl %04X sts %04X\n",
                       k, ax->ModesOfOperation, ax->ModesOfOperationDisplay,
                       ax->TargetPosition, ax->PositionActual,
                       ax->TargetVelocity, ax->VelocityActual,
                       ax->TargetTorque, ax->TorqueActual,
                       ax->Controlword, ax->Statusword);
            }
         }
         fflush (stdout);

         n = 0;
         cycles = 0;
         last = b;
      }
   }

   return 0;
}
