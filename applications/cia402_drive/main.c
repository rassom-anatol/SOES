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

/* Mailbox activity, counted per cycle from the AL event register. This splits
 * the one question worth asking when a master's CoE object list comes back
 * empty: whether the master is sending mailbox traffic that this stack fails to
 * answer, or whether it is sending nothing at all. The two have entirely
 * different causes and nothing else distinguishes them from here.
 */
static uint64_t sm0_events = 0;
static uint64_t sm1_events = 0;

static void cb_state_change (uint8_t * as, uint8_t * an);

/** Master outputs have arrived: SM2 has been read and unpacked into Obj.
 *
 * Runs before the TxPDO is packed in the same DIG_process call, so a value
 * written here reaches the master in the same cycle.
 */
void cb_apply_rxpdo (void)
{
   _Axis * a = &Obj.axis[0];

   a->PositionActual          = a->TargetPosition;
   a->VelocityActual          = a->TargetVelocity;
   a->TorqueActual            = a->TargetTorque;
   a->ModesOfOperationDisplay = a->ModesOfOperation;

   /* Zero by construction while the mirror is exact. It is mapped rather than
    * omitted so that the master's interpretation of a signed 32-bit entry that
    * legitimately goes negative is exercised by the position mirror above. */
   a->FollowingErrorActual = a->TargetPosition - a->PositionActual;

   rx_calls++;
}

/** Slave inputs are about to be packed into SM3 for the master to read. */
void cb_update_txpdo (void)
{
   _Axis * a = &Obj.axis[0];

   /* Switch on disabled: the honest CiA402 state for a device with no power
    * stage. It is a constant here rather than a computed value precisely
    * because the state machine that should compute it is not in this repository
    * -- a plausible-looking statusword from a stub would be worse than an
    * obviously static one. */
   a->Statusword = 0x0040;

   /* No gate driver and no motor to report on. */
   a->ErrorCode        = 0;
   a->GateDriverFaults = 0;
   a->DriveStatusFlags = 0;

   tx_calls++;
}

/** Called from ESC_stopoutputs when the stack leaves an output state, whether
 * because the master asked or because the watchdog expired. With no drive
 * attached the safe state is simply to forget the setpoints, so that a stale
 * target cannot be mirrored back and read as if it were still commanded.
 */
static void app_safe_state (void)
{
   _Axis * a = &Obj.axis[0];

   a->TargetPosition = 0;
   a->TargetVelocity = 0;
   a->TargetTorque   = 0;
   a->PositionActual = 0;
   a->VelocityActual = 0;
   a->TorqueActual   = 0;

   printf ("safe state: setpoints cleared\n");
}

static esc_cfg_t config =
{
   .user_arg = &hw_cfg,
   .use_interrupt = 0,
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

   printf ("cia402_drive: %s at %u Hz\n", hw_cfg.spidev,
           (unsigned)hw_cfg.spi_speed_hz);

   if (ecat_slv_init (&config) != 0)
   {
      printf ("FAIL: stack init failed\n");
      return 1;
   }
   printf ("stack init OK, entering cyclic loop\n");

   clock_gettime (CLOCK_MONOTONIC, &last);
   for (;;)
   {
      uint64_t c0 = ESC_hw_spi_count ();

      clock_gettime (CLOCK_MONOTONIC, &a);
      ecat_slv ();
      clock_gettime (CLOCK_MONOTONIC, &b);

      if (ESCvar.ALevent & ESCREG_ALEVENT_SM0)
      {
         sm0_events++;
      }
      if (ESCvar.ALevent & ESCREG_ALEVENT_SM1)
      {
         sm1_events++;
      }

      if (n < NS)
      {
         xfers[n] = ESC_hw_spi_count () - c0;
         samples[n++] = (uint64_t)(b.tv_sec - a.tv_sec) * 1000000000ull +
                        (uint64_t)(b.tv_nsec - a.tv_nsec);
      }

      if (b.tv_sec - last.tv_sec >= 5)
      {
         _Axis * ax = &Obj.axis[0];
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
            uint64_t sum = 0, xsum = 0;
            uint32_t i;
            uint64_t * srt = malloc (n * sizeof (uint64_t));

            if (srt != NULL)
            {
               memcpy (srt, samples, n * sizeof (uint64_t));
               for (i = 0; i < n; i++)
               {
                  sum += srt[i];
                  xsum += xfers[i];
               }
               qsort (srt, n, sizeof (uint64_t), cmp_u64);
               printf ("ecat_slv n=%u  median %.1f  p99 %.1f  max %.1f us"
                       "  over %.1f transfers\n",
                       n, (double)srt[n / 2] / 1000.0,
                       (double)srt[(n * 99) / 100] / 1000.0,
                       (double)srt[n - 1] / 1000.0,
                       (double)xsum / (double)n);
               free (srt);
            }
         }

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
                    " | events SM0=%llu SM1=%llu\n",
                    ESCvar.MBXrun, ESCvar.xoe, ESCvar.mbxoutpost,
                    ESCvar.mbxbackup,
                    etohs (p0), (unsigned)etohs (l0), c0m, a0m,
                    etohs (p1), (unsigned)etohs (l1), c1m, a1m,
                    (unsigned long long)sm0_events,
                    (unsigned long long)sm1_events);
         }

         /* The mirror, as the master should see it. Signed values are printed
          * as signed on purpose: this line is the readout for stack-review
          * 2.1, and an ESI that still declares these unsigned shows up as the
          * master writing a number this slave never sees as negative. */
         printf ("   PDO: mode %d->%d  pos %" PRId32 "->%" PRId32
                 "  vel %" PRId32 "->%" PRId32
                 "  torque %" PRId16 "->%" PRId16
                 "  ctrl %04X sts %04X\n",
                 ax->ModesOfOperation, ax->ModesOfOperationDisplay,
                 ax->TargetPosition, ax->PositionActual,
                 ax->TargetVelocity, ax->VelocityActual,
                 ax->TargetTorque, ax->TorqueActual,
                 ax->Controlword, ax->Statusword);
         fflush (stdout);

         n = 0;
         last = b;
      }
   }

   return 0;
}
