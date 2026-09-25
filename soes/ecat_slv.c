/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */
#include <stddef.h>
#include "esc.h"
#include "esc_coe.h"
#include "esc_foe.h"
#include "esc_eoe.h"
#include "ecat_slv.h"

#define IS_RXPDO(index) ((index) >= 0x1600 && (index) < 0x1800)
#define IS_TXPDO(index) ((index) >= 0x1A00 && (index) < 0x1C00)

/* Global variables used by the stack */
uint8_t     MBX[MBXBUFFERS * MAX(MBXSIZE,MBXSIZEBOOT)];
_MBXcontrol MBXcontrol[MBXBUFFERS];
_SMmap      SMmap2[MAX_MAPPINGS_SM2];
_SMmap      SMmap3[MAX_MAPPINGS_SM3];
_ESCvar     ESCvar;

/* Private variables */
static volatile int watchdog;

#if MAX_MAPPINGS_SM2 > 0
static uint8_t rxpdo[MAX_RXPDO_SIZE] __attribute__((aligned (8)));
#else
extern uint8_t rxpdo[];
#endif

#if MAX_MAPPINGS_SM3 > 0
static uint8_t txpdo[MAX_TXPDO_SIZE] __attribute__((aligned (8)));
#else
extern uint8_t txpdo[];
#endif

/** Function to pre-qualify the incoming SDO download.
 *
 * @param[in] index      = index of SDO download request to check
 * @param[in] sub-index  = sub-index of SDO download request to check
 * @return SDO abort code, or 0 on success
 */
uint32_t ESC_download_pre_objecthandler (uint16_t index,
      uint8_t subindex,
      void * data,
      size_t size,
      uint16_t flags)
{
   if (IS_RXPDO (index) ||
       IS_TXPDO (index) ||
       index == RX_PDO_OBJIDX ||
       index == TX_PDO_OBJIDX)
   {
      uint8_t minSub = ((flags & COMPLETE_ACCESS_FLAG) == 0) ? 0 : 1;
      if (subindex > minSub && COE_maxSub (index) != 0)
      {
         return ABORT_SUBINDEX0_NOT_ZERO;
      }
   }

   if (ESCvar.pre_object_download_hook)
   {
      return (ESCvar.pre_object_download_hook) (index,
            subindex,
            data,
            size,
            flags);
   }

   return 0;
}

/** Hook called from the slave stack SDO Download handler to act on
 * user specified Index and Sub-index.
 *
 * @param[in] index      = index of SDO download request to handle
 * @param[in] sub-index  = sub-index of SDO download request to handle
 * @return SDO abort code, or 0 on success
 */
uint32_t ESC_download_post_objecthandler (uint16_t index, uint8_t subindex, uint16_t flags)
{
   if (ESCvar.post_object_download_hook != NULL)
   {
      return (ESCvar.post_object_download_hook)(index, subindex, flags);
   }

   return 0;
}

/** Function to pre-qualify the incoming SDO upload.
 *
 * @param[in] index      = index of SDO upload request to handle
 * @param[in] sub-index  = sub-index of SDO upload request to handle
 * @return SDO abort code, or 0 on success
 */
uint32_t ESC_upload_pre_objecthandler (uint16_t index,
      uint8_t subindex,
      void * data,
      size_t *size,
      uint16_t flags)
{
   if (ESCvar.pre_object_upload_hook != NULL)
   {
      return (ESCvar.pre_object_upload_hook) (index,
            subindex,
            data,
            size,
            flags);
   }

   return 0;
}

/** Hook called from the slave stack SDO Upload handler to act on
 * user specified Index and Sub-index.
 *
 * @param[in] index      = index of SDO upload request to handle
 * @param[in] sub-index  = sub-index of SDO upload request to handle
 * @return SDO abort code, or 0 on success
 */
uint32_t ESC_upload_post_objecthandler (uint16_t index, uint8_t subindex, uint16_t flags)
{
   if (ESCvar.post_object_upload_hook != NULL)
   {
      return (ESCvar.post_object_upload_hook)(index, subindex, flags);
   }

   return 0;
}

/** Hook called from the slave stack ESC_stopoutputs to act on state changes
 * forcing us to stop outputs. Here we can set them to a safe state.
 */
void APP_safe_state (void)
{
   DPRINT ("APP_safe_state\n");

   if(ESCvar.safe_state_override != NULL)
   {
      (ESCvar.safe_state_override)();
   }
}

/** Write local process data to Sync Manager 3, Master Inputs.
 */
static void txpdo_write_sm3 (void)
{
   if(ESCvar.txpdo_override != NULL)
   {
      (ESCvar.txpdo_override)();
   }
   else
   {
      if (MAX_MAPPINGS_SM3 > 0)
      {
         COE_pdoPack (txpdo, ESCvar.sm3mappings, SMmap3);
      }
      ESC_write (ESC_SM3_sma, txpdo, ESCvar.ESC_SM3_sml);
   }
}

/** Read Sync Manager 2 to local process data, Master Outputs.
 */
static void rxpdo_read_sm2 (void)
{
   if(ESCvar.rxpdo_override != NULL)
   {
      (ESCvar.rxpdo_override)();
   }
   else
   {
      ESC_read (ESC_SM2_sma, rxpdo, ESCvar.ESC_SM2_sml);
      if (MAX_MAPPINGS_SM2 > 0)
      {
         COE_pdoUnpack (rxpdo, ESCvar.sm2mappings, SMmap2);
      }
   }
}

/* Set the watchdog count value, don't have any affect when using
 * HW watchdog 0x4xx
 *
 * @param[in] watchdogcnt  = new watchdog count value
 */
void APP_setwatchdog (int watchdogcnt)
{
   CC_ATOMIC_SET(ESCvar.watchdogcnt, watchdogcnt);
}

/* --------------------------------------------------------- watchdog ------ */

/* State of the hardware watchdog check, latched while outputs are active.
 * Re-evaluated from scratch each time the application leaves an output state,
 * because the master rewrites the watchdog configuration as part of bringing
 * the slave back up.
 */
static enum
{
   HW_WD_UNCHECKED = 0,   /* configuration not read since outputs came up */
   HW_WD_ARMED,           /* master armed it; 0x0440 is authoritative */
   HW_WD_DISABLED,        /* master disabled it; fall back to the counter */
} hw_wd_state = HW_WD_UNCHECKED;

/* Check the ESC hardware process data watchdog.
 *
 * The ESC does not change AL state by itself when the SM watchdog expires: it
 * clears bit 0 of 0x0440 and expects the application to react. Nothing in this
 * stack read that register, so a configuration that disabled the software
 * counter in favour of the hardware one had no watchdog at all, and a master
 * that stopped sending frames produced no reaction -- the drive simply held its
 * last commanded setpoint.
 *
 * The configuration is the master's to write and ours to check. A process data
 * time of zero disables the watchdog, after which the status bit reads "active
 * or disabled" forever; trusting it in that state is worse than not having the
 * check, because it looks like protection. So the configuration is read once
 * per entry into an output state and, if the master left us unprotected, the
 * software counter takes over rather than the device refusing to run.
 *
 * @return 1 if the hardware watchdog is in charge, 0 if the caller should fall
 *         back to the software counter.
 */
static int hw_watchdog_check (void)
{
   if (((CC_ATOMIC_GET (ESCvar.App.state) & APPSTATE_OUTPUT) == 0) ||
       (ESCvar.ESC_SM2_sml == 0))
   {
      /* No outputs, so nothing resets the watchdog and it would read expired.
       * Also the point at which the latched verdict stops being valid. */
      hw_wd_state = HW_WD_UNCHECKED;
      return 1;
   }

   if (hw_wd_state == HW_WD_UNCHECKED)
   {
      uint16_t divider = 0;
      uint16_t pdtime = 0;

      ESC_read (ESCREG_WDDIVIDER, &divider, sizeof (divider));
      ESC_read (ESCREG_WDTIMEPDATA, &pdtime, sizeof (pdtime));
      divider = etohs (divider);
      pdtime = etohs (pdtime);

      if ((pdtime == 0) || (divider == 0))
      {
         DPRINT ("hw watchdog: master left it disabled (divider %u, time %u), "
                 "using the software counter\n", divider, pdtime);
         hw_wd_state = HW_WD_DISABLED;
      }
      else
      {
         DPRINT ("hw watchdog: armed, %u ticks of (%u+2)*40 ns\n",
                 pdtime, divider);
         hw_wd_state = HW_WD_ARMED;
      }
   }

   if (hw_wd_state == HW_WD_DISABLED)
   {
      return 0;
   }

   if ((ESC_WDstatus () & ESCREG_WDSTATUS_OK) == 0)
   {
      DPRINT ("hw watchdog expired\n");
      ESC_ALstatusgotoerror ((ESCsafeop | ESCerror), ALERR_WATCHDOG);
   }

   return 1;
}

/* Function to update local I/O, call read ethercat outputs, call
 * write ethercat inputs. Implement watch-dog counter to count-out if we have
 * made state change affecting the App.state.
 */
void DIG_process (uint8_t flags)
{
   /* Handle watchdog.
    *
    * Exactly one of the two watchdogs is in charge per cycle. The hardware one
    * measures what actually matters -- whether process data frames are still
    * arriving -- so when it is available the software counter is not merely
    * redundant but harmful: nothing would decrement it, and a zero
    * ESCvar.watchdogcnt then reads as permanently expired.
    */
   if((flags & DIG_PROCESS_WD_FLAG) > 0)
   {
      if (!ESCvar.use_hw_watchdog || !hw_watchdog_check ())
      {
         if (CC_ATOMIC_GET(watchdog) > 0)
         {
            CC_ATOMIC_SUB(watchdog, 1);
         }

         if ((CC_ATOMIC_GET(watchdog) <= 0) &&
             ((CC_ATOMIC_GET(ESCvar.App.state) & APPSTATE_OUTPUT) > 0) &&
              (ESCvar.ESC_SM2_sml > 0))
         {
            DPRINT("DIG_process watchdog expired\n");
            ESC_ALstatusgotoerror((ESCsafeop | ESCerror), ALERR_WATCHDOG);
         }
         else if(((CC_ATOMIC_GET(ESCvar.App.state) & APPSTATE_OUTPUT) == 0))
         {
            CC_ATOMIC_SET(watchdog, ESCvar.watchdogcnt);
         }
      }
   }

   /* Handle Outputs */
   if ((flags & DIG_PROCESS_RXPDO_FLAG) > 0)
   {
      if(((CC_ATOMIC_GET(ESCvar.App.state) & APPSTATE_OUTPUT) > 0) &&
         (ESCvar.ALevent & ESCREG_ALEVENT_SM2))
      {
         rxpdo_read_sm2();
         CC_ATOMIC_SET(watchdog, ESCvar.watchdogcnt);
         /* Set outputs */
         cb_apply_rxpdo();
      }
      else if (ESCvar.ALevent & ESCREG_ALEVENT_SM2)
      {
         ESC_read (ESC_SM2_sma, rxpdo, ESCvar.ESC_SM2_sml);
      }
   }

   /* Call application */
   if ((flags & DIG_PROCESS_APP_HOOK_FLAG) > 0)
   {
      /* Call application callback if set */
      if (ESCvar.application_hook != NULL)
      {
         (ESCvar.application_hook)();
      }
   }

   /* Handle Inputs */
   if ((flags & DIG_PROCESS_TXPDO_FLAG) > 0)
   {
      if(CC_ATOMIC_GET(ESCvar.App.state) > 0)
      {
         /* Update inputs */
         cb_update_txpdo();
         txpdo_write_sm3();
      }
   }
}

/*
 * Handler for SM change, SM0/1, AL CONTROL and EEPROM events, the application
 * control what interrupts that should be served and re-activated with
 * event mask argument
 */
void ecat_slv_worker (uint32_t event_mask)
{
   do
   {
      /* Check the state machine */
      ESC_state();
      /* Check the SM activation event */
      ESC_sm_act_event();

      /* Check mailboxes */
      while ((ESC_mbxprocess() > 0) || (ESCvar.txcue > 0))
      {
         ESC_coeprocess();
#if USE_FOE
         ESC_foeprocess();
#endif
#if USE_EOE
         ESC_eoeprocess();
#endif
         ESC_xoeprocess();
      }
#if USE_EOE
      ESC_eoeprocess_tx();
#endif
      /* Call emulated eeprom handler if set */
      if (ESCvar.esc_hw_eep_handler != NULL)
      {
         (ESCvar.esc_hw_eep_handler)();
      }

      CC_ATOMIC_SET(ESCvar.ALevent, ESC_ALeventread());

   }while(ESCvar.ALevent & event_mask);

   ESC_ALeventmaskwrite(ESC_ALeventmaskread() | event_mask);
}

/*
 * Polling function. It should be called periodically for an application 
 * when only SM2/DC interrupt is active.
 * Read and handle events for the EtherCAT state, status, mailbox and eeprom.
 */
void ecat_slv_poll (void)
{
   /* Refresh the AL event register.
    *
    * This was a read of ESCREG_LOCALTIME into ESCvar.Time, which nothing in
    * this stack or its applications reads. What the cycle actually depends on
    * is ESCvar.ALevent, which that read only refreshed by accident, because a
    * port may append an AL event read to every access to mimic the ET1x00.
    * Asking for the register the cycle needs costs no more than asking for one
    * it does not, and on a port where the tail is a second bus access it costs
    * half as much as asking for both.
    */
   CC_ATOMIC_SET (ESCvar.ALevent, ESC_ALeventread ());

   /* Check the state machine */
   ESC_state();
   /* Check the SM activation event */
   ESC_sm_act_event();

   /* Check mailboxes */
   if (ESC_mbxprocess())
   {
      ESC_coeprocess();
#if USE_FOE
      ESC_foeprocess();
#endif
#if USE_EOE
      ESC_eoeprocess();
#endif
      ESC_xoeprocess();
   }
#if USE_EOE
   ESC_eoeprocess_tx();
#endif

   /* Call emulated eeprom handler if set */
   if (ESCvar.esc_hw_eep_handler != NULL)
   {
      (ESCvar.esc_hw_eep_handler)();
   }
}

/*
 * Poll all events in a free-run application
 */
void ecat_slv (void)
{
   ecat_slv_poll();
   DIG_process(DIG_PROCESS_WD_FLAG | DIG_PROCESS_RXPDO_FLAG |
         DIG_PROCESS_APP_HOOK_FLAG | DIG_PROCESS_TXPDO_FLAG);
}

/*
 * Initialize the slave stack.
 */
int ecat_slv_init (esc_cfg_t * config)
{
   uint32_t retries;

   DPRINT ("Slave stack init started\n");

   /* Init watchdog */
   watchdog = config->watchdog_cnt;

   /* Call stack configuration */
   ESC_config (config);
   /* Call HW init */
   if (ESC_init (config) != 0)
   {
      DPRINT ("ESC_init failed, aborting stack init\n");
      return -1;
   }

   /*  wait until ESC is started up, bounded so that an ESC which never
    *  reports a link does not spin here forever with no diagnostic */
   retries = DLSTATUS_WAIT_RETRIES;
   while ((ESCvar.DLstatus & 0x0001) == 0)
   {
      if (retries-- == 0)
      {
         DPRINT ("timeout waiting for ESC start-up, DLstatus 0x%04x\n",
                 ESCvar.DLstatus);
         return -1;
      }
      ESC_read (ESCREG_DLSTATUS, (void *) &ESCvar.DLstatus,
                sizeof (ESCvar.DLstatus));
      ESCvar.DLstatus = etohs (ESCvar.DLstatus);
   }

#if USE_FOE
   /* Init FoE */
   FOE_init ();
#endif

#if USE_EOE
   /* Init EoE */
   EOE_init ();
#endif

   /* reset ESC to init state */
   ESC_ALstatus (ESCinit);
   ESC_ALerror (ALERR_NONE);
   ESC_stopmbx ();
   ESC_stopinput ();
   ESC_stopoutput ();
   /* Init Object Dictionary default values */
   COE_initDefaultValues ();

   return 0;
}
