/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

#ifndef __ECAT_SLV_H__
#define __ECAT_SLV_H__

#include "options.h"
#include "esc.h"

/** Refresh the objects the TxPDO is packed from.
 *
 * Called immediately before the TxPDO is packed and written to SM3, so the
 * application samples its hardware here. TxPDO is the slave-to-master
 * direction: whatever is written to the mapped objects during this call is
 * what the master reads on the next cycle.
 */
void cb_update_txpdo (void);

/** Apply the objects the RxPDO was unpacked into.
 *
 * Called immediately after the RxPDO has been read from SM2 and unpacked, so
 * the application drives its hardware here. RxPDO is the master-to-slave
 * direction: the mapped objects already hold this cycle's commands.
 */
void cb_apply_rxpdo (void);

/** Set the watchdog count value
 *
 * @param[in] watchdogcnt  = new watchdog count value
 */
void APP_setwatchdog (int watchdogcnt);

/* Phase selectors for DIG_process. Under Distributed Clocks the two PDO
 * phases are driven from different events -- RxPDO on the SM2 interrupt,
 * TxPDO on the SYNC0 edge -- so they must be separately selectable.
 */
#define DIG_PROCESS_TXPDO_FLAG      0x01
#define DIG_PROCESS_RXPDO_FLAG      0x02
#define DIG_PROCESS_WD_FLAG         0x04
#define DIG_PROCESS_APP_HOOK_FLAG   0x08
/** Implements the watch-dog counter to count if we should make a state change
 * due to missing incoming SM2 events. Updates local I/O and run the application
 * in the following order, call read EtherCAT outputs, execute user provided
 * application hook and call write EtherCAT inputs.
 *
 * @param[in]   flags     = User input what to execute
 */
void DIG_process (uint8_t flags);

/**
 * Handler for SM change, SM0/1, AL CONTROL and EEPROM events, the application
 * control what interrupts that should be served and re-activated with
 * event mask argument
 *
 * @param[in]   event_mask = Event mask for interrupts to serve and re-activate
 *                           after served
 */
void ecat_slv_worker (uint32_t event_mask);

/**
 * Poll SM0/1, EEPROM and AL CONTROL events in a SM/DC synchronization
 * application
 */
void ecat_slv_poll (void);

/**
 * Poll all events in a free-run application
 */
void ecat_slv (void);

/**
 * Initialize the slave stack
 *
 * @param[in]   config     = User input how to configure the stack
 * @return 0 on success, non-zero if the hardware could not be initialised or
 *         the ESC did not report a link within DLSTATUS_WAIT_RETRIES polls.
 *         Callers must check: entering the cyclic loop after a failed init
 *         leaves the stack talking to nothing.
 */
int ecat_slv_init (esc_cfg_t * config);

#endif /* __ECAT_SLV_H__ */
