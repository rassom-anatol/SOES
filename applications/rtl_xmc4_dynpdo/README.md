# Retained as reference, not built

This application is not part of any build target. It is kept because it is the
only in-tree example of two things the roadmap depends on:

* **Dynamic PDO mapping** — `slave_objectlist.c` shows 0x1C12/0x1C13 as
  `ATYPE_RWpre` with backing storage, the form a master can reconfigure in
  PREOP.
* **Distributed Clocks configuration** — `main.c` contains `dc_checker`
  (setting `ESCvar.dcsync` and `synccounterlimit` from 0x10F1:02) and the
  PREOP-to-SAFEOP branch of `cb_state_change` that writes an initial TxPDO,
  without which SM3 never fires and the master stalls at SAFEOP.

Both are referenced by Phase 3 of `docs/cia402-roadmap.md`. It targets rt-kernel
on XMC4 and cannot be compiled on a Linux host.
