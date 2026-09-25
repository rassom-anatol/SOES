# The CiA402 drive dictionary, and an application that brings it up against a
# real master.
#
# Not a drive: there is no CiA402 state machine here and there is not meant to
# be one (docs/cia402-roadmap.md, Phase 5 -- it belongs in the consuming motion
# controller, where it is testable without hardware). What this target exists
# for is the layer underneath, which cannot be tested any other way: that the
# generated SyncManager layout survives PREOP->SAFEOP, that the PDO mapping
# describes the image the master actually transfers, and that the ESI data types
# match the object list.
#
# Select it with -DSOES_DEMO=applications/cia402_drive.

add_executable (cia402_drive
  main.c
  objects.c
  generated/slave_objectlist.c
  )
find_package(Threads REQUIRED)
target_link_libraries (cia402_drive LINK_PUBLIC soes Threads::Threads)
target_compile_options (cia402_drive PRIVATE ${SOES_WARNING_FLAGS})
target_compile_definitions (cia402_drive PRIVATE ${SOES_DEFINITIONS})
