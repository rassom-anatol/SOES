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
# The variant selects which generated dictionary is compiled in. It is a build
# option rather than a branch precisely so a consumer can pin one commit of this
# repository and choose its axis count at configure time:
#
#   -DSOES_DEMO=applications/cia402_drive -DCMC_VARIANT=cmc_drive_4ax

set (CMC_VARIANT cmc_drive_1ax CACHE STRING
     "CiA402 dictionary variant, a directory name under generated/")

set (CMC_GENERATED ${CMAKE_CURRENT_SOURCE_DIR}/generated/${CMC_VARIANT})
if (NOT EXISTS ${CMC_GENERATED}/utypes.h)
  message (FATAL_ERROR
    "No generated dictionary for variant '${CMC_VARIANT}'. "
    "Available: see applications/cia402_drive/generated/")
endif()

add_executable (cia402_drive
  main.c
  objects.c
  ${CMC_GENERATED}/slave_objectlist.c
  )
find_package(Threads REQUIRED)
target_link_libraries (cia402_drive LINK_PUBLIC soes Threads::Threads)
target_compile_options (cia402_drive PRIVATE ${SOES_WARNING_FLAGS})
target_compile_definitions (cia402_drive PRIVATE ${SOES_DEFINITIONS})

message (STATUS "CiA402 variant: ${CMC_VARIANT}")
