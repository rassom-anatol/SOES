# The CiA402 drive dictionary.
#
# Not an application yet: the CiA402 state machine that would drive it lives in
# the consuming motion controller rather than here (docs/cia402-roadmap.md,
# Phase 5). What this target buys today is that the generated object list and
# the generated ecat_options.h are compiled by the ordinary build, under the
# same warning flags as the rest of the tree. Before it existed, nothing in any
# build compiled either file, which is how a dictionary that dereferences NULL
# on a scan of 0x1008 went unnoticed.
#
# Select it with -DSOES_DEMO=applications/cia402_drive.

add_library (cia402_drive STATIC
  objects.c
  generated/slave_objectlist.c
  )
target_link_libraries (cia402_drive LINK_PUBLIC soes)
target_compile_options (cia402_drive PRIVATE ${SOES_WARNING_FLAGS})
target_compile_definitions (cia402_drive PRIVATE ${SOES_DEFINITIONS})
