
# NOTE: add headers to make them show up in an IDE
add_library (soes
  esc.c
  esc.h
  esc_coe.c
  esc_coe.h
  esc_foe.c
  esc_foe.h
  esc_eoe.c
  esc_eoe.h
  esc_eep.c
  esc_eep.h
  ecat_slv.c
  ecat_slv.h
  options.h
  ${HAL_SOURCES}
  )

include_directories(${HAL_INCLUDES})

target_compile_options (soes PRIVATE ${SOES_WARNING_FLAGS})
target_compile_definitions (soes PUBLIC ${SOES_DEFINITIONS})

install (TARGETS soes DESTINATION lib)
install (FILES
  esc.h
  esc_coe.h
  esc_foe.h
  esc_eoe.h
  esc_eep.h
  DESTINATION include)
