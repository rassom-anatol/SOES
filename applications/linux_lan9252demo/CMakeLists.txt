
add_executable (demo
  main.c
  slave_objectlist.c
  )
target_link_libraries(demo LINK_PUBLIC soes)
target_compile_options(demo PRIVATE ${SOES_WARNING_FLAGS})
