
add_executable (lan9252_diag
  main.c
  slave_objectlist.c
  )
target_link_libraries(lan9252_diag LINK_PUBLIC soes)
target_compile_options(lan9252_diag PRIVATE ${SOES_WARNING_FLAGS})
target_compile_definitions(lan9252_diag PRIVATE ${SOES_DEFINITIONS})
