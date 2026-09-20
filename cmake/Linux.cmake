if(SPIDEV_VARIANT)
  # LAN9252 over Linux spidev plus the GPIO chardev. No vendor library, no
  # root requirement, and nothing Raspberry Pi specific.
  set (SOES_DEMO applications/lan9252_diag)
  set(HAL_SOURCES
	${SOES_SOURCE_DIR}/soes/hal/linux-lan9252-spidev/esc_hw.c
	${SOES_SOURCE_DIR}/soes/hal/linux-lan9252-spidev/esc_hw.h
	)
  include_directories(${SOES_SOURCE_DIR}/soes/hal/linux-lan9252-spidev)
elseif(RPI_VARIANT)
  set (SOES_DEMO applications/raspberry_lan9252demo)
  set(HAL_SOURCES
	${SOES_SOURCE_DIR}/soes/hal/raspberrypi-lan9252/esc_hw.c
	${SOES_SOURCE_DIR}/soes/hal/raspberrypi-lan9252/esc_hw.h
	)
  include_directories(${SOES_SOURCE_DIR}/soes/hal/raspberrypi-lan9252)
else()
  set(SOES_DEMO applications/linux_lan9252demo)
  set(HAL_SOURCES
	${SOES_SOURCE_DIR}/soes/hal/linux-lan9252/esc_hw.c
	)
endif()

include_directories(
  ${SOES_SOURCE_DIR}/soes/include/sys/gcc
  ${SOES_SOURCE_DIR}/${SOES_DEMO}
  )

