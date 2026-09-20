# LAN9252 over Linux spidev plus the GPIO character device. No vendor library,
# no root requirement, and nothing Raspberry Pi specific: spidev and the
# gpiochip UAPI are generic Linux interfaces.
set (SOES_DEMO applications/lan9252_diag)

set (HAL_SOURCES
  ${SOES_SOURCE_DIR}/soes/hal/linux-lan9252-spidev/esc_hw.c
  ${SOES_SOURCE_DIR}/soes/hal/linux-lan9252-spidev/esc_hw.h
  )

include_directories(
  ${SOES_SOURCE_DIR}/soes/hal/linux-lan9252-spidev
  ${SOES_SOURCE_DIR}/soes/include/sys/gcc
  ${SOES_SOURCE_DIR}/${SOES_DEMO}
  )
