board_runner_args(jlink "--device=STM32L083CZ" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
