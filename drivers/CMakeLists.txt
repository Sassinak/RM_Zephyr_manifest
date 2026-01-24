# SPDX-License-Identifier: Apache-2.0

# zephyr-keep-sorted-stopadd_subdirectory_ifdef(CONFIG_BUZZER buzzer)# zephyr-keep-sorted-start# zephyr-keep-sorted-start
add_subdirectory_ifdef(CONFIG_BUZZER buzzer)
add_subdirectory_ifdef(CONFIG_CAN_RX_MANAGER can_rx_manager)
add_subdirectory_ifdef(CONFIG_MOTOR motor)
# zephyr-keep-sorted-stop
