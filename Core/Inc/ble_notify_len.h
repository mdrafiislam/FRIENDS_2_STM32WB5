/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ble_notify_len.h
  * @brief   Single source of truth for the P2P notify characteristic length.
  *
  * USER-OWNED FILE. CubeMX has never generated this.
  *
  * Why this exists: the notify length was hardcoded as the literal 20 in THREE
  * places that must agree, two of them in ST middleware
  * (Middlewares/ST/STM32_WPAN/ble/svc/Src/p2p_stm.c):
  *
  *   1. aci_gatt_add_char()        - the characteristic's max value length
  *   2. aci_gatt_update_char_value() - the length actually notified
  *   3. sizeof(BleMagPacket) / sizeof(BlePuffPacket) in p2p_server_app.c
  *
  * If (1) or (2) drifts from (3) the failure is silent: a short notification
  * truncates the packet, or a long one over-reads past the struct. Both packets
  * go through the same fixed-length update call, so they MUST be the same size -
  * that is asserted at compile time in p2p_server_app.c.
  *
  * Sizing headroom, for anyone extending the packet later:
  *   notification payload <= negotiated ATT_MTU - 3
  *   CFG_BLE_MAX_ATT_MTU is 156 (Core/Inc/app_conf.h), so this device supports
  *   up to 153 bytes of payload without touching app_conf.h. Raising that define
  *   to 247 would allow the LE maximum of 244. At 26 bytes we are nowhere near
  *   either ceiling - the binding constraint was always the hardcoded 20 in
  *   p2p_stm.c, never the MTU.
  *
  * ASSUMPTION: the central must negotiate an ATT_MTU of at least
  * BLE_NOTIFY_PACKET_LEN + 3 (29 here). I could not verify from any document in
  * project knowledge whether this BLE stack truncates or errors when the
  * negotiated MTU is smaller, so the host-side parsers check the received length
  * on every packet and report a short read loudly rather than misparsing it.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef BLE_NOTIFY_LEN_H
#define BLE_NOTIFY_LEN_H

/* Bytes sent in every P2P notify characteristic update.
 * v3 mag packet: 1 version + 1 flags + 2 sequence + 4 timestamp
 *              + 6 raw s1 + 6 raw s2 + 6 zeroed delta = 26. */
#define BLE_NOTIFY_PACKET_LEN   26U

#endif /* BLE_NOTIFY_LEN_H */
