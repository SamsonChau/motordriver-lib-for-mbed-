#include "vesc_can.h"
#include "mbed.h"
#include <stdint.h>

void vesc_can::vesc_can_init(CAN* _CAN) {
  can1 = _CAN;
  can1->frequency(1000000);
}

void vesc_can::vesc_can_set_duty(uint8_t controller_id, float duty) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

void vesc_can::comm_can_set_current(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

void vesc_can::comm_can_set_current_brake(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

void vesc_can::comm_can_set_rpm(uint8_t controller_id, float rpm) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)rpm, &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

void vesc_can::comm_can_set_pos(uint8_t controller_id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

int vesc_can::vesc_can_read() {
  CANMessage RxMsg;
  can_status_msg *stat_tmp;
  if (can1->read(RxMsg)) {

    uint8_t id = RxMsg.id & 0xFF; //take the lower 8 bits for the ID
    CAN_PACKET_ID cmd = (CAN_PACKET_ID) (RxMsg.id >> 8); // Take the upper bits as the comand
    switch (cmd) {
      case CAN_PACKET_SET_DUTY:
        ind = 0;
        break;

      case CAN_PACKET_SET_CURRENT:
        ind = 0;
        break;

      case CAN_PACKET_SET_CURRENT_BRAKE:
        ind = 0;
        break;

      case CAN_PACKET_SET_RPM:
        ind = 0;
        break;

      case CAN_PACKET_SET_POS:
        ind = 0;
        break;

      case CAN_PACKET_FILL_RX_BUFFER:
        memcpy(rx_buffer + RxMsg.data[0], RxMsg.data + 1, RxMsg.len - 1);
        break;

      case CAN_PACKET_FILL_RX_BUFFER_LONG:
        rxbuf_ind = (unsigned int)RxMsg.data[0] << 8;
        rxbuf_ind |= RxMsg.data[1];
        if (rxbuf_ind < RX_BUFFER_SIZE) {
          memcpy(rx_buffer + rxbuf_ind, RxMsg.data + 2, RxMsg.len - 2);
        }
        break;

      case CAN_PACKET_PROCESS_RX_BUFFER:
        ind = 0;
        rx_buffer_last_id = RxMsg.data[ind++];
        RxMsg.data[ind++];
        rxbuf_len = (unsigned int)RxMsg.data[ind++] << 8;
        rxbuf_len |= (unsigned int)RxMsg.data[ind++];

        if (rxbuf_len > RX_BUFFER_SIZE) {
          break;
        }

        crc_high = RxMsg.data[ind++];
        crc_low = RxMsg.data[ind++];

        if (crc16(rx_buffer, rxbuf_len) == ((unsigned short) crc_high << 8 | (unsigned short) crc_low)) {
        can_process_packet(rx_buffer, rxbuf_len);
        }
        break;

      case CAN_PACKET_PROCESS_SHORT_BUFFER:
        ind = 0;
        rx_buffer_last_id = RxMsg.data[ind++];
        RxMsg.data[ind++];
        can_process_packet(RxMsg.data + ind, RxMsg.len - ind);
        break;

      case CAN_PACKET_STATUS:
        for (i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++) {
          stat_tmp = &stat_msgs[i];
          if (stat_tmp->id == id || stat_tmp->id == -1) {
            ind = 0;
            stat_tmp->id = id;
            stat_tmp->rpm = (float)buffer_get_int32(RxMsg.data, &ind);
            stat_tmp->current = (float)buffer_get_int16(RxMsg.data, &ind) / 10.0;
            stat_tmp->duty = (float)buffer_get_int16(RxMsg.data, &ind) / 1000.0;
            break;
          } 
        }
      default:
        break;
    }
  }
  else{
    return 0; // if the message cant be read return 0
  };
  return 1;
}



//Helper methods
bool vesc_can::sendPacket(uint8_t id, uint8_t packet[], int32_t len) {
  CANMessage  Txmsg;
  Txmsg.id = id;
  Txmsg.len = len;
  memcpy(Txmsg.data, packet, len * sizeof(uint8_t));
  return (bool) can1->write(Txmsg);
}

void vesc_can::can_process_packet(unsigned char *data, unsigned int len){
   bldc_interface_process_packet(data,len);
   can_idle = 1;
}

unsigned short vesc_can::crc16(unsigned char *buf, unsigned int len) {
  unsigned int i;
  unsigned short cksum = 0;
  for (i = 0; i < len; i++) {
    cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
  }
  return cksum;
}


