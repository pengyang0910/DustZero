/*
 * @Description:
 * @version:
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-08-18 12:02:09
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2020-09-22 15:51:01
 */
#include "platform/XBase/HWDataInterface.h"
#include <xutils/xutils.h>

HWInputDataInterface::HWInputDataInterface() {
  memset(&m_mcu_mr133, 0, sizeof(mcu_mr133_data_t));
  memset(m_laserPoints, 0, sizeof(m_laserPoints));
  memset(m_laserIntensity, 0, sizeof(m_laserIntensity));
  memset(m_xd1q_points, 0, sizeof(m_xd1q_points));
  laser_update = false;
  odom_update = false;
  laserNearMode = false;
  bHomeMode = false;
}

double HWInputDataInterface::getImuYaw() {
  double q0, q1, q2, q3;
  double yaw;
  q0 = m_mcu_mr133.q0 * 0.0001;
  q1 = m_mcu_mr133.q1 * 0.0001;
  q2 = m_mcu_mr133.q2 * 0.0001;
  q3 = m_mcu_mr133.q3 * 0.0001;

  yaw = atan2(2 * (q1 * q2 + q0 * q3),
              q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3); // yaw
  return yaw;
}

HWOutputDataInterface::HWOutputDataInterface() {
  memset(&m_mr133_mcu, 0, sizeof(mr133_mcu_data_t));
}

HWOutputDataInterface::~HWOutputDataInterface() {}

void HWOutputDataInterface::updateOutput(uint16_t seq) {
  /* fill commander info */
  m_mr133_mcu.frame_header = FRAME_HEAD;
  m_mr133_mcu.seq = seq;
  m_mr133_mcu.fan_duty_cycle = 0; // m_mcu_mr133.fan_duty_cycle;

  m_mr133_mcu.brush_r_duty_cycle = 0; // m_mcu_mr133->right_brush_duty_cycle;
  m_mr133_mcu.brush_m_duty_cycle = 0; // m_mcu_mr133->main_brush_duty_cycle;
#ifdef FJ212_PROTOCOL
  m_mr133_mcu.mop_motor_duty_cycle = 0; // m_mcu_mr133->left_brush_duty_cycle;
  m_mr133_mcu.mid_brush_lift_motor_duty_cycle = 0;
#else
  m_mr133_mcu.brush_l_duty_cycle = 0; // m_mcu_mr133->left_brush_duty_cycle;
  m_mr133_mcu.pump_duty_cycle = 0;
#endif

  m_mr133_mcu.left_wheel_duty_cycle = 0;
  m_mr133_mcu.right_wheel_duty_cycle = 0;
  m_mr133_mcu.v_tar_mm_s = 0;
  m_mr133_mcu.w_tar_mrad_s = 0;

#ifdef FJ212_PROTOCOL
  m_mr133_mcu.cmd.bit_field.ultrasonic_debug = 0;
#else
  m_mr133_mcu.cmd.bit_field.cut_enable = 0;
#endif

  /* should exposed to user control*/
  m_mr133_mcu.cmd.bit_field.wheel_enable = 1;

  m_mr133_mcu.cmd.bit_field.clear_err = 0;
  m_mr133_mcu.frame_tail = FRAME_TAIL;

  // std::cout<<"update state"<<std::endl;
}

int hParseMCUFrame(ring_buf &m_mcuDataBuf, mcu_mr133_data_t *m_mcu_mr133_Pt) {
  static uint16_t last_parse_seq = 0;
  int lValidDataSize = m_mcuDataBuf.validDataSize();
  if ((size_t)lValidDataSize < MCU_RECEIVE_FRAME_SIZE)
    return 0;

  // 1. check for header & tail
  // if ((*((uint32_t*)(uart1_dma_receive_buf + out_idx)) == FRAME_HEAD))
  if (m_mcuDataBuf.peek(0) == (FRAME_HEAD & 0xFF) &&
      m_mcuDataBuf.peek(1) == ((FRAME_HEAD >> 8) & 0xFF) &&
      m_mcuDataBuf.peek(2) == ((FRAME_HEAD >> 16) & 0xFF) &&
      m_mcuDataBuf.peek(3) == ((FRAME_HEAD >> 24) & 0xFF)) {
    // header found
    // if (*((uint32_t*)(uart1_dma_receive_buf + (out_idx + frameSize - 4) %
    // 1024)) == FRAME_TAIL)
    if (m_mcuDataBuf.peek(MCU_RECEIVE_FRAME_SIZE - 4) == (FRAME_TAIL & 0xFF) &&
        m_mcuDataBuf.peek(MCU_RECEIVE_FRAME_SIZE - 3) ==
            ((FRAME_TAIL >> 8) & 0xFF) &&
        m_mcuDataBuf.peek(MCU_RECEIVE_FRAME_SIZE - 2) ==
            ((FRAME_TAIL >> 16) & 0xFF) &&
        m_mcuDataBuf.peek(MCU_RECEIVE_FRAME_SIZE - 1) ==
            ((FRAME_TAIL >> 24) & 0xFF)) {
      // tail found
      m_mcuDataBuf.retrieve((uint8_t *)(m_mcu_mr133_Pt),
                            MCU_RECEIVE_FRAME_SIZE);
      last_parse_seq = m_mcu_mr133_Pt->seq;
      // printf("out idx is %d, in_idx is %d, seq is 0x%x\n",
      // m_mcuDataBuf.out_idx, m_mcuDataBuf.in_idx, last_seq);
      return 1;
    } else {
      // tail not found, discard data
      m_mcuDataBuf.retrieve(NULL, MCU_RECEIVE_FRAME_SIZE);
      printf("[%d]MCU TAIL NOT FOUND!!!\n", getTimeStap_ms());
      return -1;
    }
  } else {
#if 1 // dump ring buffer
    printf("[%u]dump ring buffer, buffer size is %d, in_idx is %d, out_idx is "
           "%d, last valid parse seq is %d\n",
           (uint32_t)getTimeStap_us(), m_mcuDataBuf.bufferSize,
           m_mcuDataBuf.in_idx, m_mcuDataBuf.out_idx, last_parse_seq);
    for (int i = 0; i < m_mcuDataBuf.bufferSize; i++) {
      printf(" %2x", m_mcuDataBuf.data[i]);
    }
    printf("\n");
#endif
    // header not found, pop unused data
    for (int i = 0; i <= lValidDataSize - 4; i++) {
      // if (*((uint32_t*)(uart1_dma_receive_buf + out_idx)) == FRAME_HEAD)
      if (m_mcuDataBuf.peek(0) == (FRAME_HEAD & 0xFF) &&
          m_mcuDataBuf.peek(1) == ((FRAME_HEAD >> 8) & 0xFF) &&
          m_mcuDataBuf.peek(2) == ((FRAME_HEAD >> 16) & 0xFF) &&
          m_mcuDataBuf.peek(3) == ((FRAME_HEAD >> 24) & 0xFF)) {
        break; // find another header
      }

      m_mcuDataBuf.retrieve(NULL, 1);
    }
    printf("[%d]uart parse exception, buffer overrun???\n", getTimeStap_ms());
    return -2;
  }
  return -3;
}