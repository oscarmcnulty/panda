#pragma once

// VW MLB without a stock lane assist camera: openpilot is the only HCA_01 sender, and the EPS latches an HCA fault for
// the rest of the drive if HCA_01 isn't on the bus within a few seconds of power-up. The device takes 30s+ to boot from
// cold, so the panda sends a neutral HCA_01 (status 3, zero torque) on bus 0 itself while the EPS is talking and nobody
// else is sending HCA_01. openpilot takes over as soon as its own HCA_01 gets through the safety tx hook.

#define HCA_HEARTBEAT_HCA_01 0x126U
#define HCA_HEARTBEAT_PERIOD_US 20000U       // HCA_01 is 50Hz
#define HCA_HEARTBEAT_EPS_TIMEOUT_US 100000U  // LH_EPS_03 is 100Hz
#define HCA_HEARTBEAT_HOST_TIMEOUT_US 100000U

// HCA_01 checksums for counters 0-15 of the neutral frame below (CRC8H2F with the HCA_01 magic, from opendbc's packer)
static const uint8_t hca_heartbeat_checksums[16] = {0xE0U, 0x84U, 0x28U, 0x4CU, 0x5FU, 0x3BU, 0x97U, 0xF3U,
                                                    0xB1U, 0xD5U, 0x79U, 0x1DU, 0x0EU, 0x6AU, 0xC6U, 0xA2U};

bool hca_heartbeat_eps_awake = false;  // valid LH_EPS_03 seen recently, also takes bus 0 out of silent mode
static bool hca_heartbeat_foreign = false;  // another node sends HCA_01 (stock camera), never send
static bool hca_heartbeat_host_seen = false;
static uint32_t hca_heartbeat_eps_ts = 0U;
static uint32_t hca_heartbeat_host_ts = 0U;
static uint32_t hca_heartbeat_tx_ts = 0U;
static uint8_t hca_heartbeat_counter = 0U;
static bool hca_heartbeat_bus0_silent = true;  // bus 0 core is in bus monitoring mode

static bool hca_heartbeat_should_send(uint32_t now) {
  bool host_sending = hca_heartbeat_host_seen && (get_ts_elapsed(now, hca_heartbeat_host_ts) < HCA_HEARTBEAT_HOST_TIMEOUT_US);
  return hca_heartbeat_eps_awake && !hca_heartbeat_foreign && !host_sending && !(can_silent && hca_heartbeat_bus0_silent) &&
         (current_safety_mode != SAFETY_ALLOUTPUT);
}

static void hca_heartbeat_send(void) {
  CANPacket_t msg = {0};
  msg.bus = 0U;
  msg.addr = HCA_HEARTBEAT_HCA_01;
  msg.data_len_code = 8U;
  msg.data[0] = hca_heartbeat_checksums[hca_heartbeat_counter];
  msg.data[1] = 0x30U | hca_heartbeat_counter;  // HCA_01_Vib_Freq 18Hz
  msg.data[4] = 3U;                             // HCA_01_Status_HCA off
  msg.data[5] = 0xFEU;                          // EA_ACC_Wunschgeschwindigkeit 327.36 (not set)
  msg.data[6] = 0x07U;
  can_set_checksum(&msg);
  can_send(&msg, 0U, true);
  hca_heartbeat_counter = (hca_heartbeat_counter + 1U) & 0xFU;
}

// every received CAN frame
void hca_heartbeat_rx_hook(const CANPacket_t *msg) {
  if ((msg->bus == 0U) && (msg->extended == 0U)) {
    uint32_t now = microsecond_timer_get();

    if (msg->addr == HCA_HEARTBEAT_HCA_01) {
      hca_heartbeat_foreign = true;
    }
    if ((msg->addr == MSG_LH_EPS_03) && (GET_LEN(msg) == 8U) && (volkswagen_mqb_meb_compute_crc(msg) == msg->data[0])) {
      hca_heartbeat_eps_ts = now;
      hca_heartbeat_eps_awake = true;
    }

    // bus 0 traffic paces the sends
    if (hca_heartbeat_should_send(now) && (get_ts_elapsed(now, hca_heartbeat_tx_ts) >= HCA_HEARTBEAT_PERIOD_US)) {
      hca_heartbeat_tx_ts = now;
      hca_heartbeat_send();
    }
  }
}

// every frame the host sends
void hca_heartbeat_tx_hook(const CANPacket_t *msg, uint8_t bus_number, bool allowed) {
  if (allowed && (bus_number == 0U) && (msg->addr == HCA_HEARTBEAT_HCA_01)) {
    hca_heartbeat_host_ts = microsecond_timer_get();
    hca_heartbeat_host_seen = true;
  }
}

// 8Hz
void hca_heartbeat_tick(void) {
  hca_heartbeat_eps_awake = hca_heartbeat_eps_awake &&
                            (get_ts_elapsed(microsecond_timer_get(), hca_heartbeat_eps_ts) < HCA_HEARTBEAT_EPS_TIMEOUT_US);

  // SILENT puts the CAN cores in bus monitoring mode, which can't transmit at all, so bus 0
  // leaves it while the EPS is awake (see can_set_speed). Host frames are still blocked by the SILENT tx hook.
  bool bus0_silent = can_silent && !hca_heartbeat_eps_awake;
  if (bus0_silent != hca_heartbeat_bus0_silent) {
    hca_heartbeat_bus0_silent = bus0_silent;
    (void)can_init(CAN_NUM_FROM_BUS_NUM(0U));
  }
}
