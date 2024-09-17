
#include <tmx_cpp/modules/HiwonderServo.hpp>

using namespace tmx_cpp;

HiwonderServo_module::HiwonderServo_module(
    uint8_t uart_port, uint8_t rx_pin, uint8_t tx_pin,
    std::vector<uint8_t> servo_ids,
    std::function<void(std::vector<Servo_pos>)> position_cb,
    std::function<void(int, bool)> verify_cb,
    std::function<void(int, uint16_t, uint16_t)> range_cb,
    std::function<void(int, uint16_t)> offset_cb) {

  this->servo_ids = servo_ids;
  this->uart_port = uart_port;
  this->rx_pin = rx_pin;
  this->tx_pin = tx_pin;
  this->position_cb = position_cb;
  this->verify_cb = verify_cb;
  this->range_cb = range_cb;
  this->offset_cb = offset_cb;
  type = MODULE_TYPE::HIWONDER_SERVO;
}

bool HiwonderServo_module::set_single_servo(uint8_t servo_id, uint16_t angle,
                                            uint16_t time) {
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::SET_SERVO,
                               1,
                               get_servo_num(servo_id),
                               (uint8_t)(angle >> 8),
                               (uint8_t)(angle & 0xFF),

                               (uint8_t)(time >> 8),
                               (uint8_t)(time & 0xFF)};
  this->send_module(data);
  return true;
}

bool HiwonderServo_module::set_multiple_servos(
    std::vector<std::pair<uint8_t, uint16_t>> servo_vals, uint16_t time) {
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::SET_SERVO,
                               (uint8_t)servo_vals.size()};
  for (auto servo_val : servo_vals) {
    data.push_back(get_servo_num(servo_val.first));
    data.push_back((uint8_t)(servo_val.second & 0xFF));
    data.push_back((uint8_t)(servo_val.second >> 8));

    data.push_back((uint8_t)(time & 0xFF));
    data.push_back((uint8_t)(time >> 8));
  }
  this->send_module(data);
  return true;
}

bool HiwonderServo_module::set_enable_servo(uint8_t servo_id, bool enable) {
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::SET_ENABLE, 1,
                               get_servo_num(servo_id), enable};
  this->send_module(data);
  return true;
}

bool HiwonderServo_module::set_enabled_all(bool enable) {
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::SET_ENABLE, 0, enable};
  this->send_module(data);
  return true;
}

bool HiwonderServo_module::set_id(uint8_t new_id, uint8_t old_id) {
  if (old_id != 0xFF) {
    // TODO: implement this
    std::cout << "Renaming single servo not yet implemented" << std::endl;
  }
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::SET_ID, new_id};
  this->send_module(data);
  return true;
}

bool HiwonderServo_module::verify_id(uint8_t id) {
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::VERIFY_ID,
                               get_servo_num(id)};
  this->send_module(data);
  return true;
}

bool HiwonderServo_module::set_range(uint8_t servo_id, uint16_t min,
                                     uint16_t max) {
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::SET_RANGE,
                               get_servo_num(servo_id),
                               (uint8_t)(min & 0xFF),
                               (uint8_t)(min >> 8),
                               (uint8_t)(max & 0xFF),
                               (uint8_t)(max >> 8)};

  this->send_module(data);
  return true;
}

bool HiwonderServo_module::set_voltage_range(uint8_t servo_id, float minf,
                                             float maxf) {
  uint16_t min = (uint16_t)(minf * 1000); // convert to mV
  uint16_t max = (uint16_t)(maxf * 1000);
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::SET_VOLTAGE_RANGE,
                               get_servo_num(servo_id),
                               (uint8_t)(min & 0xFF),
                               (uint8_t)(min >> 8),
                               (uint8_t)(max & 0xFF),
                               (uint8_t)(max >> 8)};

  this->send_module(data);
  return true;
}

bool HiwonderServo_module::set_offset(uint8_t servo_id, uint16_t offset) {
  std::vector<uint8_t> data = {
      HIWONDER_SERVO_COMMANDS::SET_OFFSET, get_servo_num(servo_id),
      (uint8_t)(offset & 0xFF), (uint8_t)(offset >> 8)};

  this->send_module(data);
  return true;
}

bool HiwonderServo_module::get_range(uint8_t servo_id) {
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::GET_RANGE,
                               get_servo_num(servo_id)};
  this->send_module(data);
  return true;
}

bool HiwonderServo_module::get_offset(uint8_t servo_id) {
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::GET_OFFSET,
                               get_servo_num(servo_id)};
  this->send_module(data);
  return true;
}

uint8_t HiwonderServo_module::get_servo_num(uint8_t servo_id) {
  // the servo_ids vector is the list of servo ids that the user has given
  // but the comm uses the index of the servo in the list

  for (size_t i = 0; i < servo_ids.size(); i++) {
    if (servo_ids[i] == servo_id) {
      return i;
    }
  }
  return 0xFF;
}

std::vector<uint8_t> HiwonderServo_module::init_data() {
  std::vector<uint8_t> data = {this->uart_port, this->rx_pin, this->tx_pin,
                               (uint8_t)servo_ids.size()};
  for (auto servo_id : servo_ids) {
    data.push_back(servo_id);
  }
  return data;
}

void HiwonderServo_module::data_callback(std::vector<uint8_t> data) {

  auto message_type = (HIWONDER_SERVO_RESPONSES)data[0];
  switch (message_type) {
  case HIWONDER_SERVO_RESPONSES::SERVO_POSITION: {
    std::vector<Servo_pos> servo_positions;
    for (size_t i = 1; i < data.size(); i += 3) {
      Servo_pos pos;
      pos.id = servo_ids[data[i]];
      pos.angle = data[i + 2] + (data[i + 1] << 8);
      servo_positions.push_back(pos);
    }
    position_cb(servo_positions);
    return;
  } break;
  case HIWONDER_SERVO_RESPONSES::SERVO_VERIFY: {
    verify_cb(data[1], data[2]);
    return;
  } break;
  case HIWONDER_SERVO_RESPONSES::SERVO_RANGE: { // TODO: wss kloppen deze ook
                                                // niet...
    range_cb(data[1], data[2] + (data[3] << 8), data[4] + (data[5] << 8));
    return;
  } break;
  case HIWONDER_SERVO_RESPONSES::SERVO_OFFSET: {
    offset_cb(data[1], data[2] + (data[3] << 8));
    return;
  } break;

  default:
    std::cout << "Unknown message type from Hiwonder servo: " << std::dec
              << (int)message_type << std::endl;
    break;
  }
  return;
}

void HiwonderServo_module::attach_send_module(
    std::function<void(std::vector<uint8_t>)> send_module) {
  this->send_module = send_module;
}