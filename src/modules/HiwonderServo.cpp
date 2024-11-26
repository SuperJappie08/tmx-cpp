#include <chrono>
#include <ranges>

#include <tmx_cpp/modules/HiwonderServo.hpp>
#include <tmx_cpp/serialization.hpp>

using namespace tmx_cpp;
using namespace std::chrono_literals;

HiwonderServo_module::HiwonderServo_module(
    uint8_t uart_port, uint8_t rx_pin, uint8_t tx_pin, std::vector<uint8_t> servo_ids,
    std::function<void(std::vector<std::tuple<uint8_t, Servo_pos>>)> position_cb) {

  this->servo_ids = servo_ids;
  this->uart_port = uart_port;
  this->rx_pin = rx_pin;
  this->tx_pin = tx_pin;
  // TODO: Use optional futures to determine if callback of normal
  this->position_cb = position_cb;
  type = MODULE_TYPE::HIWONDER_SERVO;
}

bool HiwonderServo_module::set_single_servo(uint8_t servo_id, uint16_t angle, uint16_t time) {
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::SET_SERVO, 1, get_servo_num(servo_id)}; //,
  data.reserve(data.size() + 2 * sizeof(uint16_t));

  append_range(data, encode_u16(angle));
  append_range(data, encode_u16(time));
  //  (uint8_t)(angle >> 8),
  //  (uint8_t)(angle & 0xFF),

  //  (uint8_t)(time >> 8),
  //  (uint8_t)(time & 0xFF)};
  this->send_module(data);
  return true;
}

bool HiwonderServo_module::set_multiple_servos(std::vector<std::pair<uint8_t, uint16_t>> servo_vals,
                                               uint16_t time) {
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::SET_SERVO, (uint8_t)servo_vals.size()};
  data.reserve(data.size() + servo_vals.size() * (2 * sizeof(uint16_t) + sizeof(uint8_t)));

  auto time_bytes = encode_u16(time);
  for (auto servo_val : servo_vals) {
    data.push_back(get_servo_num(servo_val.first));
    // FIXME: Do these need to be flipped????
    append_range(data, encode_u16(servo_val.second));
    // data.push_back((uint8_t)(servo_val.second & 0xFF));
    // data.push_back((uint8_t)(servo_val.second >> 8));

    // data.push_back((uint8_t)(time & 0xFF));
    // data.push_back((uint8_t)(time >> 8));
    append_range(data, time_bytes);
  }
  this->send_module(data);
  return true;
}

bool HiwonderServo_module::set_enable_servo(uint8_t servo_id, bool enable) {
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::SET_ENABLE, 1, get_servo_num(servo_id),
                               enable};
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

bool HiwonderServo_module::verify_id(uint8_t servo_id) {
  assert(!verify_id_promise.has_value());
  verify_id_promise = std::promise<std::tuple<uint8_t, bool>>();

  auto future = verify_id_promise->get_future();
  // Do not send a servo index, but an actuall ID.
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::VERIFY_ID, servo_id};
  this->send_module(data);

  auto ok = false;

  // TODO: Wait probably toolong
  if (future.wait_for(100ms) == std::future_status::ready) {
    auto [id, ok_value] = future.get();
    assert(id == servo_id);
    ok = ok_value;
  }

  verify_id_promise.reset();

  return ok;
}

bool HiwonderServo_module::set_range(uint8_t servo_id, uint16_t min, uint16_t max) {
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::SET_RANGE, get_servo_num(servo_id)};
  // data.reserve(data.size() + 2 * sizeof(uint16_t));

  auto min_bytes = encode_u16(min);
  auto max_bytes = encode_u16(max);

  data.insert(data.end(), min_bytes.begin(), min_bytes.end());
  data.insert(data.end(), max_bytes.begin(), max_bytes.end());

  this->send_module(data);
  return true;
}

bool HiwonderServo_module::set_voltage_range(uint8_t servo_id, float minf, float maxf) {
  uint16_t min = (uint16_t)(minf * 1000); // convert to mV
  uint16_t max = (uint16_t)(maxf * 1000);
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::SET_VOLTAGE_RANGE, get_servo_num(servo_id)};
  // data.reserve(data.size() + 2*sizeof(uint16_t));

  append_range(data, encode_u16(min));
  append_range(data, encode_u16(max));

  this->send_module(data);
  return true;
}

bool HiwonderServo_module::set_offset(uint8_t servo_id, uint16_t offset) {
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::SET_OFFSET, get_servo_num(servo_id)}; //,
  // data.reserve(data.size() + sizeof(uint16_t));
  append_range(data, encode_u16(offset));

  this->send_module(data);
  return true;
}

std::optional<std::tuple<uint16_t, uint16_t>> HiwonderServo_module::get_range(uint8_t servo_id) {
  assert(!range_promise.has_value());
  range_promise = std::promise<std::tuple<uint8_t, std::tuple<uint16_t, uint16_t>>>();

  auto servo_num = get_servo_num(servo_id);
  auto future = range_promise->get_future();
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::GET_RANGE, servo_num};
  this->send_module(data);

  std::optional<std::tuple<uint16_t, uint16_t>> range;

  // TODO: Wait probably toolong
  if (future.wait_for(100ms) == std::future_status::ready) {
    auto [id, range_value] = future.get();
    assert(id == servo_num);
    range = range_value;
  }

  range_promise.reset();

  return range;
}

std::optional<uint16_t> HiwonderServo_module::get_offset(uint8_t servo_id) {
  assert(!offset_promise.has_value());
  offset_promise = std::promise<std::tuple<uint8_t, uint16_t>>();

  auto servo_num = get_servo_num(servo_id);
  auto future = offset_promise->get_future();
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::GET_OFFSET, servo_num};
  this->send_module(data);

  std::optional<uint16_t> offset;

  // TODO: Wait probably toolong
  if (future.wait_for(100ms) == std::future_status::ready) {
    auto [id, offset_value] = future.get();

    assert(id == servo_num);
    offset = offset_value;
  }

  offset_promise.reset();

  return offset;
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
  auto data_span = std::span(data);
  auto message_type = (HIWONDER_SERVO_RESPONSES)data[0];
  switch (message_type) {
  case HIWONDER_SERVO_RESPONSES::SERVO_POSITION: {
    std::vector<std::tuple<uint8_t, Servo_pos>> servo_positions;
    for (size_t i = 1; i < data.size(); i += 3) {
      Servo_pos pos;
      pos.id = servo_ids[data[i]];
      pos.angle = decode_u16(data_span.subspan(i + 1).first<2>());
      servo_positions.push_back({data[i], pos});
    }
    position_cb(servo_positions);
    return;
  } break;
  case HIWONDER_SERVO_RESPONSES::SERVO_VERIFY: {
    assert(verify_id_promise.has_value()); // TODO: REPLACE WITH WARNING INSTEAD OF EXIT -1
    verify_id_promise->set_value({(uint8_t)data[1], (bool)data[2]});
    return;
  } break;
  case HIWONDER_SERVO_RESPONSES::SERVO_RANGE: {
    assert(range_promise.has_value()); // TODO: REPLACE WITH WARNING INSTEAD OF EXIT -1
    range_promise->set_value({(uint8_t)data[1],
                              {
                                  decode_u16(data_span.subspan<2, sizeof(uint16_t)>()),
                                  decode_u16(data_span.subspan<4, sizeof(uint16_t)>()),
                              }});
    return;
  } break;
  case HIWONDER_SERVO_RESPONSES::SERVO_OFFSET: {
    assert(offset_promise.has_value()); // TODO: REPLACE WITH WARNING INSTEAD OF EXIT -1
    offset_promise->set_value(
        {(uint8_t)data[1], decode_u16(data_span.subspan<2, sizeof(uint16_t)>())});
    return;
  } break;

  default:
    std::cout << "Unknown message type from Hiwonder servo: " << std::dec << (int)message_type
              << std::endl;
    break;
  }
  return;
}

void HiwonderServo_module::attach_send_module(
    std::function<void(std::vector<uint8_t>)> send_module) {
  this->send_module = send_module;
}

bool HiwonderServo_module::motor_mode_write(uint8_t servo_id, int16_t speed) {
  std::vector<uint8_t> data = {HIWONDER_SERVO_COMMANDS::MOTOR_MODE_WRITE, get_servo_num(servo_id)};
  append_range(data, encode_i16(speed));
  this->send_module(data);
  return true;
}