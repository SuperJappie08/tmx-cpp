#include <tmx_cpp/modules.hpp>

Modules::Modules(std::shared_ptr<TMX> tmx) {
  this->tmx = tmx;
  tmx->add_callback(TMX::MESSAGE_IN_TYPE::MODULE_REPORT,
                    std::bind(&Modules::callback, this, std::placeholders::_1));
}

int Modules::add_module(uint8_t mod_num, MODULE_TYPE type,
                        std::vector<uint8_t> data,
                        std::function<void(std::vector<uint8_t>)> callback) {
  modules.push_back(std::make_pair(type, callback));
  std::vector<uint8_t> addModMsg(data.begin(), data.end());
  addModMsg.insert(addModMsg.begin(), {mod_num, (uint8_t)type});

  tmx->sendMessage(TMX::MESSAGE_TYPE::MODULE_NEW, addModMsg);
  return modules.size() - 1;
}

void Modules::add_mod(std::shared_ptr<Module_type> module) {
  auto mod_num = this->modules.size();
  auto init_data = module->init_data();

  auto act_mod_num = add_module(
      mod_num, module->type, init_data,
      std::bind(&Module_type::data_callback, module, std::placeholders::_1));
  module->attach_send_module([this, act_mod_num](std::vector<uint8_t> data) {
    this->send_module(act_mod_num, data);
  });
  if (act_mod_num != mod_num) {
    std::cout << "Error adding module" << std::endl;
    return;
  }
}

void Modules::callback(std::vector<uint8_t> data) {

  uint8_t module_num = data[2];
  std::vector<uint8_t> module_data(data.begin() + 4, data.end());
  this->modules[module_num].second(module_data);
}

bool Modules::send_module(uint8_t module_num, std::vector<uint8_t> data) {
  if (module_num >= modules.size() || module_num < 0) {
    return false;
  }
  std::vector<uint8_t> addModMsg(data.begin(), data.end());
  addModMsg.insert(addModMsg.begin(), {(uint8_t)module_num});

  tmx->sendMessage(TMX::MESSAGE_TYPE::MODULE_DATA, addModMsg);
  return true;
}

void empty_callback(std::vector<uint8_t> data){};