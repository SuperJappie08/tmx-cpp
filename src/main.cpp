
#include "AsyncSerial.h"
#include "buffer_build.hpp"
#include "tmx.hpp"
#include <iostream>
#include <thread>
using namespace std;

int main(int argc, char *argv[]) {
  try {
    TMX tmx;
    CallbackAsyncSerial serial(argv[1], 115200);
    tmx.serial = &serial;
    int count = 0;
    // byte_buffer bytes;
    // append_bytes(bytes, 1);         // appends sizeof(int) bytes
    // append_bytes(bytes, 1ul);       // appends sizeof(unsigned long) bytes
    // append_bytes(bytes, 'a');       // appends sizeof(int) bytes :p
    // append_bytes(bytes, char('a')); // appends 1 byte
    auto t = append_buffer((uint8_t)1, 2, 3, 4, 5, 6, 7, 8, 9, 10);
    std::cout << "t = ";
    for (auto i : t) {
      std::cout << std::hex << (int)i << " ";
    }
    std::cout << std::endl;
    serial.setCallback([&tmx, &count](const char *data, size_t len) {
      count += len;
      //    std::cout << "callback!!!" << count << std::endl;
      //    this_thread::sleep_for(chrono::milliseconds(1000));

      tmx.callback(data, len);
      //   std::cout << "done cb" << std::endl;
    });

    tmx.ping_callbacks.push_back(
        [](auto message) { std::cout << "ping" << std::endl; });
    // Simulate doing something else while the serial device replies.
    // When the serial device replies, the second thread stores the received
    // data in a buffer.
    this_thread::sleep_for(chrono::seconds(2));
    std::vector<uint8_t> message = {TMX::FIRMWARE_VERSION};
    tmx.sendMessage(message);
    tmx.sendMessage(TMX::MESSAGE_TYPE::GET_PICO_UNIQUE_ID, message);
    this_thread::sleep_for(chrono::milliseconds(100));
    auto pin = 25;
    auto sl = 100;
    tmx.setPinMode(pin, TMX::PIN_MODES::DIGITAL_OUTPUT);
    for (int i = 0; i < 100; i++) {
      tmx.digitalWrite(pin, 1);
      this_thread::sleep_for(chrono::milliseconds(sl));
      tmx.digitalWrite(pin, 0);
      this_thread::sleep_for(chrono::milliseconds(sl));
    }

    this_thread::sleep_for(chrono::seconds(20));

    // Always returns immediately. If the terminator \r\n has not yet
    // arrived, returns an empty string.
    //  cout<<serial.readStringUntil("\r\n")<<endl;

    serial.close();
  } catch (boost::system::system_error &e) {
    cout << "Error: " << e.what() << endl;
    return 1;
  }
}
