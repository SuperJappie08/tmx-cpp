#include <functional>
#include <vector>
#include <stdint.h>

class PCA9685_module {
        public:
        std::function<void(std::vector<uint8_t>)> send_module;
        PCA9685_module(std::function<void(std::vector<uint8_t>)> send_module);
        bool set_pwm(uint8_t channel, uint16_t high, uint16_t low=0);
        struct PWM_val {
            uint8_t channel;
            uint16_t high;
            uint16_t low = 0;
        };
        bool set_multiple_pwm(std::vector<PWM_val> pwm_vals);
};