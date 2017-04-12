#pragma once

namespace gpio {

/**
 * @brief      Class for gpio pin.
 *
 * Precomputes the pin number and port number at construction-time.
 */
class Pin {
public:
    const uint8_t pin;        //! The board pin number
    const uint8_t port;       //! The microprocessor port number
    const uint8_t mask;       //! A mask selecting which bit of the port to use
    const uint16_t port_mask; //! Shorthand for 1 << port

    Pin(uint8_t pin)
        : Pin(pin, digitalPinToPort(pin), digitalPinToBitMask(pin)) { }

    operator uint8_t() {
        return pin;
    }
private:
    constexpr Pin(uint8_t pin, uint8_t port, uint8_t mask)
        : pin(pin), port(port), mask(mask), port_mask(1 << port) { }
};

class CachedReader {
    static const size_t N = 10;  // number of ports on the board
    bool _read_mask = {0};
    uint8_t _vals[N];
public:
    inline int read(Pin pin) {
        if (!(_read_mask & pin.port_mask)) {
            _vals[pin.port] = portRegisters(pin.port)->port.reg;
            _read_mask |= pin.port_mask;
        }
        return _vals[pin.port] & pin.mask;
    }
};

}