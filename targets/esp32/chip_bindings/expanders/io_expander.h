#pragma once

// The I/O expander a board routes slow control lines through: a backlight
// enable, a touch or panel reset, a card-select. Bindings that need one of
// those lines (rgb_panel.cpp, gt911.cpp) ask board.h which expander pin carries
// it and drive it through this interface; a board with no expander compiles
// them with GEA_BOARD_HAS_EXPANDER=0 and the calls fall away.
//
// Each board target links exactly one expander binding (ch422g, …) that
// provides these symbols. Pin numbers are the expander's own (0-based), not
// MCU GPIOs.
namespace gea::platform::esp32::chip_bindings::expanders
{

  class IoExpander
  {
  public:
    virtual ~IoExpander() = default;
    // Idempotent: the first caller brings the chip up on the primary I2C bus;
    // later callers see the cached result.
    virtual bool init() = 0;
    virtual bool writePin(int pin, bool high) = 0;
    virtual bool readPin(int pin, bool &high) = 0;
  };

  IoExpander &ioExpander();
  const char *ioExpanderDriverName();

} // namespace gea::platform::esp32::chip_bindings::expanders
