#pragma once

#include <cstdint>

// A board's physical buttons, delivered to the app as key events whose `code`
// names the button (see input.h, registerButton). They are declared as JSON,
// never here: the board's "buttons" (its targets.json entry, composed
// definition or boards.json alias) plus the app's gea.targets.esp32.buttons.
// The CLI generates gea_board_buttons.h from that list (boards/buttons.mjs).
//
// GPIO buttons are interrupt-driven: an edge wakes one shared task, which
// debounces and reports the settled level. A button polls only when its entry
// says "poll": true. The PMU's power key belongs to the PMU binding, which
// reports it through pmuButton().
namespace gea::platform::esp32::buttons {

enum class Source : std::uint8_t {
	Gpio,
	// The power key of the board's PMU (AXP2101 PWRON).
	Pmu,
};

struct Button {
	// The silkscreen name, PascalCase: "Boot", "Power", "Key1".
	const char *code;
	// The key the button stands in for when an app does not ask for it by
	// name, and its web keyCode. "" and 0 for a button with no alias.
	const char *key = "";
	int keyCode = 0;
	Source source = Source::Gpio;
	int pin = -1;
	// The level the pin reads while the button is held. Active-low buttons
	// get the internal pull-up, active-high ones the pull-down.
	int activeLevel = 0;
	bool poll = false;
};

// Registers the board's buttons and starts watching its GPIO buttons. A no-op
// on a board that declares none; safe to call more than once.
void start();

// The id (for input::queueButton) of the board's PMU power key, registering
// the table first if start() has not run yet; -1 when the board has none.
int pmuButton();

}  // namespace gea::platform::esp32::buttons
