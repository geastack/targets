// App image entry: bring the board up, mount the generated app, then run the
// frame loop (input, microtasks, timers and animation frames, CSS animations,
// layout and paint) paced to the scheduler's frame interval.

#include "app.h"
#include "board.h"
#include "clock.h"
#include "console.h"
#include "css/declarative.h"
#include "css/engine.h"
#include "dev_control.h"
#include "display.h"
#include "event.h"
#include "events.h"
#include "frame_rate.h"
#include "host/timers.h"
#include "input.h"
#include "rtc_clock.h"
#include "services/frame_scheduler.h"
#include "system.h"
#include "ui/document.h"
#include "ui/style.h"
#include "ui/tree_internal.h"
#include "user_button.h"

#include <cstdint>
#include <cstdio>

extern void __gea_top_level();
void gea_plugin_cpp_register_prelude() __attribute__((weak));

namespace gea::framework::app::generated {
void drainMicrotasks();
}  // namespace gea::framework::app::generated

using namespace gea::stm32;
using gea::framework::services::FrameScheduler;

namespace {

namespace events = gea::framework::events;
namespace generated = gea::framework::app::generated;
namespace input = gea::framework::input;
namespace ui = gea::embedded::ui;

// Escape is the conventional "back" key; the USER button sends it and also
// raises the back-button flag that Input.consumeBackButton() reads.
constexpr int kUserButtonKeyCode = 27;

void dispatchQueuedEvents() {
	events::Event event{};
	while (FrameScheduler::receiveEvent(&event)) {
		if (event.type == events::EventType::Touch) events::TouchRuntime::dispatchEvent(event);
	}
}

void pollUserButton(std::uint64_t nowUs) {
	if (!user_button::pollPressed(nowUs)) return;

	input::pressBackButton();
	input::queueKeyDown(kUserButtonKeyCode);
}

// Queued keys go to the focused input first, then to document listeners.
void dispatchKeyInput() {
	auto &tree = ui::Tree::instance();
	for (int keyCode = input::consumeKeyDown(); keyCode != 0; keyCode = input::consumeKeyDown()) {
		events::PointerEvent event{};
		event.type = events::PointerEventType::KeyDown;
		event.keyCode = keyCode;
		event.bubbles = true;
		event.cancelable = true;

		const int inputId = tree.activeInputId();
		if (inputId >= 0) {
			event.targetId = inputId;
			event.currentTargetId = inputId;
			tree.dispatchEvent(event);
		}

		if (!event.propagationStopped) ui::dispatchDocumentKeyDown(event);
	}
}

// Declarative animations start once, on the first frame. CSS animations of
// nodes mounted later are only primed by the engine, so every frame starts
// the new ones, as the shared runtime does; running timelines are kept.
void startCssAnimations(std::uint32_t nowMs) {
	static bool declarativeStarted = false;
	if (!declarativeStarted) {
		gea::css::DeclarativeAnimations::scanAndStart(nowMs);
		declarativeStarted = true;
	}

	ui::StyleSheet::instance().startCssAnimations(nowMs);
}

// The same phase order as the other embedded targets. The style batch
// coalesces the class recomputes this frame's reactive updates cause into one
// pass before layout and paint.
void runFrame(int timestampMs) {
	const auto nowMs = static_cast<std::uint32_t>(timestampMs);
	startCssAnimations(nowMs);

	events::TouchRuntime::poll(timestampMs);
	dispatchQueuedEvents();
	dispatchKeyInput();

	ui::beginStyleMountBatch();
	generated::drainMicrotasks();
	gea::host::runAnimationFrameCallbacks(static_cast<double>(timestampMs));
	gea::css::AnimationEngine::instance().tick(nowMs);
	dispatchQueuedEvents();
	ui::endStyleMountBatch();

	ui::Document::instance().frame(timestampMs);
	generated::drainMicrotasks();
	ui::Document::instance().refreshMountedIfDirty();
}

// The first mount ran inside an open style batch, so it skipped layout and
// paint; mounting again gives the first frame its final styles.
void mountApp() {
	ui::beginStyleMountBatch();
	__gea_top_level();
	ui::endStyleMountBatch();
	generated::drainMicrotasks();

	auto &tree = ui::Tree::instance();
	const int root = tree.mountedRoot();
	if (root >= 0) tree.mount(root, tree.mountedWidth(), tree.mountedHeight());

	ui::Document::instance().refreshMountedIfDirty();
}

void reportBoot(bool rtcReady) {
	const BootInfo &boot = bootInfo();
	std::printf("gea.stm32 boot clock=%d sdram=%d qspi=%d jedec=%06lx rtc=%d time_set=%d\n", boot.clockReady, boot.sdramReady, boot.qspiReady,
	            static_cast<unsigned long>(boot.qspiJedecId), rtcReady, rtc_clock::isSet());
}

void waitForNextFrame(std::uint64_t frameStartUs) {
	const std::uint64_t budgetUs = static_cast<std::uint64_t>(FrameScheduler::frameIntervalMs()) * 1000;
	const std::uint64_t elapsedUs = clock::micros() - frameStartUs;
	if (elapsedUs < budgetUs) clock::delayMicros(static_cast<std::uint32_t>(budgetUs - elapsedUs));
}

}  // namespace

int main() {
	clock::init();
	console::init(board::kConsoleBaud);
	const bool rtcReady = rtc_clock::init();
	user_button::init();
	reportBoot(rtcReady);

	if (gea_plugin_cpp_register_prelude) gea_plugin_cpp_register_prelude();

	gea::platform::display::Display::init();
	gea::framework::app::Application::init(gea::platform::display::kWidth, gea::platform::display::kHeight,
	                                       GEA_EMBEDDED_CSS_LAYOUT_DEVICE_PIXEL_RATIO);
	FrameScheduler::start(FrameScheduler::createEventQueue());
	const bool touchReady = events::TouchRuntime::start();
	std::printf("gea.stm32 touch=%d launching %s\n", touchReady, GEA_STM32_APP_ID);

	mountApp();

	for (;;) {
		const std::uint64_t frameStartUs = clock::micros();
		pollUserButton(frameStartUs);
		dev_control::poll(frameStartUs);
		runFrame(static_cast<int>(frameStartUs / 1000));
		frame_rate::tick(frameStartUs);
		waitForNextFrame(frameStartUs);
	}
}
