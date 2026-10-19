// The frame-loop services gea expects from a platform, for a single-threaded
// bare-metal board: the event queue is a plain ring drained by the main loop,
// and every allocation comes from the SDRAM heap.

#include "app.h"
#include "clock.h"
#include "event.h"
#include "memory.h"
#include "services/frame_scheduler.h"
#include "ui/document.h"
#include "ui/style.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <malloc.h>
#include <unistd.h>

extern "C" char __heap_end[];

namespace {

using gea::framework::services::FrameScheduler;

std::array<gea::framework::events::Event, FrameScheduler::kEventQueueDepth> gEvents{};
int gEventHead = 0;
int gEventTail = 0;
bool gEventQueueStarted = false;
int gFrameIntervalMs = FrameScheduler::kDefaultFrameIntervalUs / 1000;
bool gVsyncDriven = false;

int nextEventIndex(int index) {
	return (index + 1) % static_cast<int>(gEvents.size());
}

// Heap still unclaimed from the SDRAM region plus what malloc holds free.
std::uint32_t heapFreeBytes() {
	const auto *brk = static_cast<const char *>(sbrk(0));
	return static_cast<std::uint32_t>((__heap_end - brk) + mallinfo().fordblks);
}

}  // namespace

namespace gea::framework::services {

EventQueue FrameScheduler::createEventQueue() {
	gEventQueueStarted = true;
	return EventQueue(&gEvents);
}

EventQueue FrameScheduler::eventQueue() { return gEventQueueStarted ? EventQueue(&gEvents) : EventQueue(); }

bool FrameScheduler::sendEvent(const gea::framework::events::Event &event, int) {
	const int next = nextEventIndex(gEventTail);
	if (next == gEventHead) return false;

	gEvents[static_cast<std::size_t>(gEventTail)] = event;
	gEventTail = next;
	return true;
}

bool FrameScheduler::receiveEvent(gea::framework::events::Event *event) {
	if (gEventHead == gEventTail) return false;

	if (event) *event = gEvents[static_cast<std::size_t>(gEventHead)];
	gEventHead = nextEventIndex(gEventHead);
	return true;
}

void FrameScheduler::start(EventQueue queue) { gEventQueueStarted = static_cast<bool>(queue); }

void FrameScheduler::runFrame(const FrameCallbacks &callbacks) {
	if (callbacks.frame) callbacks.frame(nowMs(), callbacks.context);
}

bool FrameScheduler::takeCatchUpRequest() { return false; }
bool FrameScheduler::drainFramesAndCheckInput() { return gEventHead != gEventTail; }
void FrameScheduler::setVsyncDriven(bool driven) { gVsyncDriven = driven; }
bool FrameScheduler::vsyncDriven() { return gVsyncDriven; }
void FrameScheduler::notifyVsyncFromISR() {}

void FrameScheduler::setFrameIntervalMs(int intervalMs) { gFrameIntervalMs = std::clamp(intervalMs, kMinFrameIntervalMs, kMaxFrameIntervalMs); }

int FrameScheduler::frameIntervalMs() { return gFrameIntervalMs; }

void FrameScheduler::setFrameRate(double fps) {
	if (fps > 0.0) setFrameIntervalMs(static_cast<int>(1000.0 / fps + 0.5));
}

double FrameScheduler::frameRate() { return 1000.0 / static_cast<double>(gFrameIntervalMs); }

int FrameScheduler::nowMs() { return static_cast<int>(gea::stm32::clock::micros() / 1000); }

}  // namespace gea::framework::services

namespace gea::framework::app {

void Application::init(int width, int height, double devicePixelRatio) {
	gea::embedded::ui::Document::setPreferredMountSize(width, height);
	gea::embedded::ui::setViewportMetrics(width, height, devicePixelRatio);
}

}  // namespace gea::framework::app

namespace gea::framework::memory {

void *Allocator::allocatePreferSpiram(std::size_t size, std::size_t alignment) { return memalign(alignment, size); }

void *Allocator::reallocatePreferSpiram(void *ptr, std::size_t size) { return std::realloc(ptr, size); }

void Allocator::free(void *ptr) noexcept { std::free(ptr); }

}  // namespace gea::framework::memory

// One heap, in SDRAM: it reports as "PSRAM" (the external pool) and the
// internal-RAM figures stay zero, since nothing allocates from internal RAM.
namespace gea::platform::memory {

std::uint32_t Memory::internalFree() { return 0; }
std::uint32_t Memory::internalLargestFreeBlock() { return 0; }
std::uint32_t Memory::internalMinimumFree() { return 0; }
std::uint32_t Memory::psramFree() { return heapFreeBytes(); }
std::uint32_t Memory::currentTaskStackHighWaterMark() { return 0; }
std::uint32_t Memory::geaMainStackBytes() { return 0; }
std::uint32_t Memory::geaInitStackBytes() { return 0; }
std::uint32_t Memory::appFrameStackWords() { return 0; }
std::uint32_t Memory::appFrameStackBytes() { return 0; }
std::uint32_t Memory::displayFlushConfiguredRows() { return 0; }
std::uint32_t Memory::displayFlushConfiguredDepth() { return 0; }
std::uint32_t Memory::displayFlushBufferMaxBytes() { return 0; }
std::uint32_t Memory::displayFlushRows() { return 0; }
std::uint32_t Memory::displayFlushDepth() { return 0; }
std::uint32_t Memory::displayFlushBufferBytes() { return 0; }
void *Memory::reserveInternalDma(std::size_t) { return nullptr; }
void Memory::releaseInternalDma(void *) {}

}  // namespace gea::platform::memory
