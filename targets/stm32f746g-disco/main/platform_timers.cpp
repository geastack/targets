// setTimeout, setInterval and requestAnimationFrame for a single-threaded
// board: all of them run from the frame loop, with no worker threads.

#include "host/timers.h"

#include <algorithm>
#include <utility>
#include <vector>

namespace gea::framework::app::generated {
void drainMicrotasks();
}  // namespace gea::framework::app::generated

namespace {

struct Timer {
	double id = 0;
	gea::host::TimerCallback callback;
	double delayMs = 0;
	double nextFireMs = 0;
	bool interval = false;
	bool active = true;
};

std::vector<Timer> gTimers;
std::vector<gea::host::AnimationFrameCallback> gPendingFrames;
std::vector<gea::host::AnimationFrameCallback> gRunningFrames;
double gNextId = 1;
double gLastTimestampMs = 0;

double addTimer(gea::host::TimerCallback callback, double delayMs, bool interval) {
	if (!callback) return 0;

	const double id = gNextId++;
	gTimers.push_back({id, std::move(callback), delayMs, gLastTimestampMs + delayMs, interval});
	return id;
}

// Callbacks may add or clear timers; indices stay valid while the vector
// grows, and cleared timers are dropped only after the pass. Each callback is
// a task: its microtasks (reactive re-renders) settle before the next one and
// before this frame's animation callbacks, which may look up what it mounted.
void runDueTimers(double nowMs) {
	const std::size_t count = gTimers.size();
	for (std::size_t i = 0; i < count; ++i) {
		if (!gTimers[i].active || gTimers[i].nextFireMs > nowMs) continue;

		gea::host::TimerCallback callback = gTimers[i].callback;
		if (gTimers[i].interval) {
			gTimers[i].nextFireMs = nowMs + gTimers[i].delayMs;
		} else {
			gTimers[i].active = false;
		}
		callback();
		gea::framework::app::generated::drainMicrotasks();
	}
	std::erase_if(gTimers, [](const Timer &timer) { return !timer.active; });
}

void runFrameCallbacks(gea::host::AnimationFrameTimestamp timestampMs) {
	gRunningFrames.swap(gPendingFrames);
	for (auto &callback : gRunningFrames) callback(timestampMs);
	gRunningFrames.clear();
}

}  // namespace

namespace gea::host {

double setTimeout(TimerCallback callback, double delayMs) { return addTimer(std::move(callback), std::max(0.0, delayMs), false); }

double setInterval(TimerCallback callback, double delayMs) { return addTimer(std::move(callback), std::max(1.0, delayMs), true); }

void clearTimeout(double id) {
	for (Timer &timer : gTimers) {
		if (timer.id == id) timer.active = false;
	}
}

void clearInterval(double id) { clearTimeout(id); }

void resetScheduledTimers() {
	for (Timer &timer : gTimers) timer.active = false;
}

double requestAnimationFrame(AnimationFrameCallback callback) {
	if (!callback) return 0;

	gPendingFrames.push_back(std::move(callback));
	return gNextId++;
}

void runAnimationFrameCallbacks(AnimationFrameTimestamp timestampMs) {
	gLastTimestampMs = timestampMs;
	runDueTimers(timestampMs);
	runFrameCallbacks(timestampMs);
}

void resetAnimationFrameCallbacks() {
	gPendingFrames.clear();
	gRunningFrames.clear();
}

}  // namespace gea::host
