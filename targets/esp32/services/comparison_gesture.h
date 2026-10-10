// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "services/comparison_input_trace.h"

#if GEA_EMBEDDED_COMPARISON_BENCHMARK
namespace gea::platform::comparison::gesture {

inline constexpr unsigned kCapacity = 128, kNodeCapacity = 64;
inline constexpr unsigned kGestureClassCount = 8;
inline constexpr const char* kClasses[] = {"menu-icon",
                                           "menu-title",
                                           "dot",
                                           "roller",
                                           "roller-row",
                                           "settings",
                                           "list-button",
                                           "roller-track",
                                           "percentage",
                                           "slider-fill",
                                           "slider-knob",
                                           "switch",
                                           "sw-left",
                                           "sw-right",
                                           "elapsed",
                                           "lap",
                                           "no-laps",
                                           "wheel-label",
                                           "wheel-pointer",
                                           "frequency",
                                           "accel",
                                           "classic-time",
                                           "classic-weekday",
                                           "classic-date",
                                           "simple-time",
                                           "simple-date",
                                           "simple-hint",
                                           "alarm-row",
                                           "alarm-time",
                                           "ringing",
                                           "dialog",
                                           "adjust-title",
                                           "about",
                                           "progress",
                                           "battery",
                                           "badge-hint",
                                           "adjust-summary",
                                           "ok",
                                           "crash-progress",
                                           "section-title",
                                           "laps"};

struct NodeState {
  int id, kind, parent, x, y, width, height, scrollX, scrollY, contentWidth, contentHeight,
      selected, children;
  bool textTruncated;
  std::int16_t cornerX[4], cornerY[4];
  char text[96];
};

struct Observation {
  int requestedMs = 0;
  std::int64_t actualUs = 0;
  unsigned count = 0, dropped = 0;
  NodeState nodes[kNodeCapacity];
};

struct Event {
  int requestedMs = 0, state = 0, x = 0, y = 0, phase = 0;
  std::int64_t actualUs = 0;
};

struct Plan {
  Event events[kCapacity];
  Observation observations[kCapacity];
  unsigned eventCount = 0, observationCount = 0;
  std::int64_t originUs = 0;
  bool complete = false;
  input::Snapshot pointerReads;

  bool addTouch(int ms, int state, int x, int y) {
    if (complete || eventCount == kCapacity || ms < 0 || ms > 15000 || (state != 0 && state != 1) ||
        (eventCount && ms < events[eventCount - 1].requestedMs)) {
      return false;
    }
    events[eventCount++] = {ms, state, x, y, 0, 0};
    return true;
  }

  bool addObservation(int ms) {
    if (complete || observationCount == kCapacity || ms < 0 || ms > 15000 ||
        (observationCount && ms < observations[observationCount - 1].requestedMs)) {
      return false;
    }
    observations[observationCount++].requestedMs = ms;
    return true;
  }

  bool valid() const {
    return !complete && eventCount && events[0].state == 1 && events[eventCount - 1].state == 0;
  }
};

// Events and observations use one device clock, without host transport pacing.
// Injectable operations keep the real queue/coalescing/dispatch path intact.
template <typename Clock, typename Wait, typename Inject, typename Observe>
void play(Plan& plan, Clock clock, Wait wait, Inject inject, Observe observe) {
  plan.originUs = clock();
  unsigned event = 0, observation = 0;
  bool pressed = false;
  while (event < plan.eventCount || observation < plan.observationCount) {
    const int nextEvent = event < plan.eventCount ? plan.events[event].requestedMs : 16000;
    const int nextObservation =
        observation < plan.observationCount ? plan.observations[observation].requestedMs : 16000;
    const std::int64_t deadline = plan.originUs + std::min(nextEvent, nextObservation) * 1000;
    while (clock() < deadline) {
      wait(deadline - clock());
    }
    if (nextEvent <= nextObservation) {
      auto& sample = plan.events[event++];
      sample.phase = sample.state ? (pressed ? 2 : 1) : 3;
      sample.actualUs = clock();
      inject(sample.phase, sample.state != 0, sample.x, sample.y);
      pressed = sample.state != 0;
    } else {
      observe(plan.observations[observation++]);
    }
  }
  plan.complete = true;
}

} // namespace gea::platform::comparison::gesture
#endif
