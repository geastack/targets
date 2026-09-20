// Host-side proof for the "needs two clicks to reverse" rotary bug + fix.
//
// Mirrors the exact decode loop in main/rotary_encoder.cpp. We drive the
// decoder with a synthetic stream of quadrature pin states and mark where each
// physical detent click ends, then record how many detents the decoder emits
// during each click. `restOffset` models the encoder whose detent REST position
// sits one transition past the fire threshold (the real board): it makes the
// running `partial` settle at +-1 between detents instead of 0.
//
//   restOffset = 0  -> ideal encoder (partial rests at 0)
//   restOffset = 1  -> real board    (partial rests at +-1)
//
// Build & run:
//   c++ -std=c++17 -O2 -o /tmp/rotary_decode_test \
//       targets/esp32-s3-elecrow-rotary-2.1/test/rotary_decode_test.cpp && \
//   /tmp/rotary_decode_test
// (binary is throwaway; source lives in the repo)

#include <cstdint>
#include <cstdio>
#include <vector>

namespace {

constexpr int kTransitionsPerDetent = 2;

int transitionDelta(int previous, int current) {
  static constexpr int8_t table[16] = {
      0, -1, 1, 0,
      1, 0, 0, -1,
      -1, 0, 0, 1,
      0, 1, -1, 0,
  };
  if (previous < 0 || previous > 3 || current < 0 || current > 3) return 0;
  return table[(previous << 2) | current];
}

// CW Gray cycle implied by the transition table: 00 -> 10 -> 11 -> 01 -> 00.
const int kCwCycle[4] = {0b00, 0b10, 0b11, 0b01};

struct Event {
  int state;        // pin state after this transition
  bool clickBoundary;  // true on the final transition of a physical detent click
};

// Build the transition stream for a list of (direction, clicks) segments.
// `restOffset` leading extra transitions (once, up front) phase-shift the rest
// position so `partial` settles at +-1.
std::vector<Event> buildStream(const std::vector<std::pair<int, int>>& segments,
                               int restOffset) {
  std::vector<Event> ev;
  int idx = 0;
  // Seed with the resting state itself so the decoder initializes `previous`
  // from the rest position (as the firmware does via readState() before any
  // motion) and therefore captures the very first transition.
  ev.push_back({kCwCycle[idx], false});
  // Leading offset transitions: motion that shifts the rest phase but isn't a
  // counted click in our accounting.
  for (int i = 0; i < restOffset; ++i) {
    idx = (idx + 1 + 4) % 4;
    ev.push_back({kCwCycle[idx], false});
  }
  for (auto [dir, clicks] : segments) {
    for (int c = 0; c < clicks; ++c) {
      for (int t = 0; t < kTransitionsPerDetent; ++t) {
        idx = (idx + dir + 4) % 4;
        const bool boundary = (t == kTransitionsPerDetent - 1);
        ev.push_back({kCwCycle[idx], boundary});
      }
    }
  }
  return ev;
}

// Run the decoder over the stream; return detents emitted during each click.
std::vector<int> decodePerClick(const std::vector<Event>& ev, bool reversalReset) {
  std::vector<int> perClick;
  if (ev.empty()) return perClick;
  int previous = ev.front().state;
  int partial = 0;
  int firesThisClick = 0;
  for (size_t i = 0; i < ev.size(); ++i) {
    const int current = ev[i].state;
    if (current != previous) {
      const int step = transitionDelta(previous, current);
      if (step == 0) {
        partial = 0;
      } else {
        // NEW logic only: drop stale opposite-direction credit on reversal.
        if (reversalReset &&
            ((step > 0 && partial < 0) || (step < 0 && partial > 0)))
          partial = 0;
        partial += step;
        if (partial >= kTransitionsPerDetent || partial <= -kTransitionsPerDetent) {
          partial = 0;
          ++firesThisClick;
        }
      }
      previous = current;
    }
    if (ev[i].clickBoundary) {
      perClick.push_back(firesThisClick);
      firesThisClick = 0;
    }
  }
  return perClick;
}

void report(const char* label, int restOffset, bool reversalReset) {
  // 4 forward clicks, then 4 reverse clicks.
  auto ev = buildStream({{+1, 4}, {-1, 4}}, restOffset);
  auto perClick = decodePerClick(ev, reversalReset);
  // perClick[0..3] = forward, perClick[4..7] = reverse.
  printf("%-14s fwd=[%d %d %d %d]  rev=[%d %d %d %d]  -> %s\n", label,
         perClick[0], perClick[1], perClick[2], perClick[3], perClick[4],
         perClick[5], perClick[6], perClick[7],
         perClick[4] == 1 ? "OK (1st reverse click registers)"
                          : "BUG (1st reverse click lost -> needs 2 clicks)");
}

}  // namespace

int main() {
  printf("Each cell = detents emitted for that physical click.\n\n");
  printf("Ideal encoder (rest offset = 0, partial rests at 0):\n");
  report("  OLD", 0, false);
  report("  NEW", 0, true);
  printf("\nReal board (rest offset = 1, partial rests at +-1):\n");
  report("  OLD", 1, false);
  report("  NEW", 1, true);
  return 0;
}
