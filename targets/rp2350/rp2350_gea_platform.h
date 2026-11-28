#pragma once

namespace gea::rp2350 {

void memoryInit();
void boardIoInit();
void pollTouch();
void pollButtons();
void dispatchQueuedEvents();
// Starts a GEADEV DRAG synthetic touch gesture; advanced frame-by-frame from
// pollTouch(). Returns false if touch is unavailable or a drag is running.
bool startSyntheticDrag(int x1, int y1, int x2, int y2, int steps, int delayMs);
// Maps a logical display row to its framebuffer row (identity unless the
// software scroll register's circular remap is active). Lets GEADEV
// SCREENSHOT serialize logical rows so captures are scroll-offset-invariant.
int scanoutRowToPhysical(int row);

}  // namespace gea::rp2350
