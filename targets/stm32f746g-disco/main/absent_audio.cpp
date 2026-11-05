// No audio output yet (the WM8994 codec on SAI2 is not brought up): an
// <audio> element stays silent.

#include "audio.h"

namespace gea::platform::audio {

bool AudioSystem::playFile(const std::string &) { return false; }

}  // namespace gea::platform::audio
