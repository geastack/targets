#include "audio.h"

#include <cstddef>
#include <cstdint>
#include <string>

#include "esp_timer.h"

namespace audio = gea::platform::audio;

namespace {

constexpr std::size_t kOscillatorCount = 8;

struct OscillatorState {
	audio::OscillatorType type = audio::OscillatorType::Sine;
	double frequency = 440.0;
};

OscillatorState oscillators[kOscillatorCount]{};
std::size_t nextOscillator = 0;
int outputVolume = 50;

OscillatorState *oscillator(audio::NativeAudioHandle handle)
{
	if (handle == 0 || handle > kOscillatorCount) return nullptr;
	return &oscillators[handle - 1];
}

audio::NativeAudioHandle createOscillator()
{
	const std::size_t slot = nextOscillator;
	nextOscillator = (nextOscillator + 1) % kOscillatorCount;
	oscillators[slot] = OscillatorState{};
	return static_cast<audio::NativeAudioHandle>(slot + 1);
}

}  // namespace

audio::AudioParam::AudioParam(NativeAudioHandle oscillator) : oscillator_(oscillator) {}
double audio::AudioParam::value() const
{
	const auto *state = oscillator(oscillator_);
	return state ? state->frequency : 0.0;
}
void audio::AudioParam::setValue(double value)
{
	if (auto *state = oscillator(oscillator_)) state->frequency = value;
}
void audio::AudioParam::setValueAtTime(double value, double) { setValue(value); }

audio::AudioNode::AudioNode(NativeAudioHandle native) : native_(native) {}
audio::NativeAudioHandle audio::AudioNode::nativeId() const { return native_; }
audio::AudioDestinationNode::AudioDestinationNode(NativeAudioHandle native) : AudioNode(native) {}
audio::OscillatorNode::OscillatorNode(NativeAudioHandle native) : AudioNode(native), frequency(native) {}
audio::OscillatorType audio::OscillatorNode::type() const
{
	const auto *state = oscillator(nativeId());
	return state ? state->type : OscillatorType::Sine;
}
void audio::OscillatorNode::setType(OscillatorType type)
{
	if (auto *state = oscillator(nativeId())) state->type = type;
}
void audio::OscillatorNode::connect(const AudioDestinationNode &) {}
void audio::OscillatorNode::start(double) {}
void audio::OscillatorNode::stop(double) {}

double audio::AudioContext::currentTime() const
{
	return static_cast<double>(esp_timer_get_time()) / 1000000.0;
}
audio::AudioDestinationNode audio::AudioContext::destination() const { return AudioDestinationNode{}; }
audio::OscillatorNode audio::AudioContext::createOscillator() const { return OscillatorNode(::createOscillator()); }

audio::AudioContext audio::AudioSystem::sharedContext() { return AudioContext{}; }
int audio::AudioSystem::volume() { return outputVolume; }
void audio::AudioSystem::setVolume(int value)
{
	if (value < 0) value = 0;
	if (value > 100) value = 100;
	outputVolume = value;
}
bool audio::AudioSystem::playFile(const std::string &) { return false; }
bool audio::AudioSystem::playPcm(const std::int16_t *, std::size_t, int, int) { return false; }
void audio::AudioSystem::stopPlayback() {}
