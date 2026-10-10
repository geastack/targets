#pragma once

#include <cstddef>
#include <cstdint>

// Two physical inputs require paired ADC slots [mic1, reference, mic2].
// They are averaged before one AEC channel; published output remains mono.
bool geaAudioAecStart(bool hardwareReference = false, float outputGain = 1.0f,
                      bool aggressiveNlp = true, unsigned microphoneCount = 1, bool adaptiveGain = false);
void geaAudioAecStop();
void geaAudioAecReference(const int16_t *pcm, size_t count);
void geaAudioAecReceive(const int16_t *interleaved, size_t count);
bool geaAudioAecRead(int16_t *pcm, size_t count);
size_t geaAudioAecProcess(const int16_t *pcm, size_t count,
                          const int16_t **result);

#if GEA_AUDIO_EXPERIMENT
// Lab-only runtime controls. Updates are synchronized with the DSP frame.
void geaAudioAecExperimentDefaults(float gain, bool aggressive, unsigned microphones, bool agc);
bool geaAudioAecConfigure(const char *key, float value);
void geaAudioAecDescribe(char *buffer, size_t capacity);
// Bounded diagnostic capture: interleaved physical mic1, mic2, speaker reference.
// Read only after capture has finished/stopped; never changes published audio.
bool geaAudioAecCaptureRead(size_t offset, int16_t *samples, size_t capacity,
                            size_t *frames, size_t *total, int64_t *startedUs,
                            uint32_t *drops);
bool geaAudioExperimentConfigure(const char *key, float value);
void geaAudioExperimentDescribe(char *buffer, size_t capacity);
#endif
