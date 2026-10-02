#pragma once

#include <cstddef>
#include <cstdint>

bool geaAudioAecStart(bool hardwareReference = false, float outputGain = 1.0f,
                      bool aggressiveNlp = true);
void geaAudioAecStop();
void geaAudioAecReference(const int16_t *pcm, size_t count);
void geaAudioAecReceive(const int16_t *stereo, size_t count);
bool geaAudioAecRead(int16_t *pcm, size_t count);
size_t geaAudioAecProcess(const int16_t *pcm, size_t count,
                          const int16_t **result);
