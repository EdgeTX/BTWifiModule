#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AUDIO_RESAMPLER_INPUT_RATE_HZ 16000U
#define AUDIO_RESAMPLER_OUTPUT_RATE_HZ 44100U

/** Read one signed PCM sample from the input source. */
typedef bool (*audio_resampler_read_fn)(void *context, int16_t *sample);

/** Streaming state for mono-to-stereo PCM resampling. */
typedef struct {
  uint32_t phase;
  int16_t current_sample;
  int16_t next_sample;
  bool has_current_sample;
  bool has_next_sample;
} audio_resampler_t;

/** Reset the interpolation state after an input underrun or stream restart. */
void audio_resampler_reset(audio_resampler_t *resampler);

/**
 * Resample mono input into interleaved stereo output using linear interpolation.
 *
 * @param resampler  Persistent streaming state.
 * @param read_sample Callback that supplies the next 16 kHz input sample.
 * @param context    Opaque context passed to @p read_sample.
 * @param output     Interleaved left/right 16-bit output samples.
 * @param frames     Maximum number of stereo output frames.
 * @return Number of output frames produced before the input ran dry.
 */
size_t audio_resampler_process(audio_resampler_t *resampler,
                               audio_resampler_read_fn read_sample,
                               void *context,
                               int16_t *output,
                               size_t frames);