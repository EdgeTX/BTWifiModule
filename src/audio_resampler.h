#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AUDIO_RESAMPLER_INPUT_RATE_HZ 16000U  /**< default input rate */
#define AUDIO_RESAMPLER_OUTPUT_RATE_HZ 44100U

/** FIR length of the polyphase interpolator (input samples spanned). */
#define AUDIO_RESAMPLER_TAPS 24
/** Fractional-delay resolution; the output uses the nearest of these phases. */
#define AUDIO_RESAMPLER_PHASES 256

/** Read one signed PCM sample from the input source. */
typedef bool (*audio_resampler_read_fn)(void *context, int16_t *sample);

/** Streaming state for mono-to-stereo PCM resampling. */
typedef struct {
  uint32_t input_rate;  //< input rate in Hz, 0 selects the default
  uint32_t phase;       //< fractional input position, in 1/OUTPUT_RATE units
  uint32_t loaded;      //< input samples loaded into @ref history since reset
  int16_t history[AUDIO_RESAMPLER_TAPS];  //< oldest sample first
} audio_resampler_t;

/**
 * Build the shared filter table (windowed-sinc, Kaiser window, Q15). Call once
 * from a non-realtime context; it is built lazily otherwise.
 */
void audio_resampler_init(void);

/** Reset the filter state after an input underrun or stream restart. Keeps the input rate. */
void audio_resampler_reset(audio_resampler_t *resampler);

/** Select the input sample rate (must be <= the 44.1 kHz output rate). */
void audio_resampler_set_input_rate(audio_resampler_t *resampler, uint32_t rate_hz);

/**
 * Resample mono input into interleaved stereo output with a band-limited
 * polyphase interpolator. The cutoff sits just below the input Nyquist
 * frequency so the spectral images created by upsampling are suppressed.
 * Group delay is AUDIO_RESAMPLER_TAPS / 2 input samples.
 *
 * @param resampler  Persistent streaming state.
 * @param read_sample Callback that supplies the next input sample.
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
