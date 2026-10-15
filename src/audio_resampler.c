#include "audio_resampler.h"

void audio_resampler_reset(audio_resampler_t *resampler)
{
  if (resampler == NULL) {
    return;
  }

  resampler->phase = 0;
  resampler->current_sample = 0;
  resampler->next_sample = 0;
  resampler->has_current_sample = false;
  resampler->has_next_sample = false;
}

size_t audio_resampler_process(audio_resampler_t *resampler,
                               audio_resampler_read_fn read_sample,
                               void *context,
                               int16_t *output,
                               size_t frames)
{
  if (resampler == NULL || read_sample == NULL || output == NULL) {
    return 0;
  }

  size_t produced = 0;
  while (produced < frames) {
    if (!resampler->has_current_sample) {
      if (!read_sample(context, &resampler->current_sample)) {
        break;
      }
      resampler->has_current_sample = true;
    }

    if (!resampler->has_next_sample) {
      if (!read_sample(context, &resampler->next_sample)) {
        break;
      }
      resampler->has_next_sample = true;
    }

    int32_t difference = (int32_t)resampler->next_sample - resampler->current_sample;
    int64_t interpolated = resampler->current_sample +
                           ((int64_t)difference * resampler->phase) /
                               AUDIO_RESAMPLER_OUTPUT_RATE_HZ;
    output[produced * 2] = (int16_t)interpolated;
    output[produced * 2 + 1] = (int16_t)interpolated;
    produced++;

    resampler->phase += AUDIO_RESAMPLER_INPUT_RATE_HZ;
    while (resampler->phase >= AUDIO_RESAMPLER_OUTPUT_RATE_HZ) {
      resampler->phase -= AUDIO_RESAMPLER_OUTPUT_RATE_HZ;
      resampler->current_sample = resampler->next_sample;
      resampler->has_next_sample = false;
      if (!read_sample(context, &resampler->next_sample)) {
        break;
      }
      resampler->has_next_sample = true;
    }
  }

  return produced;
}