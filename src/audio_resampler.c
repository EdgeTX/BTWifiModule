#include "audio_resampler.h"

#include <math.h>
#include <string.h>

#define CENTER (AUDIO_RESAMPLER_TAPS / 2 - 1)
#define KAISER_BETA 7.0
#define CUTOFF 0.9  // fraction of the input Nyquist frequency
#define PI_D 3.14159265358979323846

static int16_t s_table[AUDIO_RESAMPLER_PHASES + 1][AUDIO_RESAMPLER_TAPS];
static volatile bool s_table_ready;

static double bessel_i0(double x)
{
  double sum = 1.0;
  double term = 1.0;
  const double q = x * x / 4.0;
  for (int k = 1; k < 50; k++) {
    term *= q / ((double)k * k);
    sum += term;
    if (term < 1e-12 * sum) {
      break;
    }
  }
  return sum;
}

void audio_resampler_init(void)
{
  if (s_table_ready) {
    return;
  }

  const double half = AUDIO_RESAMPLER_TAPS / 2.0;
  const double window_norm = bessel_i0(KAISER_BETA);

  for (int p = 0; p <= AUDIO_RESAMPLER_PHASES; p++) {
    const double frac = (double)p / AUDIO_RESAMPLER_PHASES;
    double row[AUDIO_RESAMPLER_TAPS];
    double sum = 0.0;

    for (int j = 0; j < AUDIO_RESAMPLER_TAPS; j++) {
      const double t = (double)(j - CENTER) - frac;
      const double u = t / half;
      double value = 0.0;
      if (fabs(u) < 1.0) {
        const double x = PI_D * CUTOFF * t;
        const double sinc = (fabs(x) < 1e-9) ? 1.0 : sin(x) / x;
        const double window = bessel_i0(KAISER_BETA * sqrt(1.0 - u * u)) / window_norm;
        value = CUTOFF * sinc * window;
      }
      row[j] = value;
      sum += value;
    }

    // Unity DC gain for every phase.
    for (int j = 0; j < AUDIO_RESAMPLER_TAPS; j++) {
      double q = row[j] / sum * 32768.0;
      s_table[p][j] = (int16_t)lrint(q > 32767.0 ? 32767.0 : q);
    }
  }

  s_table_ready = true;
}

void audio_resampler_reset(audio_resampler_t *resampler)
{
  if (resampler == NULL) {
    return;
  }

  resampler->phase = 0;
  resampler->loaded = 0;
  memset(resampler->history, 0, sizeof(resampler->history));
}

void audio_resampler_set_input_rate(audio_resampler_t *resampler, uint32_t rate_hz)
{
  if (resampler == NULL || rate_hz == 0 || rate_hz > AUDIO_RESAMPLER_OUTPUT_RATE_HZ) {
    return;
  }
  resampler->input_rate = rate_hz;
}

static inline void push_sample(audio_resampler_t *resampler, int16_t sample)
{
  memmove(resampler->history, resampler->history + 1,
          (AUDIO_RESAMPLER_TAPS - 1) * sizeof(int16_t));
  resampler->history[AUDIO_RESAMPLER_TAPS - 1] = sample;
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

  audio_resampler_init();

  const uint32_t input_rate =
      resampler->input_rate ? resampler->input_rate : AUDIO_RESAMPLER_INPUT_RATE_HZ;
  size_t produced = 0;

  while (produced < frames) {
    int16_t sample;

    while (resampler->loaded < AUDIO_RESAMPLER_TAPS) {
      if (!read_sample(context, &sample)) {
        return produced;
      }
      push_sample(resampler, sample);
      resampler->loaded++;
    }

    while (resampler->phase >= AUDIO_RESAMPLER_OUTPUT_RATE_HZ) {
      if (!read_sample(context, &sample)) {
        return produced;
      }
      push_sample(resampler, sample);
      resampler->phase -= AUDIO_RESAMPLER_OUTPUT_RATE_HZ;
    }

    const uint32_t index =
        (resampler->phase * AUDIO_RESAMPLER_PHASES + AUDIO_RESAMPLER_OUTPUT_RATE_HZ / 2) /
        AUDIO_RESAMPLER_OUTPUT_RATE_HZ;
    const int16_t *coeff = s_table[index];

    int32_t acc = 0;
    for (int j = 0; j < AUDIO_RESAMPLER_TAPS; j++) {
      acc += (int32_t)resampler->history[j] * coeff[j];
    }
    acc = (acc + (1 << 14)) >> 15;
    if (acc > INT16_MAX) {
      acc = INT16_MAX;
    } else if (acc < INT16_MIN) {
      acc = INT16_MIN;
    }

    output[produced * 2] = (int16_t)acc;
    output[produced * 2 + 1] = (int16_t)acc;
    produced++;

    resampler->phase += input_rate;
  }

  return produced;
}
