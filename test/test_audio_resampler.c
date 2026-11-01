#include <math.h>
#include <unity.h>

#include "../src/audio_resampler.c"

typedef struct {
  const int16_t *samples;
  size_t count;
  size_t index;
} sample_source_t;

static bool read_sample(void *context, int16_t *sample)
{
  sample_source_t *source = context;
  if (source->index >= source->count) {
    return false;
  }
  *sample = source->samples[source->index++];
  return true;
}

static void fill_sine(int16_t *buffer, size_t count, double freq, double rate, double amplitude)
{
  for (size_t i = 0; i < count; i++) {
    buffer[i] = (int16_t)(amplitude * sin(2.0 * M_PI * freq * (double)i / rate));
  }
}

// Output frame m sits at input position CENTER + m * input_rate / output_rate.
static void check_sine(uint32_t input_rate, double freq, double tolerance)
{
  enum { INPUT_COUNT = 1200, FRAMES = 1000 };
  static int16_t input[INPUT_COUNT];
  static int16_t output[2 * FRAMES];
  const double amplitude = 10000.0;
  fill_sine(input, INPUT_COUNT, freq, input_rate, amplitude);

  sample_source_t source = {.samples = input, .count = INPUT_COUNT, .index = 0};
  audio_resampler_t resampler = {0};
  audio_resampler_set_input_rate(&resampler, input_rate);

  size_t frames = audio_resampler_process(&resampler, read_sample, &source, output, FRAMES);
  TEST_ASSERT_EQUAL_size_t(FRAMES, frames);

  for (size_t m = 0; m < frames; m++) {
    double position = (AUDIO_RESAMPLER_TAPS / 2 - 1) +
                      (double)m * input_rate / AUDIO_RESAMPLER_OUTPUT_RATE_HZ;
    double expected = amplitude * sin(2.0 * M_PI * freq * position / input_rate);
    TEST_ASSERT_DOUBLE_WITHIN(tolerance * amplitude, expected, output[2 * m]);
    TEST_ASSERT_EQUAL_INT16(output[2 * m], output[2 * m + 1]);
  }
}

void test_audio_resampler_keeps_dc_level(void)
{
  int16_t input[200];
  for (int i = 0; i < 200; i++) {
    input[i] = 1000;
  }
  sample_source_t source = {.samples = input, .count = 200, .index = 0};
  audio_resampler_t resampler = {0};
  int16_t output[2 * 100];

  size_t frames = audio_resampler_process(&resampler, read_sample, &source, output, 100);

  TEST_ASSERT_EQUAL_size_t(100, frames);
  for (size_t m = 0; m < frames; m++) {
    TEST_ASSERT_INT16_WITHIN(2, 1000, output[2 * m]);
  }
}

void test_audio_resampler_16khz_sine_is_accurate(void)
{
  check_sine(16000, 2000.0, 0.01);
}

void test_audio_resampler_32khz_sine_is_accurate(void)
{
  check_sine(32000, 5000.0, 0.01);
}

void test_audio_resampler_stops_when_input_is_empty(void)
{
  const int16_t input[] = {100, 200};
  sample_source_t source = {.samples = input, .count = 2, .index = 0};
  audio_resampler_t resampler = {0};
  int16_t output[16] = {0};
  audio_resampler_reset(&resampler);

  size_t frames = audio_resampler_process(&resampler, read_sample, &source, output, 8);

  // The FIR needs a full window of input before the first frame.
  TEST_ASSERT_EQUAL_size_t(0, frames);
}

void test_audio_resampler_32khz_input_consumes_more_samples(void)
{
  static int16_t input[256];
  for (int i = 0; i < 256; i++) {
    input[i] = (int16_t)(i * 10);
  }
  sample_source_t source16 = {.samples = input, .count = 256, .index = 0};
  sample_source_t source32 = {.samples = input, .count = 256, .index = 0};
  audio_resampler_t r16 = {0};
  audio_resampler_t r32 = {0};
  int16_t output[2 * 100] = {0};
  audio_resampler_set_input_rate(&r32, 32000);

  audio_resampler_process(&r16, read_sample, &source16, output, 100);
  audio_resampler_process(&r32, read_sample, &source32, output, 100);

  TEST_ASSERT_GREATER_THAN_size_t(source16.index + 10, source32.index);
}

void run_audio_resampler_tests(void)
{
  RUN_TEST(test_audio_resampler_keeps_dc_level);
  RUN_TEST(test_audio_resampler_16khz_sine_is_accurate);
  RUN_TEST(test_audio_resampler_32khz_sine_is_accurate);
  RUN_TEST(test_audio_resampler_stops_when_input_is_empty);
  RUN_TEST(test_audio_resampler_32khz_input_consumes_more_samples);
}
