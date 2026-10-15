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

void test_audio_resampler_interpolates_and_duplicates_mono(void)
{
  const int16_t input[] = {-1000, 1000, 3000, 5000};
  sample_source_t source = {.samples = input, .count = 4, .index = 0};
  audio_resampler_t resampler;
  int16_t output[4] = {0};
  audio_resampler_reset(&resampler);

  size_t frames = audio_resampler_process(&resampler, read_sample, &source, output, 2);

  TEST_ASSERT_EQUAL_size_t(2, frames);
  TEST_ASSERT_EQUAL_INT16(-1000, output[0]);
  TEST_ASSERT_EQUAL_INT16(-1000, output[1]);
  TEST_ASSERT_EQUAL_INT16(-275, output[2]);
  TEST_ASSERT_EQUAL_INT16(-275, output[3]);
}

void test_audio_resampler_stops_when_input_is_empty(void)
{
  const int16_t input[] = {100, 200};
  sample_source_t source = {.samples = input, .count = 2, .index = 0};
  audio_resampler_t resampler;
  int16_t output[16] = {0};
  audio_resampler_reset(&resampler);

  size_t frames = audio_resampler_process(&resampler, read_sample, &source, output, 8);

  TEST_ASSERT_EQUAL_size_t(3, frames);
}

void run_audio_resampler_tests(void)
{
  RUN_TEST(test_audio_resampler_interpolates_and_duplicates_mono);
  RUN_TEST(test_audio_resampler_stops_when_input_is_empty);
}