/**
 * @file test_at_command.c
 * @brief Unit tests for the AT command parser (at_command.h / at_command.c).
 */

#include "unity.h"
#include <string.h>

#include "../src/at_command.c"

void test_role0(void)
{
  char line[] = "+ROLE0\r\n";
  at_command_parse_t r = at_command_parse(line);
  TEST_ASSERT_EQUAL(AT_CMD_ROLE0, r.command);
}

void test_role1_with_at_prefix(void)
{
  char line[] = "AT+ROLE1\n";
  at_command_parse_t r = at_command_parse(line);
  TEST_ASSERT_EQUAL(AT_CMD_ROLE1, r.command);
}

void test_con_argument(void)
{
  char line[] = "+CONAABBCCDDEEFF\r\n";
  at_command_parse_t r = at_command_parse(line);
  TEST_ASSERT_EQUAL(AT_CMD_CON, r.command);
  TEST_ASSERT_EQUAL_STRING("AABBCCDDEEFF", r.argument);
}

void test_name_argument(void)
{
  char line[] = "AT+NAMETX16S\r\n";
  at_command_parse_t r = at_command_parse(line);
  TEST_ASSERT_EQUAL(AT_CMD_NAME, r.command);
  TEST_ASSERT_EQUAL_STRING("TX16S", r.argument);
}

void test_txpw_argument(void)
{
  char line[] = "AT+TXPW2\r\n";
  at_command_parse_t r = at_command_parse(line);
  TEST_ASSERT_EQUAL(AT_CMD_TXPW, r.command);
  TEST_ASSERT_EQUAL_STRING("2", r.argument);
}

void test_disc(void)
{
  char line[] = "AT+DISC?\r\n";
  at_command_parse_t r = at_command_parse(line);
  TEST_ASSERT_EQUAL(AT_CMD_DISC, r.command);
}

void test_clear(void)
{
  char line[] = "AT+CLEAR\r\n";
  at_command_parse_t r = at_command_parse(line);
  TEST_ASSERT_EQUAL(AT_CMD_CLEAR, r.command);
}

void test_baud_argument(void)
{
  char default_baud[] = "AT+BAUD4\r\n";
  char a2dp_baud[] = "AT+BAUD6\r\n";
  at_command_parse_t default_result = at_command_parse(default_baud);
  at_command_parse_t a2dp_result = at_command_parse(a2dp_baud);
  TEST_ASSERT_EQUAL(AT_CMD_BAUD, default_result.command);
  TEST_ASSERT_EQUAL_STRING("4", default_result.argument);
  TEST_ASSERT_EQUAL(AT_CMD_BAUD, a2dp_result.command);
  TEST_ASSERT_EQUAL_STRING("6", a2dp_result.argument);
}

void test_htreset(void)
{
  char line[] = "AT+HTRESET\r\n";
  at_command_parse_t r = at_command_parse(line);
  TEST_ASSERT_EQUAL(AT_CMD_HTRESET, r.command);
}

void test_bluetooth_control_commands(void)
{
  char ble_on[] = "AT+BLEON\r\n";
  char ble_off[] = "AT+BLEOFF\r\n";
  char a2dp_on[] = "AT+A2DPON\r\n";
  char a2dp_off[] = "AT+A2DPOFF\r\n";
  char a2dp_connect[] = "AT+A2DPCONAABBCCDDEEFF\r\n";
  char bt_off[] = "AT+BTOFF\r\n";
  char bt_both[] = "AT+BTBOTH\r\n";

  TEST_ASSERT_EQUAL(AT_CMD_BLEON, at_command_parse(ble_on).command);
  TEST_ASSERT_EQUAL(AT_CMD_BLEOFF, at_command_parse(ble_off).command);
  TEST_ASSERT_EQUAL(AT_CMD_A2DPON, at_command_parse(a2dp_on).command);
  TEST_ASSERT_EQUAL(AT_CMD_A2DPOFF, at_command_parse(a2dp_off).command);
  at_command_parse_t connect_result = at_command_parse(a2dp_connect);
  TEST_ASSERT_EQUAL(AT_CMD_A2DPCON, connect_result.command);
  TEST_ASSERT_EQUAL_STRING("AABBCCDDEEFF", connect_result.argument);
  TEST_ASSERT_EQUAL(AT_CMD_BTOFF, at_command_parse(bt_off).command);
  TEST_ASSERT_EQUAL(AT_CMD_BTBOTH, at_command_parse(bt_both).command);
}

void test_unknown(void)
{
  char line[] = "AT+BOGUS\r\n";
  at_command_parse_t r = at_command_parse(line);
  TEST_ASSERT_EQUAL(AT_CMD_UNKNOWN, r.command);
}

void test_trailing_whitespace_stripped(void)
{
  char line[] = "+NAMEfoo   ";
  at_command_parse_t r = at_command_parse(line);
  TEST_ASSERT_EQUAL(AT_CMD_NAME, r.command);
  TEST_ASSERT_EQUAL_STRING("foo", r.argument);
}

void test_null_input(void)
{
  at_command_parse_t r = at_command_parse(NULL);
  TEST_ASSERT_EQUAL(AT_CMD_UNKNOWN, r.command);
}

void test_empty_input(void)
{
  char line[] = "";
  at_command_parse_t r = at_command_parse(line);
  TEST_ASSERT_EQUAL(AT_CMD_UNKNOWN, r.command);
}

void run_at_command_tests(void)
{
  RUN_TEST(test_role0);
  RUN_TEST(test_role1_with_at_prefix);
  RUN_TEST(test_con_argument);
  RUN_TEST(test_name_argument);
  RUN_TEST(test_txpw_argument);
  RUN_TEST(test_disc);
  RUN_TEST(test_clear);
  RUN_TEST(test_baud_argument);
  RUN_TEST(test_htreset);
  RUN_TEST(test_bluetooth_control_commands);
  RUN_TEST(test_unknown);
  RUN_TEST(test_trailing_whitespace_stripped);
  RUN_TEST(test_null_input);
  RUN_TEST(test_empty_input);
}
