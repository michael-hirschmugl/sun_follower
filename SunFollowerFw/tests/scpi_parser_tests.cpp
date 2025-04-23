#include <gtest/gtest.h>
#include "scpi_parser.hpp"

// Test 1: Einfacher LED Befehl
TEST(ScpiParserTest, ParsesLedCommand) {
    ScpiParser parser;
    Command cmd;
    ASSERT_TRUE(parser.parse(":LED:STATE ON", cmd));
    EXPECT_EQ(cmd.subCmd, "STATE");
    EXPECT_FALSE(cmd.isQuery);
    ASSERT_EQ(cmd.params.size(), 1);
    EXPECT_EQ(cmd.params[0], "ON");
}

// Test 2: Status Abfrage
TEST(ScpiParserTest, ParsesStatusQuery) {
    ScpiParser parser;
    Command cmd;
    ASSERT_TRUE(parser.parse(":STATUS?", cmd));
    EXPECT_EQ(cmd.subCmd, "STATUS");
    EXPECT_TRUE(cmd.isQuery);
    EXPECT_EQ(cmd.params.size(), 0);
}

// Test 3: Ungültiger Befehl
TEST(ScpiParserTest, RejectsInvalidCommand) {
    ScpiParser parser;
    Command cmd;
    ASSERT_FALSE(parser.parse("INVALID", cmd));
}
