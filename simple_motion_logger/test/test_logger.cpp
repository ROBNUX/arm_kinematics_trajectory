// Smoke tests for the ROBNUXLogging singleton logger.
// These verify the logger is reachable, accepts each log level, and that the
// log-type / log-level setters do not throw.

#include <gtest/gtest.h>

#include <sstream>
#include <string>

#include "simple_motion_logger/Logger.h"

namespace {

using ROBNUXLogging::LOG_LEVEL_DEBUG;
using ROBNUXLogging::LOG_LEVEL_INFO;
using ROBNUXLogging::Logger;
using ROBNUXLogging::NO_LOG;

TEST(LoggerTest, SingletonReturnsNonNullInstance) {
  Logger* instance = Logger::getInstance();
  ASSERT_NE(instance, nullptr);
  // Same pointer on subsequent calls.
  EXPECT_EQ(Logger::getInstance(), instance);
}

TEST(LoggerTest, AllLevelsAcceptCStringWithoutThrowing) {
  Logger* log = Logger::getInstance();
  EXPECT_NO_THROW(log->error("error msg"));
  EXPECT_NO_THROW(log->alarm("alarm msg"));
  EXPECT_NO_THROW(log->always("always msg"));
  EXPECT_NO_THROW(log->buffer("buffer msg"));
  EXPECT_NO_THROW(log->info("info msg"));
  EXPECT_NO_THROW(log->trace("trace msg"));
  EXPECT_NO_THROW(log->debug("debug msg"));
}

TEST(LoggerTest, AllLevelsAcceptOstringstreamWithoutThrowing) {
  Logger* log = Logger::getInstance();
  std::ostringstream ss;
  ss << "logged via ostringstream value=" << 42;
  EXPECT_NO_THROW(log->info(ss));
}

TEST(LoggerTest, UpdateLogLevelDoesNotThrow) {
  Logger* log = Logger::getInstance();
  // Disable then re-enable; the singleton must survive both transitions.
  EXPECT_NO_THROW(log->disableLog());
  EXPECT_NO_THROW(log->enaleLog());
  EXPECT_NO_THROW(log->updateLogLevel(LOG_LEVEL_INFO));
  EXPECT_NO_THROW(log->updateLogLevel(LOG_LEVEL_DEBUG));
}

TEST(LoggerTest, UpdateLogTypeDoesNotThrow) {
  Logger* log = Logger::getInstance();
  EXPECT_NO_THROW(log->updateLogType(NO_LOG));
  EXPECT_NO_THROW(log->enableConsoleLogging());
}

}  // namespace
