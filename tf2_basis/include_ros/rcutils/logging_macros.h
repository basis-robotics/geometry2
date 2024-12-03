#pragma once

#include <spdlog/fmt/bundled/printf.h>
#include <basis/core/logging/macros.h>


// Helper file to shim over ROS logging
DEFINE_AUTO_LOGGER_NS(tf2)
DECLARE_AUTO_LOGGER_NS(tf2)
#define RCUTILS_LOG_WARN_THROTTLE(_, __, pattern, ...) BASIS_LOG_WARN_NS(tf2, "{}", fmt::sprintf(pattern, __VA_ARGS__))
// TODO
#define RCUTILS_STEADY_TIME(...)
#define RCUTILS_LOG_ERROR(pattern, ...) BASIS_LOG_ERROR_NS(tf2, "{}", fmt::sprintf(pattern, __VA_ARGS__))
#define RCUTILS_LOG_WARN(pattern, ...) BASIS_LOG_WARN_NS(tf2, "{}", fmt::sprintf(pattern, __VA_ARGS__))