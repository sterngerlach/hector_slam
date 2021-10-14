
/* Assert.hpp */

#ifndef HECTOR_SLAM_UTIL_ASSERT_HPP
#define HECTOR_SLAM_UTIL_ASSERT_HPP

#include <cstdlib>
#include <iostream>

/* Assert macro with a custom message (nullptr is allowed) */
#define XAssert(predicate, message) \
  (!(predicate) && AssertHandler(#predicate, __FILE__, __LINE__, (message)))
/* Assert macro with no custom message */
#define Assert(predicate) XAssert((predicate), nullptr)

/* Assert macro with a custom message enabled in Debug mode only */
#ifdef DEBUG
#define DebugXAssert(predicate, message) XAssert((predicate), (message))
#else
#define DebugXAssert(predicate, message)
#endif

/* Assert macro enabled in Debug mode only */
#ifdef DEBUG
#define DebugAssert(predicate) Assert((predicate))
#else
#define DebugAssert(predicate)
#endif

/* Assert function that is enabled in Release mode */
inline bool AssertHandler(const char* predicateExpression,
                          const char* fileName,
                          const int lineNumber,
                          const char* message)
{
  const char* customMessage = message != nullptr ? message : "(None)";
  std::cerr << "Assertion failed: " << predicateExpression << '\n'
            << "File: " << fileName << ", "
            << "Line: " << lineNumber << ", "
            << "Message: " << customMessage << '\n';
  std::abort();
  return true;
}

#endif /* HECTOR_SLAM_UTIL_ASSERT_HPP */
