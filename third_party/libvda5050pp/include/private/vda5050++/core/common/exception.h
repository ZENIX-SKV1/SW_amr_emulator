//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_COMMON_EXCEPTION_H_
#define VDA5050_2B_2B_CORE_COMMON_EXCEPTION_H_

#include <vector>

#include "vda5050++/exception.h"

///
///\brief Construct an exception context inside of a class
/// Also gathers a stacktrace of depth 30
///
#define MK_CLASS_EX_CONTEXT(description) \
  vda5050pp::core::common::mkClassExceptionContext<decltype(*this)>(description)
///
///\brief Construct an exception context inside of a free function
/// Also gathers a stacktrace of depth 30
///
#define MK_FN_EX_CONTEXT(description) \
  vda5050pp::core::common::mkFnExceptionContext(std::string(__FUNCTION__), description)

///
///\brief Create an exception context inside of a class member function
/// Also gathers a stacktrace of depth 30
///
#define MK_EX_CONTEXT(description)                                                        \
  vda5050pp::core::common::mkExceptionContext<decltype(*this)>(std::string(__FUNCTION__), \
                                                               description)

namespace vda5050pp::core::common {

///
///\brief Get the current stack trace
///
/// Requires LIBVDA5050PP_HAS_BACKTRACE, i.e. execinfo's backtrace and backtrace_symbols
///
///\param trace_size the depth of the stack trace
///\return std::vector<vda5050pp::VDA5050PPErrorContext::StackEntry> a vector of stack entries
///
std::vector<vda5050pp::VDA5050PPErrorContext::StackEntry> stacktrace(size_t trace_size = 15);

template <typename ClassT>
inline vda5050pp::VDA5050PPErrorContext mkClassExceptionContext(
    std::optional<std::string> &&description) noexcept(true) {
  VDA5050PPErrorContext context;
  context.class_context = typeid(ClassT).name();
  context.description = std::move(description);
  context.stack_trace = stacktrace(30);
  return context;
}

inline vda5050pp::VDA5050PPErrorContext mkFnExceptionContext(
    std::string &&function_context, std::optional<std::string> &&description) noexcept(true) {
  VDA5050PPErrorContext context;
  context.function_context = std::move(function_context);
  context.description = std::move(description);
  context.stack_trace = stacktrace(30);
  return context;
}

template <typename ClassT>
inline vda5050pp::VDA5050PPErrorContext mkExceptionContext(
    std::string &&function_context, std::optional<std::string> &&description) noexcept(true) {
  VDA5050PPErrorContext context;
  context.class_context = typeid(ClassT).name();
  context.function_context = std::move(function_context);
  context.description = std::move(description);
  context.stack_trace = stacktrace(30);
  return context;
}

}  // namespace vda5050pp::core::common

#endif  // VDA5050_2B_2B_CORE_COMMON_EXCEPTION_H_
