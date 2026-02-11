## Exceptions

All exceptions thrown by the library should inherit from [`vda5050pp::VDA5050PPError`](/doxygen/html/classvda5050pp_1_1VDA5050PPError.html).

There is already an extensive list of existing error classes. If none matches your error, feel free to add a new one.
Make sure to forward an [`vda5050pp::VDA5050PPErrorContext`](/doxygen/html/structvda5050pp_1_1VDA5050PPErrorContext.html) to the base-classes ctor.
The `format()` member function should be overridden, you probably want to implement it like:

```c++
std::string VDA5050PPYourError::format() const noexcept(true) {
  return this->formatDefault("VDA5050PPYourError");
}
```

### Creating exceptions

Exceptions require [`vda5050pp::VDA5050PPErrorContext`](/doxygen/html/structvda5050pp_1_1VDA5050PPErrorContext.html) for construction.
These contexts contain information about the source of the error, such as the class name, function name and n stack-track if available.

There are three macros which help you to construct an exception context:

```c++
// Include the core exception macros
#include "vda5050++/core/common/exception.h"

// Inside class member functions
throw vda5050pp::VDA5050PPYourError(MK_EX_CONTEXT("description"));

// Inside free functions
throw vda5050pp::VDA5050PPYourError(MK_FN_EX_CONTEXT("description"));

// Inside classes
throw vda5050pp::VDA5050PPYourError(MK_CLASS_EX_CONTEXT("description"));
```