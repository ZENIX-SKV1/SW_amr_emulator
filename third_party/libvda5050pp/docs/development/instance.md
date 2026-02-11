## Instance

The `vda5050pp::core::Instance` is a [singleton](https://en.wikipedia.org/wiki/Singleton_pattern) instance containing the whole state of the
libvda5050++.

For initialization the `Instance::init()` function takes a [config object](../configuration.md#configuration-of-the-libvda5050).
It keeps a read-only copy of it, such that all [modules](#modules) can access it.
Next to a module-registry (see [modules](#modules)) all event managers are held by the instance. It also 
keeps track of all [handlers](../api.md#handler) registered by the user.
Currently the managers used by the [state](state.md#overview) are also part of the instance.

## Modules

Modules are a concept introduced early in development of the library, they are still used. The idea of a
module is to encapsulate behavior (based on events) in an object, that can be loaded by the library on demand.
Meaning it is meant to be possible to not initialize certain modules.

The current module interface contains the following:

- `virtual void initialize()` setup all subscribers and initialize the state of the module.
- `virtual void deinitialize()` remove all subscribers and reset the state of the module. **It is important clean up everything, because modules themselves are only destructed at program exit!**
- `virtual std::string_view describe()` return a module description. This is a legacy function only returning the name of the module. It might get removed some day.
- `virtual std::shared_ptr<vda5050pp::config::ModuleSubConfig> generateSubConfig()` this function may be overridden to return a custom config for the module. This function will be called when
  constructing a `vda5050pp::Config` object, once known to the config it can load it from the toml file.


### Registering a module

A module can be registered `vda5050pp::core::Instance::registerModule()`. This must happen before initialization, thus a convenience wrapper
[`vda5050pp::core::AutoRegisterModule`](/doxygen/html/structvda5050pp_1_1core_1_1AutoRegisterModule.html) was added.

To register your own module, do:
```c++
// include/private/vda5050++/core/instance.h

namespace vda5050pp::core::module_keys {
//...
static constexpr std::string_view k_your_module_key = "YourModule";
//...
}

// src/vda5050++/core/instance.cpp
static const vda5050pp::core::AutoRegisterModule<YourModule,
                                                 module_keys::k_your_module_key>
    _auto_register_your_module;
```

### Logger for your module

For each module a logger is automatically created. You can access it with:

```c++
auto spdlog_logger_ptr = vda5050pp::core::getLogger(module_key::your_module_key);
```

In case you want another name instead of `[YourModule]` in front of each log line, you can add an entry to the
`vda5050pp::core::k_logger_by_module` mapping in `src/vda5050++/core/logger.cpp`. But make sure to use another function
to get the logger:

```c++
auto spdlog_logger_ptr = vda5050pp::core::getRemappedLogger("your_custom_logger_name");
```

The default [`ModuleSubConfig`](#module-config) already contains configuration entries for a log-level and log-file, see [Configuration/logging](../configuration.md#logging).
They are loaded automatically.


### Module Config

To create your own module config, you need to create a class inheriting from `vda5050pp::config::ModuleSubConfig`:

```c++
class YourConfig : public vda5050pp::config::ModuleSubConfig {
private:
  int your_integer_ = 0;

protected:
  void getFrom(const ConstConfigNode &node) override {
    // Call load of parent class to load logging config
    this->ModuleSubConfig::getFrom(node);

    // Cast ConstConfigNode wrapper to an actual toml::node from tomlplusplus
    auto toml_node = vda5050pp::core::config::ConstConfigNode::upcast(node).get();

    // Load your values:
    if (auto maybe_int = toml_node["your_integer"].value<int>(); maybe_int.has_value()) {
      this->your_integer_ = *maybe_int;
    } else {
      // Set default or throw error like:
      throw vda5050pp::VDA5050PPTOMLError(MK_EX_CONTEXT("your_integer is unset"));
    }
  }

  void putTo(ConfigNode &node) const {
    // Call save of parent class to save logging config if changed
    this->ModuleSubConfig::getFrom(node);

    // Cast ConstConfigNode wrapper to an actual toml_node from tomlplusplus
    auto toml_node = vda5050pp::core::config::ConfigNode::upcast(node).get();

    // Insert value into the node (as table)
    toml_node.as_table()->insert("your_integer", this->your_integer_);
  }
};
```

Finally you have to return a pointer to an object of `YourConfig` in `generateSubConfig()`:

```c++
std::shared_ptr<vda5050pp::config::ModuleSubConfig> YourModule::generateSubConfig() const {
  return std::make_shared<YourConfig>();
}
```

To access your config, do:

```c++
auto cfg_ptr = vda5050pp::core::Instance::ref()
                    .getConfig()
                    .lookupModuleConfig(module_keys::your_module_key);
YourConfig &cfg_ref = cfg_ptr->as<YourConfig>(); // May throw on type mismatch
```