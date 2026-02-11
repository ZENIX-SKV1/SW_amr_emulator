# Events

The events used in the libVDA5050++ are propagated using the [eventpp](https://github.com/wqking/eventpp) library.
They are categorized into different types like `vda5050pp::events::ActionEvent`s or `vda5050pp::events::NavigationEvent`s.
Each type has an associated event manager like `vda5050pp::core::ActionEventManager` or `vda5050pp::core::NavigationEventManager` respectively.

These managers are essentially an event queue paired with a thread, which can dispatch all registered callbacks for a
new event. Each manager has a dispatch function, which can either dispatch the event `synchronously`, calling the
callbacks with the event dispatching thread or `asynchronously` enqueueing the event for the manager's thread.
The default `synchronous` toggle can be set in each Manager's constructor by passing in a `vda5050pp::core::events::EventManagerOptions`
object.
In order to subscribe to events, each manager provides a function to get a scoped event subscriber.

In general each "component" of the library has an own event manager, that is stored in the [Instance](instance.md)

## public vs private events

Some events live in the [public](design-guildelines.md#public-and-private-namespace) `vda5050pp::` namespace, such that their managers have to _hide_ their dependency to eventpp, which
requires them to have an explicit implementation, that does not rely on templates. This differentiates them from managers for events
used only internally in the `vda5050pp::core` namespace, which can simply be of type `vda5050pp::core::GenericEventManager<EventType>`


## Templated events and managers

Private events can easily be created and managed with the templates defined in `vda5050++/events/event_types.h` and
`vda5050pp/core/generic_event_manager.h`.

Events managed by the `vda5050pp::core::GenericEventManager` need to inherit `vda5050pp::events::EventBase` and
must provide a `EventType::EventIdType` sub type, which is used to identify the concrete type of an event.
This is best done by extending the `vda5050pp::events::Event` type:

```c++
// Identifier type for MyEvent
enum class MyEventId {
  k_type1,
  k_type2,
};

// Base type for MyEvent distinguished with MyEventId objects
struct MyEvent : public vda5050pp::events::Event<MyEventId> {
  std::string my_common_data;
};
```

Now a `vda5050pp::core::GenericEventManager` type could be created for `MyEvent`, however
there are no concrete types of `MyEvent`. Together with the base event type `MyEvent`
and a concrete identifier object of type `MyEventId` these can be defined with:

```c++
// Concrete event types
struct MyEventType1 : public vda5050pp::events::EventId<MyEvent, MyEventId::k_type1> {
  int my_special_data = 0;
};

// Concrete event types
struct MyEventType2 : public vda5050pp::events::EventId<MyEvent, MyEventId::k_type2> {
  std::vector<int> my_special_data;
};
```

The `vda5050pp::core::GenericEventManager` can now be created and used with:

```c++
vda5050pp::core::events::EventManagerOpts opts{};

vda5050pp::core::GenericEventManager<MyEvent> manager(opts);

auto sub = manager.getScopedSubscriber();
sub.subscribe<MyEventType1>([](std::shared_ptr<MyEventType1> evt) {
  // Handle type 1 events
});

sub.subscribe<MyEventType2>([](std::shared_ptr<MyEventType2> evt) {
  // Handle type 2 events
});

manager.dispatch(std::make_shared<MyEventType1>{ /*...*/ });
manager.dispatch(std::make_shared<MyEventType2>{ /*...*/ });
```


## Synchronized events / obtaining event results

One inherent property of events are, that they do not "return" data. However in some
situations it may be desirable to wait for some result of of an event handler. This
is usually done with `std::promise` and `std::future`, which could very well be put into
custom events. Since this behavior is required in different places of the library, a
special event class `vda5050pp::events::SynchronizedEvent` was created.
Let's say the event from the previous section should return a `std::string` as a result:

```c++
struct MyEvent
  : public vda5050pp::events::Event<MyEventId>,
    public vda5050pp::events::SynchronizedEvent<std::string> {
  std::string my_common_data;
};
```

`MyEvent` now has the member functions `MyEvent::acquireResultToken()` and `MyEvent::getFuture()`.
`getFuture()` can be used to get a `std::future<std::string>` which will eventually contain the result
for the event. To set a result, one and **only** one thread may acquire a valid result token
by calling `acquireResultToken()`.

```c++
auto my_event = std::make_shared<MyEventType1>();
std::future<std::string> future = my_event->getFuture();

manager.subscribe<MyEventType1>([](std::shared_ptr<MyEventType1> evt){
  if (auto tkn = evt->acquireResultToken(); tkn) {
    // We have the result token and can set the value
    tkn->setValue("Here, have your result!");
  } else {
    // Failed to acquire result token (another thread already has it)
  }
});

manager.dispatch(my_event);
std::cout << future->get() << std::endl;
```