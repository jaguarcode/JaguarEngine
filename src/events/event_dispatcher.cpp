/**
 * @file event_dispatcher.cpp
 * @brief Event dispatcher implementation
 */

#include "jaguar/events/event_dispatcher.h"
#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>

namespace jaguar::events {

// ============================================================================
// EventDispatcher Implementation Details
// ============================================================================

struct EventDispatcher::Impl {
    EventDispatcherConfig config;

    // Handler storage
    std::vector<HandlerInfo> handlers;
    EventHandlerId next_handler_id{1};

    // Event queue (priority queue using vector + heap operations)
    std::vector<Event> event_queue;

    // Event history
    std::vector<Event> event_history;
    bool history_enabled{false};

    // Statistics
    EventStatistics stats;

    // Dispatch state
    bool is_dispatching{false};

    // Thread safety
    mutable std::shared_mutex mutex;

    // Priority comparison for queue
    static bool compare_priority(const Event& a, const Event& b) {
        // Lower priority value = higher priority (process first)
        return static_cast<UInt8>(a.priority()) > static_cast<UInt8>(b.priority());
    }
};

// ============================================================================
// EventDispatcher Construction/Destruction
// ============================================================================

EventDispatcher::EventDispatcher()
    : impl_(std::make_unique<Impl>()) {
}

EventDispatcher::EventDispatcher(const EventDispatcherConfig& config)
    : impl_(std::make_unique<Impl>()) {
    impl_->config = config;
    impl_->history_enabled = config.enable_event_history;
    impl_->event_history.reserve(config.history_size);
}

EventDispatcher::~EventDispatcher() = default;

EventDispatcher::EventDispatcher(EventDispatcher&&) noexcept = default;
EventDispatcher& EventDispatcher::operator=(EventDispatcher&&) noexcept = default;

// ============================================================================
// Handler Registration
// ============================================================================

EventHandlerId EventDispatcher::subscribe(EventType type, EventHandler handler,
                                          const std::string& name, int order) {
    if (!handler) {
        return INVALID_HANDLER_ID;
    }

    std::unique_lock lock(impl_->mutex);

    // Check handler limit
    SizeT type_count = 0;
    for (const auto& h : impl_->handlers) {
        if (h.type == type) {
            type_count++;
        }
    }

    if (type_count >= impl_->config.max_handlers_per_type) {
        return INVALID_HANDLER_ID;
    }

    HandlerInfo info;
    info.id = impl_->next_handler_id++;
    info.handler = std::move(handler);
    info.type = type;
    info.category_filter = EventCategory::None;  // Type-specific handler
    info.order = order;
    info.name = name;
    info.enabled = true;

    impl_->handlers.push_back(std::move(info));

    // Sort by order
    std::stable_sort(impl_->handlers.begin(), impl_->handlers.end(),
        [](const HandlerInfo& a, const HandlerInfo& b) {
            return a.order < b.order;
        });

    return info.id;
}

EventHandlerId EventDispatcher::subscribe_category(EventCategory category,
                                                   EventHandler handler,
                                                   const std::string& name,
                                                   int order) {
    if (!handler) {
        return INVALID_HANDLER_ID;
    }

    std::unique_lock lock(impl_->mutex);

    HandlerInfo info;
    info.id = impl_->next_handler_id++;
    info.handler = std::move(handler);
    info.type = EventType::UserDefined;  // Not used for category handlers
    info.category_filter = category;
    info.order = order;
    info.name = name;
    info.enabled = true;

    impl_->handlers.push_back(std::move(info));

    std::stable_sort(impl_->handlers.begin(), impl_->handlers.end(),
        [](const HandlerInfo& a, const HandlerInfo& b) {
            return a.order < b.order;
        });

    return info.id;
}

EventHandlerId EventDispatcher::subscribe_all(EventHandler handler,
                                              const std::string& name,
                                              int order) {
    return subscribe_category(EventCategory::All, std::move(handler), name, order);
}

bool EventDispatcher::unsubscribe(EventHandlerId id) {
    if (id == INVALID_HANDLER_ID) {
        return false;
    }

    std::unique_lock lock(impl_->mutex);

    auto it = std::find_if(impl_->handlers.begin(), impl_->handlers.end(),
        [id](const HandlerInfo& info) { return info.id == id; });

    if (it != impl_->handlers.end()) {
        impl_->handlers.erase(it);
        return true;
    }

    return false;
}

void EventDispatcher::set_handler_enabled(EventHandlerId id, bool enabled) {
    std::unique_lock lock(impl_->mutex);

    for (auto& handler : impl_->handlers) {
        if (handler.id == id) {
            handler.enabled = enabled;
            break;
        }
    }
}

bool EventDispatcher::is_handler_enabled(EventHandlerId id) const {
    std::shared_lock lock(impl_->mutex);

    for (const auto& handler : impl_->handlers) {
        if (handler.id == id) {
            return handler.enabled;
        }
    }

    return false;
}

SizeT EventDispatcher::handler_count(EventType type) const {
    std::shared_lock lock(impl_->mutex);

    SizeT count = 0;
    for (const auto& handler : impl_->handlers) {
        if (handler.type == type && handler.category_filter == EventCategory::None) {
            count++;
        }
    }

    return count;
}

SizeT EventDispatcher::total_handler_count() const {
    std::shared_lock lock(impl_->mutex);
    return impl_->handlers.size();
}

// ============================================================================
// Event Dispatching
// ============================================================================

void EventDispatcher::dispatch_to_handlers(Event& event) {
    EventCategory event_category = event.category();
    EventType event_type = event.type();

    for (auto& handler_info : impl_->handlers) {
        if (!handler_info.enabled) {
            continue;
        }

        // Check if this handler should receive the event
        bool should_receive = false;

        if (handler_info.category_filter != EventCategory::None) {
            // Category-based handler
            if (has_category(handler_info.category_filter, event_category) ||
                handler_info.category_filter == EventCategory::All) {
                should_receive = true;
            }
        } else {
            // Type-specific handler
            if (handler_info.type == event_type) {
                should_receive = true;
            }
        }

        if (should_receive) {
            // Check priority threshold
            if (static_cast<UInt8>(event.priority()) <=
                static_cast<UInt8>(handler_info.min_priority)) {

                impl_->stats.total_handlers_called++;

                // Call handler
                bool continue_propagation = handler_info.handler(event);

                // Check if event was consumed
                if (!continue_propagation || event.is_consumed()) {
                    break;
                }
            }
        }
    }
}

void EventDispatcher::dispatch(Event& event) {
    // Check for recursive dispatch
    if (impl_->is_dispatching && !impl_->config.allow_recursive_dispatch) {
        // Queue instead of dispatching recursively
        queue(event);
        return;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    {
        std::unique_lock lock(impl_->mutex);
        impl_->is_dispatching = true;
    }

    // Dispatch to handlers
    dispatch_to_handlers(event);

    {
        std::unique_lock lock(impl_->mutex);
        impl_->is_dispatching = false;

        // Update statistics
        impl_->stats.total_events_dispatched++;
        impl_->stats.events_by_type[event.type()]++;

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time).count();

        // Update average dispatch time (exponential moving average)
        impl_->stats.average_dispatch_time_us =
            impl_->stats.average_dispatch_time_us * 0.9 +
            static_cast<Real>(duration) * 0.1;

        if (static_cast<Real>(duration) > impl_->stats.max_dispatch_time_us) {
            impl_->stats.max_dispatch_time_us = static_cast<Real>(duration);
        }

        // Record history
        if (impl_->history_enabled) {
            if (impl_->event_history.size() >= impl_->config.history_size) {
                impl_->event_history.erase(impl_->event_history.begin());
            }
            impl_->event_history.push_back(event);
        }
    }
}

void EventDispatcher::dispatch(Event&& event) {
    Event e = std::move(event);
    dispatch(e);
}

// ============================================================================
// Event Queue
// ============================================================================

bool EventDispatcher::queue(Event event) {
    std::unique_lock lock(impl_->mutex);

    if (impl_->event_queue.size() >= impl_->config.max_queue_size) {
        impl_->stats.total_events_dropped++;
        return false;
    }

    impl_->event_queue.push_back(std::move(event));
    std::push_heap(impl_->event_queue.begin(), impl_->event_queue.end(),
                   Impl::compare_priority);

    impl_->stats.total_events_queued++;

    return true;
}


SizeT EventDispatcher::flush_queue() {
    SizeT processed = 0;

    while (true) {
        Event event;

        {
            std::unique_lock lock(impl_->mutex);

            if (impl_->event_queue.empty()) {
                break;
            }

            std::pop_heap(impl_->event_queue.begin(), impl_->event_queue.end(),
                         Impl::compare_priority);
            event = std::move(impl_->event_queue.back());
            impl_->event_queue.pop_back();
        }

        dispatch(event);
        processed++;
    }

    return processed;
}

SizeT EventDispatcher::flush_queue_timed(Real max_time_ms) {
    SizeT processed = 0;
    auto start_time = std::chrono::high_resolution_clock::now();

    while (true) {
        // Check time limit
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - start_time).count();

        if (static_cast<Real>(elapsed) >= max_time_ms) {
            break;
        }

        Event event;

        {
            std::unique_lock lock(impl_->mutex);

            if (impl_->event_queue.empty()) {
                break;
            }

            std::pop_heap(impl_->event_queue.begin(), impl_->event_queue.end(),
                         Impl::compare_priority);
            event = std::move(impl_->event_queue.back());
            impl_->event_queue.pop_back();
        }

        dispatch(event);
        processed++;
    }

    return processed;
}

void EventDispatcher::clear_queue() {
    std::unique_lock lock(impl_->mutex);
    impl_->event_queue.clear();
}

SizeT EventDispatcher::queue_size() const {
    std::shared_lock lock(impl_->mutex);
    return impl_->event_queue.size();
}

bool EventDispatcher::queue_empty() const {
    std::shared_lock lock(impl_->mutex);
    return impl_->event_queue.empty();
}

// ============================================================================
// Event History
// ============================================================================

void EventDispatcher::enable_history(bool enable) {
    std::unique_lock lock(impl_->mutex);
    impl_->history_enabled = enable;

    if (!enable) {
        impl_->event_history.clear();
    }
}

bool EventDispatcher::history_enabled() const {
    std::shared_lock lock(impl_->mutex);
    return impl_->history_enabled;
}

const std::vector<Event>& EventDispatcher::get_history() const {
    // Note: This returns a reference, so caller should ensure thread safety
    return impl_->event_history;
}

void EventDispatcher::clear_history() {
    std::unique_lock lock(impl_->mutex);
    impl_->event_history.clear();
}

std::vector<Event> EventDispatcher::query_history(EventType type) const {
    std::shared_lock lock(impl_->mutex);

    std::vector<Event> result;
    for (const auto& event : impl_->event_history) {
        if (event.type() == type) {
            result.push_back(event);
        }
    }

    return result;
}

std::vector<Event> EventDispatcher::query_history(EventCategory category) const {
    std::shared_lock lock(impl_->mutex);

    std::vector<Event> result;
    for (const auto& event : impl_->event_history) {
        if (has_category(event.category(), category)) {
            result.push_back(event);
        }
    }

    return result;
}

std::vector<Event> EventDispatcher::query_history(Real start_time, Real end_time) const {
    std::shared_lock lock(impl_->mutex);

    std::vector<Event> result;
    for (const auto& event : impl_->event_history) {
        if (event.timestamp() >= start_time && event.timestamp() <= end_time) {
            result.push_back(event);
        }
    }

    return result;
}

// ============================================================================
// Statistics and Debugging
// ============================================================================

const EventStatistics& EventDispatcher::get_statistics() const {
    return impl_->stats;
}

void EventDispatcher::reset_statistics() {
    std::unique_lock lock(impl_->mutex);
    impl_->stats = EventStatistics{};
}

const EventDispatcherConfig& EventDispatcher::config() const {
    return impl_->config;
}

std::vector<std::string> EventDispatcher::get_handler_names() const {
    std::shared_lock lock(impl_->mutex);

    std::vector<std::string> names;
    names.reserve(impl_->handlers.size());

    for (const auto& handler : impl_->handlers) {
        names.push_back(handler.name);
    }

    return names;
}

bool EventDispatcher::is_dispatching() const {
    std::shared_lock lock(impl_->mutex);
    return impl_->is_dispatching;
}

// ============================================================================
// Global Event Dispatcher
// ============================================================================

namespace {
    std::unique_ptr<EventDispatcher> g_global_dispatcher;
    std::once_flag g_dispatcher_init_flag;
}

EventDispatcher& get_global_dispatcher() {
    std::call_once(g_dispatcher_init_flag, []() {
        g_global_dispatcher = std::make_unique<EventDispatcher>();
    });
    return *g_global_dispatcher;
}

void set_global_dispatcher(std::unique_ptr<EventDispatcher> dispatcher) {
    g_global_dispatcher = std::move(dispatcher);
}

// ============================================================================
// EventSubscription RAII Guard
// ============================================================================

EventSubscription::EventSubscription(EventDispatcher& dispatcher, EventType type,
                                     EventHandler handler, const std::string& name)
    : dispatcher_(&dispatcher)
    , handler_id_(dispatcher.subscribe(type, std::move(handler), name)) {
}

EventSubscription::EventSubscription(EventDispatcher& dispatcher, EventCategory category,
                                     EventHandler handler, const std::string& name)
    : dispatcher_(&dispatcher)
    , handler_id_(dispatcher.subscribe_category(category, std::move(handler), name)) {
}

EventSubscription::~EventSubscription() {
    if (dispatcher_ && handler_id_ != INVALID_HANDLER_ID) {
        dispatcher_->unsubscribe(handler_id_);
    }
}

EventSubscription::EventSubscription(EventSubscription&& other) noexcept
    : dispatcher_(other.dispatcher_)
    , handler_id_(other.handler_id_) {
    other.dispatcher_ = nullptr;
    other.handler_id_ = INVALID_HANDLER_ID;
}

EventSubscription& EventSubscription::operator=(EventSubscription&& other) noexcept {
    if (this != &other) {
        if (dispatcher_ && handler_id_ != INVALID_HANDLER_ID) {
            dispatcher_->unsubscribe(handler_id_);
        }

        dispatcher_ = other.dispatcher_;
        handler_id_ = other.handler_id_;

        other.dispatcher_ = nullptr;
        other.handler_id_ = INVALID_HANDLER_ID;
    }
    return *this;
}

EventHandlerId EventSubscription::release() {
    EventHandlerId id = handler_id_;
    dispatcher_ = nullptr;
    handler_id_ = INVALID_HANDLER_ID;
    return id;
}

bool EventSubscription::valid() const {
    return dispatcher_ != nullptr && handler_id_ != INVALID_HANDLER_ID;
}

EventHandlerId EventSubscription::id() const {
    return handler_id_;
}

} // namespace jaguar::events
