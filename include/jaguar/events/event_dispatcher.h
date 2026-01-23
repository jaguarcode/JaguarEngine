#pragma once
/**
 * @file event_dispatcher.h
 * @brief Event dispatcher and event queue management
 *
 * The EventDispatcher provides the central hub for event routing in the
 * simulation. It supports immediate and deferred dispatch, priority queuing,
 * category filtering, and handler management.
 */

#include "jaguar/events/event.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <shared_mutex>
#include <functional>

namespace jaguar::events {

// ============================================================================
// Event Dispatcher Configuration
// ============================================================================

/**
 * @brief Configuration for the event dispatcher
 */
struct EventDispatcherConfig {
    SizeT max_queue_size{10000};           ///< Maximum events in queue
    SizeT max_handlers_per_type{100};      ///< Maximum handlers per event type
    bool enable_event_history{false};      ///< Record event history
    SizeT history_size{1000};              ///< Max events in history
    bool thread_safe{true};                ///< Enable thread-safe operations
    bool allow_recursive_dispatch{false};  ///< Allow dispatching during dispatch
};

// ============================================================================
// Event Statistics
// ============================================================================

/**
 * @brief Statistics about event dispatching
 */
struct EventStatistics {
    UInt64 total_events_dispatched{0};
    UInt64 total_events_queued{0};
    UInt64 total_events_dropped{0};
    UInt64 total_handlers_called{0};
    Real average_dispatch_time_us{0.0};
    Real max_dispatch_time_us{0.0};

    // Per-type statistics
    std::unordered_map<EventType, UInt64> events_by_type;
};

// ============================================================================
// Handler Registration
// ============================================================================

/**
 * @brief Handler registration info
 */
struct HandlerInfo {
    EventHandlerId id{INVALID_HANDLER_ID};
    EventHandler handler;
    EventType type{EventType::UserDefined};
    EventCategory category_filter{EventCategory::All};
    EventPriority min_priority{EventPriority::Deferred};
    int order{0};                          ///< Handler execution order (lower = earlier)
    std::string name;                      ///< Debug name for handler
    bool enabled{true};
};

// ============================================================================
// Event Dispatcher
// ============================================================================

/**
 * @brief Central event dispatcher for the simulation
 *
 * The EventDispatcher manages event routing between simulation components.
 * It supports:
 * - Immediate dispatch for time-critical events
 * - Deferred/queued dispatch for batched processing
 * - Priority-based ordering
 * - Category and type filtering
 * - Handler management with enable/disable
 * - Thread-safe operation (optional)
 *
 * Usage:
 * @code
 * EventDispatcher dispatcher;
 *
 * // Register a handler
 * auto id = dispatcher.subscribe(EventType::CollisionEnter,
 *     [](Event& e) {
 *         auto& data = e.get_data<CollisionEventData>();
 *         // Handle collision...
 *         return true; // Continue propagation
 *     });
 *
 * // Dispatch an event
 * dispatcher.dispatch(Event::create_collision_enter(...));
 *
 * // Unsubscribe when done
 * dispatcher.unsubscribe(id);
 * @endcode
 */
class EventDispatcher {
public:
    EventDispatcher();
    explicit EventDispatcher(const EventDispatcherConfig& config);
    ~EventDispatcher();

    // Non-copyable
    EventDispatcher(const EventDispatcher&) = delete;
    EventDispatcher& operator=(const EventDispatcher&) = delete;

    // Movable
    EventDispatcher(EventDispatcher&&) noexcept;
    EventDispatcher& operator=(EventDispatcher&&) noexcept;

    // ========================================================================
    // Handler Registration
    // ========================================================================

    /**
     * @brief Subscribe to a specific event type
     *
     * @param type Event type to listen for
     * @param handler Callback function
     * @param name Optional debug name
     * @param order Execution order (lower = earlier)
     * @return Handler ID for later unsubscription
     */
    EventHandlerId subscribe(EventType type, EventHandler handler,
                            const std::string& name = "",
                            int order = 0);

    /**
     * @brief Subscribe to events by category
     *
     * @param category Event category mask
     * @param handler Callback function
     * @param name Optional debug name
     * @param order Execution order
     * @return Handler ID
     */
    EventHandlerId subscribe_category(EventCategory category, EventHandler handler,
                                      const std::string& name = "",
                                      int order = 0);

    /**
     * @brief Subscribe to all events
     *
     * @param handler Callback function
     * @param name Optional debug name
     * @param order Execution order
     * @return Handler ID
     */
    EventHandlerId subscribe_all(EventHandler handler,
                                 const std::string& name = "",
                                 int order = 0);

    /**
     * @brief Unsubscribe a handler
     *
     * @param id Handler ID from subscribe
     * @return true if handler was found and removed
     */
    bool unsubscribe(EventHandlerId id);

    /**
     * @brief Enable/disable a handler
     *
     * @param id Handler ID
     * @param enabled true to enable, false to disable
     */
    void set_handler_enabled(EventHandlerId id, bool enabled);

    /**
     * @brief Check if handler is enabled
     */
    bool is_handler_enabled(EventHandlerId id) const;

    /**
     * @brief Get number of registered handlers for a type
     */
    SizeT handler_count(EventType type) const;

    /**
     * @brief Get total number of registered handlers
     */
    SizeT total_handler_count() const;

    // ========================================================================
    // Event Dispatching
    // ========================================================================

    /**
     * @brief Dispatch an event immediately
     *
     * Calls all registered handlers for this event type synchronously.
     * Handlers are called in order until one consumes the event or all
     * handlers have been called.
     *
     * @param event Event to dispatch (may be modified by handlers)
     */
    void dispatch(Event& event);

    /**
     * @brief Dispatch an event immediately (move version)
     */
    void dispatch(Event&& event);

    /**
     * @brief Queue an event for deferred dispatch
     *
     * Event will be dispatched during the next flush_queue() call,
     * ordered by priority.
     *
     * @param event Event to queue (passed by value, use std::move for efficiency)
     * @return true if event was queued, false if queue is full
     */
    bool queue(Event event);

    /**
     * @brief Process all queued events
     *
     * Dispatches events in priority order (Immediate first, Deferred last).
     * Within same priority, FIFO order is maintained.
     *
     * @return Number of events processed
     */
    SizeT flush_queue();

    /**
     * @brief Process queued events up to a time limit
     *
     * @param max_time_ms Maximum time to spend processing (milliseconds)
     * @return Number of events processed
     */
    SizeT flush_queue_timed(Real max_time_ms);

    /**
     * @brief Clear all queued events
     */
    void clear_queue();

    /**
     * @brief Get number of events in queue
     */
    SizeT queue_size() const;

    /**
     * @brief Check if queue is empty
     */
    bool queue_empty() const;

    // ========================================================================
    // Event History
    // ========================================================================

    /**
     * @brief Enable event history recording
     */
    void enable_history(bool enable);

    /**
     * @brief Check if history is enabled
     */
    bool history_enabled() const;

    /**
     * @brief Get event history
     */
    const std::vector<Event>& get_history() const;

    /**
     * @brief Clear event history
     */
    void clear_history();

    /**
     * @brief Get events from history matching criteria
     */
    std::vector<Event> query_history(EventType type) const;
    std::vector<Event> query_history(EventCategory category) const;
    std::vector<Event> query_history(Real start_time, Real end_time) const;

    // ========================================================================
    // Statistics and Debugging
    // ========================================================================

    /**
     * @brief Get event dispatch statistics
     */
    const EventStatistics& get_statistics() const;

    /**
     * @brief Reset statistics
     */
    void reset_statistics();

    /**
     * @brief Get configuration
     */
    const EventDispatcherConfig& config() const;

    /**
     * @brief Get list of registered handler names
     */
    std::vector<std::string> get_handler_names() const;

    /**
     * @brief Check if currently dispatching (for recursion detection)
     */
    bool is_dispatching() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    void dispatch_to_handlers(Event& event);
};

// ============================================================================
// Global Event Dispatcher
// ============================================================================

/**
 * @brief Get the global event dispatcher instance
 *
 * Provides a singleton-style access for convenience, though explicit
 * injection is preferred for testability.
 */
EventDispatcher& get_global_dispatcher();

/**
 * @brief Set the global event dispatcher
 *
 * Takes ownership of the provided dispatcher.
 */
void set_global_dispatcher(std::unique_ptr<EventDispatcher> dispatcher);

// ============================================================================
// RAII Subscription Guard
// ============================================================================

/**
 * @brief RAII guard for automatic handler unsubscription
 *
 * Usage:
 * @code
 * {
 *     EventSubscription sub(dispatcher, EventType::CollisionEnter, handler);
 *     // Handler is active...
 * } // Handler automatically unsubscribed
 * @endcode
 */
class EventSubscription {
public:
    EventSubscription() = default;

    EventSubscription(EventDispatcher& dispatcher, EventType type,
                     EventHandler handler, const std::string& name = "");

    EventSubscription(EventDispatcher& dispatcher, EventCategory category,
                     EventHandler handler, const std::string& name = "");

    ~EventSubscription();

    // Non-copyable
    EventSubscription(const EventSubscription&) = delete;
    EventSubscription& operator=(const EventSubscription&) = delete;

    // Movable
    EventSubscription(EventSubscription&& other) noexcept;
    EventSubscription& operator=(EventSubscription&& other) noexcept;

    /**
     * @brief Release the subscription without unsubscribing
     * @return The handler ID
     */
    EventHandlerId release();

    /**
     * @brief Check if subscription is valid
     */
    bool valid() const;

    /**
     * @brief Get the handler ID
     */
    EventHandlerId id() const;

private:
    EventDispatcher* dispatcher_{nullptr};
    EventHandlerId handler_id_{INVALID_HANDLER_ID};
};

} // namespace jaguar::events
