#pragma once
/**
 * @file thread_pool.h
 * @brief Work-stealing thread pool for parallel physics computation
 *
 * Provides a high-performance thread pool with per-thread work queues and
 * work-stealing capabilities for optimal load balancing in physics simulations.
 */

#include "jaguar/core/types.h"
#include <thread>
#include <atomic>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <vector>
#include <random>

namespace jaguar::core {

// ============================================================================
// Lock-free Work-Stealing Deque
// ============================================================================

/**
 * @brief Thread-safe work-stealing deque (Chase-Lev algorithm)
 *
 * Allows the owner thread to push/pop from the bottom (LIFO)
 * and other threads to steal from the top (FIFO).
 */
template<typename T>
class WorkStealingDeque {
public:
    static constexpr SizeT INITIAL_CAPACITY = 1024;

    WorkStealingDeque()
        : buffer_(new CircularBuffer(INITIAL_CAPACITY))
        , top_(0)
        , bottom_(0)
    {}

    ~WorkStealingDeque() {
        delete buffer_.load(std::memory_order_relaxed);
    }

    // Non-copyable, non-moveable
    WorkStealingDeque(const WorkStealingDeque&) = delete;
    WorkStealingDeque& operator=(const WorkStealingDeque&) = delete;

    /**
     * @brief Push a task to the bottom (called by owner thread only)
     */
    void push(T item) {
        Int64 b = bottom_.load(std::memory_order_relaxed);
        Int64 t = top_.load(std::memory_order_acquire);
        CircularBuffer* buf = buffer_.load(std::memory_order_relaxed);

        if (b - t > static_cast<Int64>(buf->capacity()) - 1) {
            // Buffer full, need to grow
            buf = buf->grow(t, b);
            buffer_.store(buf, std::memory_order_relaxed);
        }

        buf->store(b, std::move(item));
        std::atomic_thread_fence(std::memory_order_release);
        bottom_.store(b + 1, std::memory_order_relaxed);
    }

    /**
     * @brief Pop a task from the bottom (called by owner thread only)
     * @return Optional containing the task if available
     */
    std::optional<T> pop() {
        Int64 b = bottom_.load(std::memory_order_relaxed) - 1;
        CircularBuffer* buf = buffer_.load(std::memory_order_relaxed);
        bottom_.store(b, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_seq_cst);

        Int64 t = top_.load(std::memory_order_relaxed);

        if (t <= b) {
            // Non-empty queue
            T item = buf->load(b);

            if (t == b) {
                // Last element - compete with stealers
                if (!top_.compare_exchange_strong(t, t + 1,
                        std::memory_order_seq_cst, std::memory_order_relaxed)) {
                    // Lost race to stealer
                    bottom_.store(b + 1, std::memory_order_relaxed);
                    return std::nullopt;
                }
                bottom_.store(b + 1, std::memory_order_relaxed);
            }

            return item;
        } else {
            // Empty queue
            bottom_.store(b + 1, std::memory_order_relaxed);
            return std::nullopt;
        }
    }

    /**
     * @brief Steal a task from the top (called by other threads)
     * @return Optional containing the stolen task if available
     */
    std::optional<T> steal() {
        Int64 t = top_.load(std::memory_order_acquire);
        std::atomic_thread_fence(std::memory_order_seq_cst);
        Int64 b = bottom_.load(std::memory_order_acquire);

        if (t < b) {
            CircularBuffer* buf = buffer_.load(std::memory_order_consume);
            T item = buf->load(t);

            if (!top_.compare_exchange_strong(t, t + 1,
                    std::memory_order_seq_cst, std::memory_order_relaxed)) {
                // Failed to steal (concurrent pop or steal)
                return std::nullopt;
            }

            return item;
        }

        return std::nullopt;
    }

    /**
     * @brief Check if the deque is empty
     */
    bool empty() const {
        Int64 b = bottom_.load(std::memory_order_relaxed);
        Int64 t = top_.load(std::memory_order_relaxed);
        return b <= t;
    }

    /**
     * @brief Get approximate size (may be inaccurate due to concurrent access)
     */
    SizeT size() const {
        Int64 b = bottom_.load(std::memory_order_relaxed);
        Int64 t = top_.load(std::memory_order_relaxed);
        return static_cast<SizeT>(std::max(Int64(0), b - t));
    }

private:
    /**
     * @brief Circular buffer for storing tasks
     */
    struct CircularBuffer {
        explicit CircularBuffer(SizeT cap)
            : capacity_(cap)
            , mask_(cap - 1)
            , data_(new T[cap])
        {}

        ~CircularBuffer() {
            delete[] data_;
        }

        SizeT capacity() const { return capacity_; }

        void store(Int64 index, T item) {
            data_[static_cast<SizeT>(index) & mask_] = std::move(item);
        }

        T load(Int64 index) const {
            return data_[static_cast<SizeT>(index) & mask_];
        }

        CircularBuffer* grow(Int64 top, Int64 bottom) {
            CircularBuffer* new_buf = new CircularBuffer(capacity_ * 2);
            for (Int64 i = top; i < bottom; ++i) {
                new_buf->store(i, load(i));
            }
            return new_buf;
        }

    private:
        SizeT capacity_;
        SizeT mask_;
        T* data_;
    };

    std::atomic<CircularBuffer*> buffer_;
    std::atomic<Int64> top_;
    std::atomic<Int64> bottom_;
};

// ============================================================================
// Thread Pool
// ============================================================================

/**
 * @brief Work-stealing thread pool for parallel physics computation
 *
 * Features:
 * - Per-thread work queues for cache efficiency
 * - Work stealing for automatic load balancing
 * - parallel_for() for easy data-parallel operations
 * - Futures support for arbitrary async tasks
 */
class ThreadPool {
public:
    /**
     * @brief Construct a thread pool
     * @param num_threads Number of worker threads (0 = hardware concurrency)
     */
    explicit ThreadPool(SizeT num_threads = 0);

    /**
     * @brief Destructor - waits for all tasks to complete
     */
    ~ThreadPool();

    // Non-copyable, non-moveable
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    /**
     * @brief Submit a task and get a future for the result
     * @tparam F Callable type
     * @tparam Args Argument types
     * @param f Function to execute
     * @param args Arguments to pass to the function
     * @return Future containing the result
     */
    template<typename F, typename... Args>
    auto submit(F&& f, Args&&... args)
        -> std::future<std::invoke_result_t<F, Args...>>
    {
        using ReturnType = std::invoke_result_t<F, Args...>;

        auto task = std::make_shared<std::packaged_task<ReturnType()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );

        std::future<ReturnType> result = task->get_future();

        auto wrapper = [task]() { (*task)(); };

        // Try to push to a specific queue based on current thread
        SizeT queue_idx = get_queue_index();
        queues_[queue_idx]->push(std::move(wrapper));

        // Wake up a worker
        cv_.notify_one();

        return result;
    }

    /**
     * @brief Execute a parallel for loop
     * @param start Start index (inclusive)
     * @param end End index (exclusive)
     * @param body Function to execute for each index
     * @param grain_size Minimum iterations per task (0 = auto)
     *
     * The body function receives the loop index as parameter.
     */
    void parallel_for(SizeT start, SizeT end,
                      const std::function<void(SizeT)>& body,
                      SizeT grain_size = 0);

    /**
     * @brief Execute a parallel for loop with range-based body
     * @param start Start index (inclusive)
     * @param end End index (exclusive)
     * @param body Function receiving (start, end) range
     * @param grain_size Minimum iterations per task (0 = auto)
     *
     * More efficient for operations that benefit from processing
     * multiple elements together (e.g., SIMD operations).
     */
    void parallel_for_range(SizeT start, SizeT end,
                           const std::function<void(SizeT, SizeT)>& body,
                           SizeT grain_size = 0);

    /**
     * @brief Get the number of worker threads
     */
    SizeT num_threads() const { return workers_.size(); }

    /**
     * @brief Get the index of the current thread (0 to num_threads-1)
     * @return Thread index, or num_threads if called from non-worker thread
     */
    SizeT current_thread_index() const;

    /**
     * @brief Shutdown the pool and wait for all tasks to complete
     */
    void shutdown();

    /**
     * @brief Check if the pool has been shut down
     */
    bool is_shutdown() const { return stop_.load(std::memory_order_acquire); }

    /**
     * @brief Wait for all pending tasks to complete
     */
    void wait_all();

    /**
     * @brief Get the global thread pool instance
     * @param num_threads Number of threads (only used on first call)
     */
    static ThreadPool& get_global(SizeT num_threads = 0);

private:
    using Task = std::function<void()>;
    using WorkQueue = WorkStealingDeque<Task>;

    /**
     * @brief Worker thread function
     */
    void worker_thread(SizeT id);

    /**
     * @brief Try to steal work from another thread's queue
     */
    bool try_steal(SizeT thief_id, Task& task);

    /**
     * @brief Get queue index for the current thread
     */
    SizeT get_queue_index() const;

    /**
     * @brief Execute a task or steal work
     */
    bool execute_task(SizeT worker_id);

    std::vector<std::thread> workers_;
    std::vector<std::unique_ptr<WorkQueue>> queues_;
    std::atomic<bool> stop_{false};
    std::atomic<SizeT> active_tasks_{0};

    std::mutex cv_mutex_;
    std::condition_variable cv_;
    std::condition_variable wait_cv_;

    // Thread-local storage for thread identification
    static thread_local SizeT tl_thread_id_;
    static thread_local bool tl_in_pool_;
};

// ============================================================================
// Parallel Algorithms
// ============================================================================

/**
 * @brief Parallel reduce operation
 * @tparam Iterator Iterator type
 * @tparam T Value type
 * @tparam BinaryOp Binary operation type
 * @param pool Thread pool to use
 * @param first Start iterator
 * @param last End iterator
 * @param init Initial value
 * @param op Binary reduction operation
 * @return Reduced value
 */
template<typename Iterator, typename T, typename BinaryOp>
T parallel_reduce(ThreadPool& pool,
                  Iterator first, Iterator last,
                  T init, BinaryOp op)
{
    const SizeT size = std::distance(first, last);
    const SizeT num_threads = pool.num_threads();

    if (size == 0) return init;
    if (size < num_threads * 16 || num_threads == 0) {
        // Too small for parallel execution
        return std::accumulate(first, last, init, op);
    }

    std::vector<std::future<T>> futures;
    futures.reserve(num_threads);

    const SizeT chunk_size = (size + num_threads - 1) / num_threads;

    for (SizeT i = 0; i < num_threads; ++i) {
        SizeT chunk_start = i * chunk_size;
        SizeT chunk_end = std::min(chunk_start + chunk_size, size);

        if (chunk_start >= size) break;

        futures.push_back(pool.submit([=, &op]() {
            auto chunk_first = first + chunk_start;
            auto chunk_last = first + chunk_end;
            return std::accumulate(chunk_first, chunk_last, T{}, op);
        }));
    }

    T result = init;
    for (auto& future : futures) {
        result = op(result, future.get());
    }

    return result;
}

/**
 * @brief Parallel transform operation
 * @tparam InputIt Input iterator type
 * @tparam OutputIt Output iterator type
 * @tparam UnaryOp Unary operation type
 * @param pool Thread pool to use
 * @param first Start iterator
 * @param last End iterator
 * @param d_first Output start iterator
 * @param op Transformation operation
 */
template<typename InputIt, typename OutputIt, typename UnaryOp>
void parallel_transform(ThreadPool& pool,
                        InputIt first, InputIt last,
                        OutputIt d_first, UnaryOp op)
{
    const SizeT size = std::distance(first, last);

    pool.parallel_for(0, size, [&](SizeT i) {
        *(d_first + i) = op(*(first + i));
    });
}

} // namespace jaguar::core
