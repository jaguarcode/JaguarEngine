/**
 * @file thread_pool.cpp
 * @brief Work-stealing thread pool implementation
 *
 * Implements a high-performance work-stealing thread pool based on the
 * Chase-Lev algorithm for optimal load balancing in physics simulations.
 */

#include "jaguar/core/threading/thread_pool.h"
#include <algorithm>
#include <cassert>

namespace jaguar::core {

// Thread-local storage definitions
thread_local SizeT ThreadPool::tl_thread_id_ = 0;
thread_local bool ThreadPool::tl_in_pool_ = false;

// ============================================================================
// ThreadPool Implementation
// ============================================================================

ThreadPool::ThreadPool(SizeT num_threads)
{
    if (num_threads == 0) {
        num_threads = std::thread::hardware_concurrency();
        if (num_threads == 0) {
            num_threads = 4;  // Fallback
        }
    }

    // Create per-thread work queues
    queues_.reserve(num_threads);
    for (SizeT i = 0; i < num_threads; ++i) {
        queues_.push_back(std::make_unique<WorkQueue>());
    }

    // Create worker threads
    workers_.reserve(num_threads);
    for (SizeT i = 0; i < num_threads; ++i) {
        workers_.emplace_back(&ThreadPool::worker_thread, this, i);
    }
}

ThreadPool::~ThreadPool()
{
    shutdown();
}

void ThreadPool::shutdown()
{
    if (stop_.exchange(true, std::memory_order_acq_rel)) {
        return;  // Already stopped
    }

    // Wake up all workers
    cv_.notify_all();

    // Wait for all workers to finish
    for (auto& worker : workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}

void ThreadPool::wait_all()
{
    std::unique_lock<std::mutex> lock(cv_mutex_);
    wait_cv_.wait(lock, [this]() {
        return active_tasks_.load(std::memory_order_acquire) == 0;
    });
}

void ThreadPool::worker_thread(SizeT id)
{
    // Set thread-local identification
    tl_thread_id_ = id;
    tl_in_pool_ = true;

    // Random number generator for stealing victim selection
    std::random_device rd;
    std::mt19937 rng(rd());

    while (!stop_.load(std::memory_order_acquire)) {
        // Try to execute a task
        if (!execute_task(id)) {
            // No work found, wait for notification
            std::unique_lock<std::mutex> lock(cv_mutex_);
            cv_.wait_for(lock, std::chrono::microseconds(100), [this, id]() {
                return stop_.load(std::memory_order_acquire) ||
                       !queues_[id]->empty();
            });
        }
    }

    // Drain remaining tasks before exiting
    Task task;
    while (auto opt_task = queues_[id]->pop()) {
        try {
            (*opt_task)();
        } catch (...) {
            // Swallow exceptions in worker threads
        }
    }
}

bool ThreadPool::execute_task(SizeT worker_id)
{
    // First, try our own queue
    if (auto opt_task = queues_[worker_id]->pop()) {
        active_tasks_.fetch_add(1, std::memory_order_relaxed);
        try {
            (*opt_task)();
        } catch (...) {
            // Swallow exceptions
        }
        if (active_tasks_.fetch_sub(1, std::memory_order_release) == 1) {
            wait_cv_.notify_all();
        }
        return true;
    }

    // Try to steal from other queues
    Task stolen_task;
    if (try_steal(worker_id, stolen_task)) {
        active_tasks_.fetch_add(1, std::memory_order_relaxed);
        try {
            stolen_task();
        } catch (...) {
            // Swallow exceptions
        }
        if (active_tasks_.fetch_sub(1, std::memory_order_release) == 1) {
            wait_cv_.notify_all();
        }
        return true;
    }

    return false;
}

bool ThreadPool::try_steal(SizeT thief_id, Task& task)
{
    const SizeT num_queues = queues_.size();
    if (num_queues <= 1) return false;

    // Start from a random victim to avoid contention
    SizeT start = (thief_id + 1) % num_queues;

    for (SizeT i = 0; i < num_queues - 1; ++i) {
        SizeT victim = (start + i) % num_queues;
        if (victim == thief_id) continue;

        if (auto opt_task = queues_[victim]->steal()) {
            task = std::move(*opt_task);
            return true;
        }
    }

    return false;
}

SizeT ThreadPool::get_queue_index() const
{
    if (tl_in_pool_) {
        return tl_thread_id_;
    }
    // For external threads, distribute across queues
    static std::atomic<SizeT> counter{0};
    return counter.fetch_add(1, std::memory_order_relaxed) % queues_.size();
}

SizeT ThreadPool::current_thread_index() const
{
    if (tl_in_pool_) {
        return tl_thread_id_;
    }
    return workers_.size();  // Indicates non-worker thread
}

void ThreadPool::parallel_for(SizeT start, SizeT end,
                              const std::function<void(SizeT)>& body,
                              SizeT grain_size)
{
    if (start >= end) return;

    const SizeT total = end - start;
    const SizeT num_threads = workers_.size();

    // Auto-select grain size if not specified
    if (grain_size == 0) {
        // Aim for ~4 tasks per thread for good load balancing
        grain_size = std::max(SizeT(1), total / (num_threads * 4));
    }

    // For very small ranges, execute serially
    if (total <= grain_size || num_threads == 0) {
        for (SizeT i = start; i < end; ++i) {
            body(i);
        }
        return;
    }

    // Calculate number of chunks
    const SizeT num_chunks = (total + grain_size - 1) / grain_size;

    // Track completion
    std::atomic<SizeT> completed_chunks{0};
    std::mutex done_mutex;
    std::condition_variable done_cv;

    // Submit all chunks except one (we'll execute that ourselves)
    for (SizeT chunk = 0; chunk < num_chunks - 1; ++chunk) {
        SizeT chunk_start = start + chunk * grain_size;
        SizeT chunk_end = std::min(chunk_start + grain_size, end);

        SizeT queue_idx = chunk % queues_.size();
        queues_[queue_idx]->push([=, &body, &completed_chunks, &done_cv]() {
            for (SizeT i = chunk_start; i < chunk_end; ++i) {
                body(i);
            }
            if (completed_chunks.fetch_add(1, std::memory_order_release) ==
                num_chunks - 2) {
                done_cv.notify_all();
            }
        });
    }

    // Wake up workers
    cv_.notify_all();

    // Execute the last chunk ourselves
    SizeT last_chunk = num_chunks - 1;
    SizeT last_start = start + last_chunk * grain_size;
    for (SizeT i = last_start; i < end; ++i) {
        body(i);
    }

    // Wait for all other chunks to complete
    if (num_chunks > 1) {
        std::unique_lock<std::mutex> lock(done_mutex);
        done_cv.wait(lock, [&]() {
            return completed_chunks.load(std::memory_order_acquire) >= num_chunks - 1;
        });
    }
}

void ThreadPool::parallel_for_range(SizeT start, SizeT end,
                                    const std::function<void(SizeT, SizeT)>& body,
                                    SizeT grain_size)
{
    if (start >= end) return;

    const SizeT total = end - start;
    const SizeT num_threads = workers_.size();

    // Auto-select grain size if not specified
    if (grain_size == 0) {
        // Aim for ~4 tasks per thread for good load balancing
        grain_size = std::max(SizeT(1), total / (num_threads * 4));
    }

    // For very small ranges, execute serially
    if (total <= grain_size || num_threads == 0) {
        body(start, end);
        return;
    }

    // Calculate number of chunks
    const SizeT num_chunks = (total + grain_size - 1) / grain_size;

    // Track completion
    std::atomic<SizeT> completed_chunks{0};
    std::mutex done_mutex;
    std::condition_variable done_cv;

    // Submit all chunks except one
    for (SizeT chunk = 0; chunk < num_chunks - 1; ++chunk) {
        SizeT chunk_start = start + chunk * grain_size;
        SizeT chunk_end = std::min(chunk_start + grain_size, end);

        SizeT queue_idx = chunk % queues_.size();
        queues_[queue_idx]->push([=, &body, &completed_chunks, &done_cv]() {
            body(chunk_start, chunk_end);
            if (completed_chunks.fetch_add(1, std::memory_order_release) ==
                num_chunks - 2) {
                done_cv.notify_all();
            }
        });
    }

    // Wake up workers
    cv_.notify_all();

    // Execute the last chunk ourselves
    SizeT last_chunk = num_chunks - 1;
    SizeT last_start = start + last_chunk * grain_size;
    body(last_start, end);

    // Wait for all other chunks to complete
    if (num_chunks > 1) {
        std::unique_lock<std::mutex> lock(done_mutex);
        done_cv.wait(lock, [&]() {
            return completed_chunks.load(std::memory_order_acquire) >= num_chunks - 1;
        });
    }
}

ThreadPool& ThreadPool::get_global(SizeT num_threads)
{
    static ThreadPool global_pool(num_threads);
    return global_pool;
}

} // namespace jaguar::core
