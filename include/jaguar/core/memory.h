#pragma once
/**
 * @file memory.h
 * @brief Memory management utilities for JaguarEngine
 *
 * Provides aligned memory allocation, pool allocators, and SIMD-friendly
 * containers for high-performance physics simulation.
 */

#include "jaguar/core/types.h"
#include <memory>
#include <vector>
#include <cstdlib>
#include <new>
#include <cassert>

namespace jaguar {

// ============================================================================
// Aligned Memory Allocation
// ============================================================================

namespace aligned {

/**
 * @brief Allocate aligned memory
 * @param size Number of bytes to allocate
 * @param alignment Alignment boundary (must be power of 2)
 * @return Pointer to aligned memory, or nullptr on failure
 */
inline void* allocate(SizeT size, SizeT alignment = 64) {
    assert((alignment & (alignment - 1)) == 0 && "Alignment must be power of 2");
#ifdef _MSC_VER
    return _aligned_malloc(size, alignment);
#else
    void* ptr = nullptr;
    if (posix_memalign(&ptr, alignment, size) != 0) {
        return nullptr;
    }
    return ptr;
#endif
}

/**
 * @brief Deallocate aligned memory
 */
inline void deallocate(void* ptr) {
    if (ptr) {
#ifdef _MSC_VER
        _aligned_free(ptr);
#else
        std::free(ptr);
#endif
    }
}

/**
 * @brief Allocate an array with alignment
 */
template<typename T>
T* allocate_array(SizeT count, SizeT alignment = 64) {
    void* ptr = allocate(count * sizeof(T), alignment);
    if (!ptr) return nullptr;
    // Default-construct elements
    T* arr = static_cast<T*>(ptr);
    for (SizeT i = 0; i < count; ++i) {
        new (&arr[i]) T();
    }
    return arr;
}

/**
 * @brief Deallocate an array allocated with allocate_array
 */
template<typename T>
void deallocate_array(T* arr, SizeT count) {
    if (arr) {
        // Destruct elements
        for (SizeT i = 0; i < count; ++i) {
            arr[i].~T();
        }
        deallocate(arr);
    }
}

} // namespace aligned

// ============================================================================
// Aligned Vector (SIMD-friendly container)
// ============================================================================

/**
 * @brief STL-compatible vector with cache-line alignment
 *
 * This container ensures all data is aligned to 64 bytes (typical cache line)
 * for optimal SIMD performance and cache utilization.
 */
template<typename T, SizeT Alignment = 64>
class AlignedVector {
public:
    using value_type = T;
    using size_type = SizeT;
    using iterator = T*;
    using const_iterator = const T*;

    AlignedVector() = default;

    explicit AlignedVector(size_type count) {
        resize(count);
    }

    AlignedVector(size_type count, const T& value) {
        resize(count);
        for (size_type i = 0; i < count; ++i) {
            data_[i] = value;
        }
    }

    ~AlignedVector() {
        clear();
        if (data_) {
            aligned::deallocate(data_);
        }
    }

    // Move operations
    AlignedVector(AlignedVector&& other) noexcept
        : data_(other.data_)
        , size_(other.size_)
        , capacity_(other.capacity_)
    {
        other.data_ = nullptr;
        other.size_ = 0;
        other.capacity_ = 0;
    }

    AlignedVector& operator=(AlignedVector&& other) noexcept {
        if (this != &other) {
            clear();
            if (data_) aligned::deallocate(data_);

            data_ = other.data_;
            size_ = other.size_;
            capacity_ = other.capacity_;

            other.data_ = nullptr;
            other.size_ = 0;
            other.capacity_ = 0;
        }
        return *this;
    }

    // Copy operations
    AlignedVector(const AlignedVector& other) {
        resize(other.size_);
        for (size_type i = 0; i < other.size_; ++i) {
            data_[i] = other.data_[i];
        }
    }

    AlignedVector& operator=(const AlignedVector& other) {
        if (this != &other) {
            resize(other.size_);
            for (size_type i = 0; i < other.size_; ++i) {
                data_[i] = other.data_[i];
            }
        }
        return *this;
    }

    // Element access
    T& operator[](size_type i) { return data_[i]; }
    const T& operator[](size_type i) const { return data_[i]; }

    T& at(size_type i) {
        if (i >= size_) throw std::out_of_range("AlignedVector index out of range");
        return data_[i];
    }

    const T& at(size_type i) const {
        if (i >= size_) throw std::out_of_range("AlignedVector index out of range");
        return data_[i];
    }

    T& front() { return data_[0]; }
    const T& front() const { return data_[0]; }

    T& back() { return data_[size_ - 1]; }
    const T& back() const { return data_[size_ - 1]; }

    T* data() noexcept { return data_; }
    const T* data() const noexcept { return data_; }

    // Iterators
    iterator begin() noexcept { return data_; }
    const_iterator begin() const noexcept { return data_; }
    const_iterator cbegin() const noexcept { return data_; }

    iterator end() noexcept { return data_ + size_; }
    const_iterator end() const noexcept { return data_ + size_; }
    const_iterator cend() const noexcept { return data_ + size_; }

    // Capacity
    bool empty() const noexcept { return size_ == 0; }
    size_type size() const noexcept { return size_; }
    size_type capacity() const noexcept { return capacity_; }

    void reserve(size_type new_cap) {
        if (new_cap <= capacity_) return;

        T* new_data = static_cast<T*>(aligned::allocate(new_cap * sizeof(T), Alignment));
        if (!new_data) throw std::bad_alloc();

        // Move existing elements
        for (size_type i = 0; i < size_; ++i) {
            new (&new_data[i]) T(std::move(data_[i]));
            data_[i].~T();
        }

        if (data_) aligned::deallocate(data_);

        data_ = new_data;
        capacity_ = new_cap;
    }

    // Modifiers
    void clear() noexcept {
        for (size_type i = 0; i < size_; ++i) {
            data_[i].~T();
        }
        size_ = 0;
    }

    void resize(size_type count) {
        if (count > capacity_) {
            reserve(std::max(count, capacity_ * 2 + 1));
        }

        // Construct new elements
        for (size_type i = size_; i < count; ++i) {
            new (&data_[i]) T();
        }

        // Destruct excess elements
        for (size_type i = count; i < size_; ++i) {
            data_[i].~T();
        }

        size_ = count;
    }

    void resize(size_type count, const T& value) {
        if (count > capacity_) {
            reserve(std::max(count, capacity_ * 2 + 1));
        }

        // Construct new elements with value
        for (size_type i = size_; i < count; ++i) {
            new (&data_[i]) T(value);
        }

        // Destruct excess elements
        for (size_type i = count; i < size_; ++i) {
            data_[i].~T();
        }

        size_ = count;
    }

    void push_back(const T& value) {
        if (size_ >= capacity_) {
            reserve(capacity_ == 0 ? 8 : capacity_ * 2);
        }
        new (&data_[size_]) T(value);
        ++size_;
    }

    void push_back(T&& value) {
        if (size_ >= capacity_) {
            reserve(capacity_ == 0 ? 8 : capacity_ * 2);
        }
        new (&data_[size_]) T(std::move(value));
        ++size_;
    }

    template<typename... Args>
    T& emplace_back(Args&&... args) {
        if (size_ >= capacity_) {
            reserve(capacity_ == 0 ? 8 : capacity_ * 2);
        }
        new (&data_[size_]) T(std::forward<Args>(args)...);
        return data_[size_++];
    }

    void pop_back() {
        if (size_ > 0) {
            --size_;
            data_[size_].~T();
        }
    }

    void shrink_to_fit() {
        if (size_ < capacity_) {
            T* new_data = nullptr;
            if (size_ > 0) {
                new_data = static_cast<T*>(aligned::allocate(size_ * sizeof(T), Alignment));
                for (size_type i = 0; i < size_; ++i) {
                    new (&new_data[i]) T(std::move(data_[i]));
                    data_[i].~T();
                }
            }
            if (data_) aligned::deallocate(data_);
            data_ = new_data;
            capacity_ = size_;
        }
    }

private:
    T* data_ = nullptr;
    size_type size_ = 0;
    size_type capacity_ = 0;
};

// ============================================================================
// Pool Allocator
// ============================================================================

/**
 * @brief Fixed-size object pool allocator
 *
 * Provides fast allocation/deallocation for objects of the same type.
 * Uses free list for O(1) allocation and deallocation.
 */
template<typename T, SizeT BlockSize = 64>
class PoolAllocator {
public:
    PoolAllocator() = default;

    ~PoolAllocator() {
        clear();
    }

    // Non-copyable, moveable
    PoolAllocator(const PoolAllocator&) = delete;
    PoolAllocator& operator=(const PoolAllocator&) = delete;

    PoolAllocator(PoolAllocator&& other) noexcept
        : blocks_(std::move(other.blocks_))
        , free_list_(std::move(other.free_list_))
        , allocated_(other.allocated_)
    {
        other.allocated_ = 0;
    }

    PoolAllocator& operator=(PoolAllocator&& other) noexcept {
        if (this != &other) {
            clear();
            blocks_ = std::move(other.blocks_);
            free_list_ = std::move(other.free_list_);
            allocated_ = other.allocated_;
            other.allocated_ = 0;
        }
        return *this;
    }

    /**
     * @brief Allocate an object from the pool
     * @return Pointer to uninitialized memory for T
     */
    T* allocate() {
        if (free_list_.empty()) {
            grow();
        }

        T* ptr = free_list_.back();
        free_list_.pop_back();
        ++allocated_;
        return ptr;
    }

    /**
     * @brief Return an object to the pool
     */
    void deallocate(T* ptr) {
        if (ptr) {
            free_list_.push_back(ptr);
            --allocated_;
        }
    }

    /**
     * @brief Construct an object in the pool
     */
    template<typename... Args>
    T* construct(Args&&... args) {
        T* ptr = allocate();
        new (ptr) T(std::forward<Args>(args)...);
        return ptr;
    }

    /**
     * @brief Destroy an object and return to pool
     */
    void destroy(T* ptr) {
        if (ptr) {
            ptr->~T();
            deallocate(ptr);
        }
    }

    /**
     * @brief Clear all allocated memory
     */
    void clear() {
        blocks_.clear();
        free_list_.clear();
        allocated_ = 0;
    }

    SizeT capacity() const { return blocks_.size() * BlockSize; }
    SizeT size() const { return allocated_; }
    SizeT free_count() const { return free_list_.size(); }

private:
    void grow() {
        // Allocate new block with alignment
        void* raw = aligned::allocate(BlockSize * sizeof(T), 64);
        if (!raw) throw std::bad_alloc();

        blocks_.push_back(raw);

        // Add all slots to free list
        T* block = static_cast<T*>(raw);
        for (SizeT i = 0; i < BlockSize; ++i) {
            free_list_.push_back(&block[i]);
        }
    }

    std::vector<void*> blocks_;
    std::vector<T*> free_list_;
    SizeT allocated_{0};
};

} // namespace jaguar
