/**
 * @file pool_allocator.cpp
 * @brief Pool allocator implementation (stub)
 */

#include "jaguar/core/memory.h"
#include <cstdlib>

namespace jaguar::core::aligned {

void* allocate(SizeT size, SizeT alignment) {
#if defined(_MSC_VER)
    return _aligned_malloc(size, alignment);
#else
    void* ptr = nullptr;
    posix_memalign(&ptr, alignment, size);
    return ptr;
#endif
}

void deallocate(void* ptr) {
#if defined(_MSC_VER)
    _aligned_free(ptr);
#else
    free(ptr);
#endif
}

} // namespace jaguar::core::aligned
