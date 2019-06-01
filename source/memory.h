#ifndef __MEMORY__
#define __MEMORY__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

typedef struct pagemap_entry_t
{
  uint64_t pfn : 55;
  uint64_t softDirty : 1;
  uint64_t exclusiveMap : 1;
  uint64_t _reserved : 4;
  uint64_t filePageSharedAnon : 1;
  uint64_t swapped : 1;
  uint64_t present : 1;
} pagemap_entry_t;

static_assert(sizeof(pagemap_entry_t) == sizeof(uint64_t), "pagemap_entry_t must be 64 bits.");

void* memoryMapPhysical(off_t offset, size_t length);
void* memoryAllocate(size_t length);
void memoryReadPagemap(void* virtual);

#endif