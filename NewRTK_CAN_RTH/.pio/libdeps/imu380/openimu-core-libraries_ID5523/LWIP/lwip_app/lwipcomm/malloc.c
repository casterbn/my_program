#include "malloc.h"

uint8_t mem1base[MEM1_MAX_SIZE]; 

uint32_t mem1mapbase[MEM1_ALLOC_TABLE_SIZE]; 

const uint32_t memtblsize[SRAMBANK] = {MEM1_ALLOC_TABLE_SIZE};
const uint32_t memblksize[SRAMBANK] = {MEM1_BLOCK_SIZE};
const uint32_t memsize[SRAMBANK] = {MEM1_MAX_SIZE};

struct _m_mallco_dev mallco_dev = {
    my_mem_init,
    my_mem_perused,
    mem1base,
    mem1mapbase,
    0};

void mymemcpy(void *des, void *src, uint32_t n)
{
    uint8_t *xdes = des;
    uint8_t *xsrc = src;
    while (n--)
        *xdes++ = *xsrc++;
}

void mymemset(void *s, uint8_t c, uint32_t count)
{
    uint8_t *xs = s;
    while (count--)
        *xs++ = c;
}

void my_mem_init(uint8_t memx)
{
    mymemset(mallco_dev.memmap[memx], 0, memtblsize[memx] * 4);
    mallco_dev.memrdy[memx] = 1;
}

uint16_t my_mem_perused(uint8_t memx)
{
    uint32_t used = 0;
    uint32_t i;
    for (i = 0; i < memtblsize[memx]; i++)
    {
        if (mallco_dev.memmap[memx][i])
            used++;
    }
    return (used * 1000) / (memtblsize[memx]);
}

uint32_t my_mem_malloc(uint8_t memx, uint32_t size)
{
    signed long offset = 0;
    uint32_t nmemb;
    uint32_t cmemb = 0;
    uint32_t i;
    if (!mallco_dev.memrdy[memx])
        mallco_dev.init(memx);
    if (size == 0)
        return 0XFFFFFFFF;
    nmemb = size / memblksize[memx];
    if (size % memblksize[memx])
        nmemb++;
    for (offset = memtblsize[memx] - 1; offset >= 0; offset--)
    {
        if (!mallco_dev.memmap[memx][offset])
            cmemb++;
        else
            cmemb = 0;
        if (cmemb == nmemb)
        {
            for (i = 0; i < nmemb; i++)
            {
                mallco_dev.memmap[memx][offset + i] = nmemb;
            }
            return (offset * memblksize[memx]);
        }
    }
    return 0XFFFFFFFF;
}

uint8_t my_mem_free(uint8_t memx, uint32_t offset)
{
    int i;
    if (!mallco_dev.memrdy[memx])
    {
        mallco_dev.init(memx);
        return 1;
    }
    if (offset < memsize[memx])
    {
        int index = offset / memblksize[memx];
        int nmemb = mallco_dev.memmap[memx][index];
        for (i = 0; i < nmemb; i++)
        {
            mallco_dev.memmap[memx][index + i] = 0;
        }
        return 0;
    }
    else
    {
        return 2;
    }
}

void myfree(uint8_t memx, void *ptr)
{
    uint32_t offset;
    if (ptr == NULL)
        return;
    offset = (uint32_t)ptr - (uint32_t)mallco_dev.membase[memx];
    my_mem_free(memx, offset);
}

void *mymalloc(uint8_t memx, uint32_t size)
{
    uint32_t offset;
    offset = my_mem_malloc(memx, size);
    if (offset == 0XFFFFFFFF)
        return NULL;
    else
        return (void *)((uint32_t)mallco_dev.membase[memx] + offset);
}

void *myrealloc(uint8_t memx, void *ptr, uint32_t size)
{
    uint32_t offset;
    offset = my_mem_malloc(memx, size);
    if (offset == 0XFFFFFFFF)
    {
        return NULL;
    }
    else
    {
        mymemcpy((void *)((uint32_t)mallco_dev.membase[memx] + offset), ptr, size);
        myfree(memx, ptr);
        return (void *)((uint32_t)mallco_dev.membase[memx] + offset);
    }
}
