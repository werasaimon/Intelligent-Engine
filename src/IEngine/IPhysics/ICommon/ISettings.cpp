
//Libries
#include "IMaths.h"
#include "ISettings.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>


namespace IPhysics
{

u32 allocCalls = 0;
u32 maxAllocCalls = 0;

void* IAlloc(u32 size)
{
    ++allocCalls;
    maxAllocCalls = IMax(maxAllocCalls, allocCalls);
    return malloc(size);
}

void IFree(void* block)
{
    return free(block);
}


void *IRealloc(void* block , u32 size)
{
    return realloc(block,size);
}

}
