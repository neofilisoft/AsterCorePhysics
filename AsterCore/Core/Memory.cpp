// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

ACPH_SUPPRESS_WARNINGS_STD_BEGIN
#include <cstdlib>
ACPH_SUPPRESS_WARNINGS_STD_END
#include <stdlib.h>

ACPH_NAMESPACE_BEGIN

#ifdef ACPH_DISABLE_CUSTOM_ALLOCATOR
	#define ACPH_ALLOC_FN(x)	x
	#define ACPH_ALLOC_SCOPE
#else
	#define ACPH_ALLOC_FN(x)	x##Impl
	#define ACPH_ALLOC_SCOPE static
#endif

ACPH_ALLOC_SCOPE void *ACPH_ALLOC_FN(Allocate)(size_t inSize)
{
	ACPH_ASSERT(inSize > 0);
	return malloc(inSize);
}

ACPH_ALLOC_SCOPE void *ACPH_ALLOC_FN(Reallocate)(void *inBlock, [[maybe_unused]] size_t inOldSize, size_t inNewSize)
{
	ACPH_ASSERT(inNewSize > 0);
	return realloc(inBlock, inNewSize);
}

ACPH_ALLOC_SCOPE void ACPH_ALLOC_FN(Free)(void *inBlock)
{
	free(inBlock);
}

ACPH_ALLOC_SCOPE void *ACPH_ALLOC_FN(AlignedAllocate)(size_t inSize, size_t inAlignment)
{
	ACPH_ASSERT(inSize > 0 && inAlignment > 0);

#if defined(ACPH_PLATFORM_WINDOWS)
	// Microsoft doesn't implement posix_memalign
	return _aligned_malloc(inSize, inAlignment);
#else
	void *block = nullptr;
	ACPH_SUPPRESS_WARNING_PUSH
	ACPH_GCC_SUPPRESS_WARNING("-Wunused-result")
	ACPH_CLANG_SUPPRESS_WARNING("-Wunused-result")
	posix_memalign(&block, inAlignment, inSize);
	ACPH_SUPPRESS_WARNING_POP
	return block;
#endif
}

ACPH_ALLOC_SCOPE void ACPH_ALLOC_FN(AlignedFree)(void *inBlock)
{
#if defined(ACPH_PLATFORM_WINDOWS)
	_aligned_free(inBlock);
#else
	free(inBlock);
#endif
}

#ifndef ACPH_DISABLE_CUSTOM_ALLOCATOR

AllocateFunction Allocate = nullptr;
ReallocateFunction Reallocate = nullptr;
FreeFunction Free = nullptr;
AlignedAllocateFunction AlignedAllocate = nullptr;
AlignedFreeFunction AlignedFree = nullptr;

void RegisterDefaultAllocator()
{
	Allocate = AllocateImpl;
	Reallocate = ReallocateImpl;
	Free = FreeImpl;
	AlignedAllocate = AlignedAllocateImpl;
	AlignedFree = AlignedFreeImpl;
}

#endif // ACPH_DISABLE_CUSTOM_ALLOCATOR

ACPH_NAMESPACE_END
