// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

ACPH_NAMESPACE_BEGIN

#ifndef ACPH_DISABLE_CUSTOM_ALLOCATOR

// Normal memory allocation, must be at least 8 byte aligned on 32 bit platform and 16 byte aligned on 64 bit platform
using AllocateFunction = void *(*)(size_t inSize);
using ReallocateFunction = void *(*)(void *inBlock, size_t inOldSize, size_t inNewSize);
using FreeFunction = void (*)(void *inBlock);

// Aligned memory allocation
using AlignedAllocateFunction = void *(*)(size_t inSize, size_t inAlignment);
using AlignedFreeFunction = void (*)(void *inBlock);

// User defined allocation / free functions
ACPH_EXPORT extern AllocateFunction Allocate;
ACPH_EXPORT extern ReallocateFunction Reallocate;
ACPH_EXPORT extern FreeFunction Free;
ACPH_EXPORT extern AlignedAllocateFunction AlignedAllocate;
ACPH_EXPORT extern AlignedFreeFunction AlignedFree;

/// Register platform default allocation / free functions
ACPH_EXPORT void RegisterDefaultAllocator();

// 32-bit MinGW g++ doesn't call the correct overload for the new operator when a type is 16 bytes aligned.
// It uses the non-aligned version, which on 32 bit platforms usually returns an 8 byte aligned block.
// We therefore default to 16 byte aligned allocations when the regular new operator is used.
// See: https://github.com/godotengine/godot/issues/105455#issuecomment-2824311547
#if defined(ACPH_COMPILER_MINGW) && ACPH_CPU_ADDRESS_BITS == 32
	#define ACPH_INTERNAL_DEFAULT_ALLOCATE(size) ACPH::AlignedAllocate(size, 16)
	#define ACPH_INTERNAL_DEFAULT_FREE(pointer) ACPH::AlignedFree(pointer)
#else
	#define ACPH_INTERNAL_DEFAULT_ALLOCATE(size) ACPH::Allocate(size)
	#define ACPH_INTERNAL_DEFAULT_FREE(pointer) ACPH::Free(pointer)
#endif

/// Macro to override the new and delete functions
#define ACPH_OVERRIDE_NEW_DELETE \
	ACPH_INLINE void *operator new (size_t inCount)												{ return ACPH_INTERNAL_DEFAULT_ALLOCATE(inCount); } \
	ACPH_INLINE void operator delete (void *inPointer) noexcept									{ ACPH_INTERNAL_DEFAULT_FREE(inPointer); } \
	ACPH_INLINE void *operator new[] (size_t inCount)											{ return ACPH_INTERNAL_DEFAULT_ALLOCATE(inCount); } \
	ACPH_INLINE void operator delete[] (void *inPointer) noexcept								{ ACPH_INTERNAL_DEFAULT_FREE(inPointer); } \
	ACPH_INLINE void *operator new (size_t inCount, std::align_val_t inAlignment)				{ return ACPH::AlignedAllocate(inCount, static_cast<size_t>(inAlignment)); } \
	ACPH_INLINE void operator delete (void *inPointer, [[maybe_unused]] std::align_val_t inAlignment) noexcept	{ ACPH::AlignedFree(inPointer); } \
	ACPH_INLINE void *operator new[] (size_t inCount, std::align_val_t inAlignment)				{ return ACPH::AlignedAllocate(inCount, static_cast<size_t>(inAlignment)); } \
	ACPH_INLINE void operator delete[] (void *inPointer, [[maybe_unused]] std::align_val_t inAlignment) noexcept	{ ACPH::AlignedFree(inPointer); } \
	ACPH_INLINE void *operator new ([[maybe_unused]] size_t inCount, void *inPointer) noexcept	{ return inPointer; } \
	ACPH_INLINE void operator delete ([[maybe_unused]] void *inPointer, [[maybe_unused]] void *inPlace) noexcept { /* Do nothing */ } \
	ACPH_INLINE void *operator new[] ([[maybe_unused]] size_t inCount, void *inPointer) noexcept	{ return inPointer; } \
	ACPH_INLINE void operator delete[] ([[maybe_unused]] void *inPointer, [[maybe_unused]] void *inPlace) noexcept { /* Do nothing */ }

#else

// Directly define the allocation functions
ACPH_EXPORT void *Allocate(size_t inSize);
ACPH_EXPORT void *Reallocate(void *inBlock, size_t inOldSize, size_t inNewSize);
ACPH_EXPORT void Free(void *inBlock);
ACPH_EXPORT void *AlignedAllocate(size_t inSize, size_t inAlignment);
ACPH_EXPORT void AlignedFree(void *inBlock);

// Don't implement allocator registering
inline void RegisterDefaultAllocator() { }

// Don't override new/delete
#define ACPH_OVERRIDE_NEW_DELETE

#endif // !ACPH_DISABLE_CUSTOM_ALLOCATOR

ACPH_NAMESPACE_END
