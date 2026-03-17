// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#if defined(ACPH_PLATFORM_WINDOWS) && defined(_DEBUG) && !defined(ACPH_DISABLE_CUSTOM_ALLOCATOR) && !defined(ACPH_COMPILER_MINGW)

/// Enable this define to signal that the custom memory hook is enabled
#define ACPH_CUSTOM_MEMORY_HOOK_ENABLED

/// Register hook that detects allocations that aren't made through the custom allocator
void RegisterCustomMemoryHook();

/// Enable the custom memory hook to detect allocations not made through the custom allocator
void EnableCustomMemoryHook(bool inEnable);

/// Check if the hook is currently checking allocations
bool IsCustomMemoryHookEnabled();

#else

inline void RegisterCustomMemoryHook() { RegisterDefaultAllocator(); }

#endif // _DEBUG && !ACPH_DISABLE_CUSTOM_ALLOCATOR && !ACPH_COMPILER_MINGW

/// Struct that, when put on the stack, temporarily disables checking that all allocations go through the custom memory allocator
struct DisableCustomMemoryHook
{
	DisableCustomMemoryHook();
	~DisableCustomMemoryHook();
};
