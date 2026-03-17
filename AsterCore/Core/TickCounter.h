// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

// Include for __rdtsc
#if defined(ACPH_PLATFORM_WINDOWS)
	#include <intrin.h>
#elif defined(ACPH_CPU_X86) && defined(ACPH_COMPILER_GCC)
	#include <x86intrin.h>
#elif defined(ACPH_CPU_E2K)
	#include <x86intrin.h>
#elif defined(ACPH_CPU_LOONGARCH)
	#include <larchintrin.h>
#endif

ACPH_NAMESPACE_BEGIN

#if defined(ACPH_PLATFORM_WINDOWS_UWP) || (defined(ACPH_PLATFORM_WINDOWS) && defined(ACPH_CPU_ARM))

/// Functionality to get the processors cycle counter
uint64 GetProcessorTickCount(); // Not inline to avoid having to include Windows.h

#else

/// Functionality to get the processors cycle counter
ACPH_INLINE uint64 GetProcessorTickCount()
{
#if defined(ACPH_PLATFORM_BLUE)
	return ACPH_PLATFORM_BLUE_GET_TICKS();
#elif defined(ACPH_CPU_X86)
	return __rdtsc();
#elif defined(ACPH_CPU_E2K)
	return __rdtsc();
#elif defined(ACPH_CPU_ARM) && defined(ACPH_USE_NEON)
	uint64 val;
	asm volatile("mrs %0, cntvct_el0" : "=r" (val));
	return val;
#elif defined(ACPH_CPU_LOONGARCH)
	#if ACPH_CPU_ADDRESS_BITS == 64
		__drdtime_t t = __rdtime_d();
		return t.dvalue;
	#else
		__rdtime_t h = __rdtimeh_w();
		__rdtime_t l = __rdtimel_w();
		return ((uint64)h.value << 32) + l.value;
	#endif
#elif defined(ACPH_CPU_ARM) || defined(ACPH_CPU_RISCV) || defined(ACPH_CPU_WASM) || defined(ACPH_CPU_PPC)
	return 0; // Not supported
#else
	#error Undefined
#endif
}

#endif // ACPH_PLATFORM_WINDOWS_UWP || (ACPH_PLATFORM_WINDOWS && ACPH_CPU_ARM)

ACPH_NAMESPACE_END
