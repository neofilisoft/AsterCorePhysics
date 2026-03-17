// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2023 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

ACPH_NAMESPACE_BEGIN

/// Construct a string that lists the most important configuration settings
inline const char *GetConfigurationString()
{
	return ACPH_IF_SINGLE_PRECISION_ELSE("Single", "Double") " precision "
#if defined(ACPH_CPU_X86)
		"x86 "
#elif defined(ACPH_CPU_ARM)
		"ARM "
#elif defined(ACPH_CPU_RISCV)
		"RISC-V "
#elif defined(ACPH_CPU_PPC)
		"PowerPC "
	#ifdef ACPH_CPU_BIG_ENDIAN
		"(Big Endian) "
	#else
		"(Little Endian) "
	#endif
#elif defined(ACPH_CPU_LOONGARCH)
		"LoongArch "
#elif defined(ACPH_CPU_E2K)
		"E2K "
#elif defined(ACPH_CPU_WASM)
		"WASM "
#else
	#error Unknown CPU architecture
#endif
#if ACPH_CPU_ADDRESS_BITS == 64
		"64-bit "
#elif ACPH_CPU_ADDRESS_BITS == 32
		"32-bit "
#endif
		"with instructions: "
#ifdef ACPH_USE_NEON
		"NEON "
#endif
#ifdef ACPH_USE_SSE
		"SSE2 "
#endif
#ifdef ACPH_USE_SSE4_1
		"SSE4.1 "
#endif
#ifdef ACPH_USE_SSE4_2
		"SSE4.2 "
#endif
#ifdef ACPH_USE_AVX
		"AVX "
#endif
#ifdef ACPH_USE_AVX2
		"AVX2 "
#endif
#ifdef ACPH_USE_AVX512
		"AVX512 "
#endif
#ifdef ACPH_USE_F16C
		"F16C "
#endif
#ifdef ACPH_USE_LZCNT
		"LZCNT "
#endif
#ifdef ACPH_USE_TZCNT
		"TZCNT "
#endif
#ifdef ACPH_USE_FMADD
		"FMADD "
#endif
#ifdef ACPH_CROSS_PLATFORM_DETERMINISTIC
		"(Cross Platform Deterministic) "
#endif
#ifdef ACPH_FLOATING_POINT_EXCEPTIONS_ENABLED
		"(FP Exceptions) "
#endif
#ifdef ACPH_DEBUG_RENDERER
		"(Debug Renderer) "
#endif
#ifdef ACPH_PROFILE_ENABLED
		"(Profile) "
#endif
#ifdef ACPH_EXTERNAL_PROFILE
		"(External Profile) "
#endif
#if defined(ACPH_OBJECT_LAYER_BITS) && ACPH_OBJECT_LAYER_BITS == 32
		"(32-bit ObjectLayer) "
#else
		"(16-bit ObjectLayer) "
#endif
#ifdef ACPH_ENABLE_ASSERTS
		"(Assertions) "
#endif
#ifdef ACPH_OBJECT_STREAM
		"(ObjectStream) "
#endif
#ifdef ACPH_DEBUG
		"(Debug) "
#endif
#if defined(__cpp_rtti) && __cpp_rtti
		"(C++ RTTI) "
#endif
#if defined(__cpp_exceptions) && __cpp_exceptions
		"(C++ Exceptions) "
#endif
		;
}

ACPH_NAMESPACE_END
