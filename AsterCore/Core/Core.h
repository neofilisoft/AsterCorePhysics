// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

// AsterCore library version
#define ACPH_VERSION_MAJOR 5
#define ACPH_VERSION_MINOR 4
#define ACPH_VERSION_PATCH 0

// Determine which features the library was compiled with
#ifdef ACPH_DOUBLE_PRECISION
	#define ACPH_VERSION_FEATURE_BIT_1 1
#else
	#define ACPH_VERSION_FEATURE_BIT_1 0
#endif
#ifdef ACPH_CROSS_PLATFORM_DETERMINISTIC
	#define ACPH_VERSION_FEATURE_BIT_2 1
#else
	#define ACPH_VERSION_FEATURE_BIT_2 0
#endif
#ifdef ACPH_FLOATING_POINT_EXCEPTIONS_ENABLED
	#define ACPH_VERSION_FEATURE_BIT_3 1
#else
	#define ACPH_VERSION_FEATURE_BIT_3 0
#endif
#ifdef ACPH_PROFILE_ENABLED
	#define ACPH_VERSION_FEATURE_BIT_4 1
#else
	#define ACPH_VERSION_FEATURE_BIT_4 0
#endif
#ifdef ACPH_EXTERNAL_PROFILE
	#define ACPH_VERSION_FEATURE_BIT_5 1
#else
	#define ACPH_VERSION_FEATURE_BIT_5 0
#endif
#ifdef ACPH_DEBUG_RENDERER
	#define ACPH_VERSION_FEATURE_BIT_6 1
#else
	#define ACPH_VERSION_FEATURE_BIT_6 0
#endif
#ifdef ACPH_DISABLE_TEMP_ALLOCATOR
	#define ACPH_VERSION_FEATURE_BIT_7 1
#else
	#define ACPH_VERSION_FEATURE_BIT_7 0
#endif
#ifdef ACPH_DISABLE_CUSTOM_ALLOCATOR
	#define ACPH_VERSION_FEATURE_BIT_8 1
#else
	#define ACPH_VERSION_FEATURE_BIT_8 0
#endif
#if defined(ACPH_OBJECT_LAYER_BITS) && ACPH_OBJECT_LAYER_BITS == 32
	#define ACPH_VERSION_FEATURE_BIT_9 1
#else
	#define ACPH_VERSION_FEATURE_BIT_9 0
#endif
#ifdef ACPH_ENABLE_ASSERTS
	#define ACPH_VERSION_FEATURE_BIT_10 1
#else
	#define ACPH_VERSION_FEATURE_BIT_10 0
#endif
#ifdef ACPH_OBJECT_STREAM
	#define ACPH_VERSION_FEATURE_BIT_11 1
#else
	#define ACPH_VERSION_FEATURE_BIT_11 0
#endif
#define ACPH_VERSION_FEATURES (uint64(ACPH_VERSION_FEATURE_BIT_1) | (ACPH_VERSION_FEATURE_BIT_2 << 1) | (ACPH_VERSION_FEATURE_BIT_3 << 2) | (ACPH_VERSION_FEATURE_BIT_4 << 3) | (ACPH_VERSION_FEATURE_BIT_5 << 4) | (ACPH_VERSION_FEATURE_BIT_6 << 5) | (ACPH_VERSION_FEATURE_BIT_7 << 6) | (ACPH_VERSION_FEATURE_BIT_8 << 7) | (ACPH_VERSION_FEATURE_BIT_9 << 8) | (ACPH_VERSION_FEATURE_BIT_10 << 9) | (ACPH_VERSION_FEATURE_BIT_11 << 10))

// Combine the version and features in a single ID
#define ACPH_VERSION_ID ((ACPH_VERSION_FEATURES << 24) | (ACPH_VERSION_MAJOR << 16) | (ACPH_VERSION_MINOR << 8) | ACPH_VERSION_PATCH)

// Determine platform
#if defined(ACPH_PLATFORM_BLUE)
	// Correct define already defined, this overrides everything else
#elif defined(_WIN32) || defined(_WIN64)
	#include <winapifamily.h>
	#if WINAPI_FAMILY == WINAPI_FAMILY_APP
		#define ACPH_PLATFORM_WINDOWS_UWP // Building for Universal Windows Platform
	#endif
	#define ACPH_PLATFORM_WINDOWS
#elif defined(__ANDROID__) // Android is linux too, so that's why we check it first
	#define ACPH_PLATFORM_ANDROID
#elif defined(__linux__)
	#define ACPH_PLATFORM_LINUX
#elif defined(__FreeBSD__) || defined(__OpenBSD__) || defined(__NetBSD__)
	#define ACPH_PLATFORM_BSD
#elif defined(__APPLE__)
	#include <TargetConditionals.h>
	#if defined(TARGET_OS_IPHONE) && !TARGET_OS_IPHONE
		#define ACPH_PLATFORM_MACOS
	#else
		#define ACPH_PLATFORM_IOS
	#endif
#elif defined(__EMSCRIPTEN__)
	#define ACPH_PLATFORM_WASM
#endif

// Platform helper macros
#ifdef ACPH_PLATFORM_ANDROID
	#define ACPH_IF_NOT_ANDROID(x)
#else
	#define ACPH_IF_NOT_ANDROID(x) x
#endif

// Determine compiler
#if defined(__clang__)
	#define ACPH_COMPILER_CLANG
#elif defined(__GNUC__)
	#define ACPH_COMPILER_GCC
#elif defined(_MSC_VER)
	#define ACPH_COMPILER_MSVC
#endif

#if defined(__MINGW64__) || defined (__MINGW32__)
	#define ACPH_COMPILER_MINGW
#endif

// Detect CPU architecture
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
	// X86 CPU architecture
	#define ACPH_CPU_X86
	#if defined(__x86_64__) || defined(_M_X64)
		#define ACPH_CPU_ADDRESS_BITS 64
	#else
		#define ACPH_CPU_ADDRESS_BITS 32
	#endif
	#define ACPH_USE_SSE
	#define ACPH_VECTOR_ALIGNMENT 16
	#define ACPH_DVECTOR_ALIGNMENT 32

	// Detect enabled instruction sets
	#if defined(__AVX512F__) && defined(__AVX512VL__) && defined(__AVX512DQ__) && !defined(ACPH_USE_AVX512)
		#define ACPH_USE_AVX512
	#endif
	#if (defined(__AVX2__) || defined(ACPH_USE_AVX512)) && !defined(ACPH_USE_AVX2)
		#define ACPH_USE_AVX2
	#endif
	#if (defined(__AVX__) || defined(ACPH_USE_AVX2)) && !defined(ACPH_USE_AVX)
		#define ACPH_USE_AVX
	#endif
	#if (defined(__SSE4_2__) || defined(ACPH_USE_AVX)) && !defined(ACPH_USE_SSE4_2)
		#define ACPH_USE_SSE4_2
	#endif
	#if (defined(__SSE4_1__) || defined(ACPH_USE_SSE4_2)) && !defined(ACPH_USE_SSE4_1)
		#define ACPH_USE_SSE4_1
	#endif
	#if (defined(__F16C__) || defined(ACPH_USE_AVX2)) && !defined(ACPH_USE_F16C)
		#define ACPH_USE_F16C
	#endif
	#if (defined(__LZCNT__) || defined(ACPH_USE_AVX2)) && !defined(ACPH_USE_LZCNT)
		#define ACPH_USE_LZCNT
	#endif
	#if (defined(__BMI__) || defined(ACPH_USE_AVX2)) && !defined(ACPH_USE_TZCNT)
		#define ACPH_USE_TZCNT
	#endif
	#ifndef ACPH_CROSS_PLATFORM_DETERMINISTIC // FMA is not compatible with cross platform determinism
		#if defined(ACPH_COMPILER_CLANG) || defined(ACPH_COMPILER_GCC)
			#if defined(__FMA__) && !defined(ACPH_USE_FMADD)
				#define ACPH_USE_FMADD
			#endif
		#elif defined(ACPH_COMPILER_MSVC)
			#if defined(__AVX2__) && !defined(ACPH_USE_FMADD) // AVX2 also enables fused multiply add
				#define ACPH_USE_FMADD
			#endif
		#else
			#error Undefined compiler
		#endif
	#endif
#elif defined(__aarch64__) || defined(_M_ARM64) || defined(__arm__) || defined(_M_ARM)
	// ARM CPU architecture
	#define ACPH_CPU_ARM
	#if defined(__aarch64__) || defined(_M_ARM64)
		#define ACPH_CPU_ADDRESS_BITS 64
		#define ACPH_USE_NEON
		#define ACPH_VECTOR_ALIGNMENT 16
		#define ACPH_DVECTOR_ALIGNMENT 32
	#else
		#define ACPH_CPU_ADDRESS_BITS 32
		#define ACPH_VECTOR_ALIGNMENT 8 // 32-bit ARM does not support aligning on the stack on 16 byte boundaries
		#define ACPH_DVECTOR_ALIGNMENT 8
	#endif
#elif defined(__riscv)
	// RISC-V CPU architecture
	#define ACPH_CPU_RISCV
	#if __riscv_xlen == 64
		#define ACPH_CPU_ADDRESS_BITS 64
		#define ACPH_VECTOR_ALIGNMENT 16
		#define ACPH_DVECTOR_ALIGNMENT 32
	#else
		#define ACPH_CPU_ADDRESS_BITS 32
		#define ACPH_VECTOR_ALIGNMENT 16
		#define ACPH_DVECTOR_ALIGNMENT 8
	#endif
#elif defined(ACPH_PLATFORM_WASM)
	// WebAssembly CPU architecture
	#define ACPH_CPU_WASM
	#if defined(__wasm64__)
		#define ACPH_CPU_ADDRESS_BITS 64
	#else
		#define ACPH_CPU_ADDRESS_BITS 32
	#endif
	#define ACPH_VECTOR_ALIGNMENT 16
	#define ACPH_DVECTOR_ALIGNMENT 32
	#ifdef __wasm_simd128__
		#define ACPH_USE_SSE
		#define ACPH_USE_SSE4_1
		#define ACPH_USE_SSE4_2
	#endif
#elif defined(__powerpc__) || defined(__powerpc64__)
	// PowerPC CPU architecture
	#define ACPH_CPU_PPC
	#if defined(__powerpc64__)
		#define ACPH_CPU_ADDRESS_BITS 64
	#else
		#define ACPH_CPU_ADDRESS_BITS 32
	#endif
	#ifdef _BIG_ENDIAN
		#define ACPH_CPU_BIG_ENDIAN
	#endif
	#define ACPH_VECTOR_ALIGNMENT 16
	#define ACPH_DVECTOR_ALIGNMENT 8
#elif defined(__loongarch__)
	// LoongArch CPU architecture
	#define ACPH_CPU_LOONGARCH
	#if defined(__loongarch64)
		#define ACPH_CPU_ADDRESS_BITS 64
	#else
		#define ACPH_CPU_ADDRESS_BITS 32
	#endif
	#define ACPH_VECTOR_ALIGNMENT 16
	#define ACPH_DVECTOR_ALIGNMENT 8
#elif defined(__e2k__)
	// E2K CPU architecture (MCST Elbrus 2000)
	#define ACPH_CPU_E2K
	#define ACPH_CPU_ADDRESS_BITS 64
	#define ACPH_VECTOR_ALIGNMENT 16
	#define ACPH_DVECTOR_ALIGNMENT 32

	// Compiler flags on e2k arch determine CPU features
	#if defined(__SSE__) && !defined(ACPH_USE_SSE)
		#define ACPH_USE_SSE
	#endif
#else
	#error Unsupported CPU architecture
#endif

// If this define is set, AsterCore is compiled as a shared library
#ifdef ACPH_SHARED_LIBRARY
	#ifdef ACPH_BUILD_SHARED_LIBRARY
		// While building the shared library, we must export these symbols
		#if defined(ACPH_PLATFORM_WINDOWS) && !defined(ACPH_COMPILER_MINGW)
			#define ACPH_EXPORT __declspec(dllexport)
		#else
			#define ACPH_EXPORT __attribute__ ((visibility ("default")))
			#if defined(ACPH_COMPILER_GCC)
				// Prevents an issue with GCC attribute parsing (see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=69585)
				#define ACPH_EXPORT_GCC_BUG_WORKAROUND [[gnu::visibility("default")]]
			#endif
		#endif
	#else
		// When linking against AsterCore, we must import these symbols
		#if defined(ACPH_PLATFORM_WINDOWS) && !defined(ACPH_COMPILER_MINGW)
			#define ACPH_EXPORT __declspec(dllimport)
		#else
			#define ACPH_EXPORT __attribute__ ((visibility ("default")))
			#if defined(ACPH_COMPILER_GCC)
				// Prevents an issue with GCC attribute parsing (see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=69585)
				#define ACPH_EXPORT_GCC_BUG_WORKAROUND [[gnu::visibility("default")]]
			#endif
		#endif
	#endif
#else
	// If the define is not set, we use static linking and symbols don't need to be imported or exported
	#define ACPH_EXPORT
#endif

#ifndef ACPH_EXPORT_GCC_BUG_WORKAROUND
	#define ACPH_EXPORT_GCC_BUG_WORKAROUND ACPH_EXPORT
#endif

// Macro used by the RTTI macros to not export a function
#define ACPH_NO_EXPORT

// Pragmas to store / restore the warning state and to disable individual warnings
#ifdef ACPH_COMPILER_CLANG
#define ACPH_PRAGMA(x)					_Pragma(#x)
#define ACPH_SUPPRESS_WARNING_PUSH		ACPH_PRAGMA(clang diagnostic push)
#define ACPH_SUPPRESS_WARNING_POP		ACPH_PRAGMA(clang diagnostic pop)
#define ACPH_CLANG_SUPPRESS_WARNING(w)	ACPH_PRAGMA(clang diagnostic ignored w)
#if __clang_major__ >= 13
	#define ACPH_CLANG_13_PLUS_SUPPRESS_WARNING(w) ACPH_CLANG_SUPPRESS_WARNING(w)
#else
	#define ACPH_CLANG_13_PLUS_SUPPRESS_WARNING(w)
#endif
#if __clang_major__ >= 16
	#define ACPH_CLANG_16_PLUS_SUPPRESS_WARNING(w) ACPH_CLANG_SUPPRESS_WARNING(w)
#else
	#define ACPH_CLANG_16_PLUS_SUPPRESS_WARNING(w)
#endif
#else
#define ACPH_CLANG_SUPPRESS_WARNING(w)
#define ACPH_CLANG_13_PLUS_SUPPRESS_WARNING(w)
#define ACPH_CLANG_16_PLUS_SUPPRESS_WARNING(w)
#endif
#ifdef ACPH_COMPILER_GCC
#define ACPH_PRAGMA(x)					_Pragma(#x)
#define ACPH_SUPPRESS_WARNING_PUSH		ACPH_PRAGMA(GCC diagnostic push)
#define ACPH_SUPPRESS_WARNING_POP		ACPH_PRAGMA(GCC diagnostic pop)
#define ACPH_GCC_SUPPRESS_WARNING(w)		ACPH_PRAGMA(GCC diagnostic ignored w)
#else
#define ACPH_GCC_SUPPRESS_WARNING(w)
#endif
#ifdef ACPH_COMPILER_MSVC
#define ACPH_PRAGMA(x)					__pragma(x)
#define ACPH_SUPPRESS_WARNING_PUSH		ACPH_PRAGMA(warning (push))
#define ACPH_SUPPRESS_WARNING_POP		ACPH_PRAGMA(warning (pop))
#define ACPH_MSVC_SUPPRESS_WARNING(w)	ACPH_PRAGMA(warning (disable : w))
#if _MSC_VER >= 1920 && _MSC_VER < 1930
	#define ACPH_MSVC2019_SUPPRESS_WARNING(w) ACPH_MSVC_SUPPRESS_WARNING(w)
#else
	#define ACPH_MSVC2019_SUPPRESS_WARNING(w)
#endif
#else
#define ACPH_MSVC_SUPPRESS_WARNING(w)
#define ACPH_MSVC2019_SUPPRESS_WARNING(w)
#endif

// Disable common warnings triggered by AsterCore when compiling with -Wall
#define ACPH_SUPPRESS_WARNINGS																	\
	ACPH_CLANG_SUPPRESS_WARNING("-Wc++98-compat")												\
	ACPH_CLANG_SUPPRESS_WARNING("-Wc++98-compat-pedantic")										\
	ACPH_CLANG_SUPPRESS_WARNING("-Wfloat-equal")													\
	ACPH_CLANG_SUPPRESS_WARNING("-Wsign-conversion")												\
	ACPH_CLANG_SUPPRESS_WARNING("-Wold-style-cast")												\
	ACPH_CLANG_SUPPRESS_WARNING("-Wgnu-anonymous-struct")										\
	ACPH_CLANG_SUPPRESS_WARNING("-Wnested-anon-types")											\
	ACPH_CLANG_SUPPRESS_WARNING("-Wglobal-constructors")											\
	ACPH_CLANG_SUPPRESS_WARNING("-Wexit-time-destructors")										\
	ACPH_CLANG_SUPPRESS_WARNING("-Wnonportable-system-include-path")								\
	ACPH_CLANG_SUPPRESS_WARNING("-Wlanguage-extension-token")									\
	ACPH_CLANG_SUPPRESS_WARNING("-Wunused-parameter")											\
	ACPH_CLANG_SUPPRESS_WARNING("-Wformat-nonliteral")											\
	ACPH_CLANG_SUPPRESS_WARNING("-Wcovered-switch-default")										\
	ACPH_CLANG_SUPPRESS_WARNING("-Wcast-align")													\
	ACPH_CLANG_SUPPRESS_WARNING("-Winvalid-offsetof")											\
	ACPH_CLANG_SUPPRESS_WARNING("-Wgnu-zero-variadic-macro-arguments")							\
	ACPH_CLANG_SUPPRESS_WARNING("-Wdocumentation-unknown-command")								\
	ACPH_CLANG_SUPPRESS_WARNING("-Wctad-maybe-unsupported")										\
	ACPH_CLANG_SUPPRESS_WARNING("-Wswitch-default")												\
	ACPH_CLANG_13_PLUS_SUPPRESS_WARNING("-Wdeprecated-copy")										\
	ACPH_CLANG_13_PLUS_SUPPRESS_WARNING("-Wdeprecated-copy-with-dtor")							\
	ACPH_CLANG_16_PLUS_SUPPRESS_WARNING("-Wunsafe-buffer-usage")									\
	ACPH_IF_NOT_ANDROID(ACPH_CLANG_SUPPRESS_WARNING("-Wimplicit-int-float-conversion"))			\
																								\
	ACPH_GCC_SUPPRESS_WARNING("-Wcomment")														\
	ACPH_GCC_SUPPRESS_WARNING("-Winvalid-offsetof")												\
	ACPH_GCC_SUPPRESS_WARNING("-Wclass-memaccess")												\
	ACPH_GCC_SUPPRESS_WARNING("-Wpedantic")														\
	ACPH_GCC_SUPPRESS_WARNING("-Wunused-parameter")												\
	ACPH_GCC_SUPPRESS_WARNING("-Wmaybe-uninitialized")											\
																								\
	ACPH_MSVC_SUPPRESS_WARNING(4619) /* #pragma warning: there is no warning number 'XXXX' */	\
	ACPH_MSVC_SUPPRESS_WARNING(4514) /* 'X' : unreferenced inline function has been removed */	\
	ACPH_MSVC_SUPPRESS_WARNING(4710) /* 'X' : function not inlined */							\
	ACPH_MSVC_SUPPRESS_WARNING(4711) /* function 'X' selected for automatic inline expansion */	\
	ACPH_MSVC_SUPPRESS_WARNING(4714) /* function 'X' marked as __forceinline not inlined */		\
	ACPH_MSVC_SUPPRESS_WARNING(4820) /* 'X': 'Y' bytes padding added after data member 'Z' */	\
	ACPH_MSVC_SUPPRESS_WARNING(4100) /* 'X' : unreferenced formal parameter */					\
	ACPH_MSVC_SUPPRESS_WARNING(4626) /* 'X' : assignment operator was implicitly defined as deleted because a base class assignment operator is inaccessible or deleted */ \
	ACPH_MSVC_SUPPRESS_WARNING(5027) /* 'X' : move assignment operator was implicitly defined as deleted because a base class move assignment operator is inaccessible or deleted */ \
	ACPH_MSVC_SUPPRESS_WARNING(4365) /* 'argument' : conversion from 'X' to 'Y', signed / unsigned mismatch */ \
	ACPH_MSVC_SUPPRESS_WARNING(4324) /* 'X' : structure was padded due to alignment specifier */ \
	ACPH_MSVC_SUPPRESS_WARNING(4625) /* 'X' : copy constructor was implicitly defined as deleted because a base class copy constructor is inaccessible or deleted */ \
	ACPH_MSVC_SUPPRESS_WARNING(5026) /* 'X': move constructor was implicitly defined as deleted because a base class move constructor is inaccessible or deleted */ \
	ACPH_MSVC_SUPPRESS_WARNING(4623) /* 'X' : default constructor was implicitly defined as deleted */ \
	ACPH_MSVC_SUPPRESS_WARNING(4201) /* nonstandard extension used: nameless struct/union */		\
	ACPH_MSVC_SUPPRESS_WARNING(4371) /* 'X': layout of class may have changed from a previous version of the compiler due to better packing of member 'Y' */ \
	ACPH_MSVC_SUPPRESS_WARNING(5045) /* Compiler will insert Spectre mitigation for memory load if /Qspectre switch specified */ \
	ACPH_MSVC_SUPPRESS_WARNING(4583) /* 'X': destructor is not implicitly called */				\
	ACPH_MSVC_SUPPRESS_WARNING(4582) /* 'X': constructor is not implicitly called */				\
	ACPH_MSVC_SUPPRESS_WARNING(5219) /* implicit conversion from 'X' to 'Y', possible loss of data  */ \
	ACPH_MSVC_SUPPRESS_WARNING(4826) /* Conversion from 'X *' to 'ACPH::uint64' is sign-extended. This may cause unexpected runtime behavior. (32-bit) */ \
	ACPH_MSVC_SUPPRESS_WARNING(5264) /* 'X': 'const' variable is not used */						\
	ACPH_MSVC_SUPPRESS_WARNING(4251) /* class 'X' needs to have DLL-interface to be used by clients of class 'Y' */ \
	ACPH_MSVC_SUPPRESS_WARNING(4738) /* storing 32-bit float result in memory, possible loss of performance */ \
	ACPH_MSVC2019_SUPPRESS_WARNING(5246) /* the initialization of a subobject should be wrapped in braces */

// OS-specific includes
#if defined(ACPH_PLATFORM_WINDOWS)
	#define ACPH_BREAKPOINT		__debugbreak()
#elif defined(ACPH_PLATFORM_BLUE)
	// Configuration for a popular game console.
	// This file is not distributed because it would violate an NDA.
	// Creating one should only be a couple of minutes of work if you have the documentation for the platform
	// (you only need to define ACPH_BREAKPOINT, ACPH_PLATFORM_BLUE_GET_TICKS, ACPH_PLATFORM_BLUE_MUTEX*, ACPH_PLATFORM_BLUE_RWLOCK*, ACPH_PLATFORM_BLUE_SEMAPHORE* and include the right header).
	#include <AsterCore/Core/PlatformBlue.h>
#elif defined(ACPH_PLATFORM_LINUX) || defined(ACPH_PLATFORM_ANDROID) || defined(ACPH_PLATFORM_MACOS) || defined(ACPH_PLATFORM_IOS) || defined(ACPH_PLATFORM_BSD)
	#if defined(ACPH_CPU_X86)
		#define ACPH_BREAKPOINT	__asm volatile ("int $0x3")
	#elif defined(ACPH_CPU_ARM) || defined(ACPH_CPU_RISCV) || defined(ACPH_CPU_E2K) || defined(ACPH_CPU_PPC) || defined(ACPH_CPU_LOONGARCH)
		#define ACPH_BREAKPOINT	__builtin_trap()
	#else
		#error Unknown CPU architecture
	#endif
#elif defined(ACPH_PLATFORM_WASM)
	#define ACPH_BREAKPOINT		do { } while (false) // Not supported
#else
	#error Unknown platform
#endif

// Begin the ACPH namespace
#define ACPH_NAMESPACE_BEGIN																		\
	ACPH_SUPPRESS_WARNING_PUSH																	\
	ACPH_SUPPRESS_WARNINGS																		\
	namespace ACPH {

// End the ACPH namespace
#define ACPH_NAMESPACE_END																		\
	}																							\
	ACPH_SUPPRESS_WARNING_POP

// Suppress warnings generated by the standard template library
#define ACPH_SUPPRESS_WARNINGS_STD_BEGIN															\
	ACPH_SUPPRESS_WARNING_PUSH																	\
	ACPH_MSVC_SUPPRESS_WARNING(4365)																\
	ACPH_MSVC_SUPPRESS_WARNING(4619)																\
	ACPH_MSVC_SUPPRESS_WARNING(4710)																\
	ACPH_MSVC_SUPPRESS_WARNING(4711)																\
	ACPH_MSVC_SUPPRESS_WARNING(4820)																\
	ACPH_MSVC_SUPPRESS_WARNING(4514)																\
	ACPH_MSVC_SUPPRESS_WARNING(5262)																\
	ACPH_MSVC_SUPPRESS_WARNING(5264)																\
	ACPH_MSVC_SUPPRESS_WARNING(4738)																\
	ACPH_MSVC_SUPPRESS_WARNING(5045)

#define ACPH_SUPPRESS_WARNINGS_STD_END															\
	ACPH_SUPPRESS_WARNING_POP

// MSVC STL requires _HAS_EXCEPTIONS=0 if exceptions are turned off
#if defined(ACPH_COMPILER_MSVC) && (!defined(__cpp_exceptions) || !__cpp_exceptions) && !defined(_HAS_EXCEPTIONS)
	#define _HAS_EXCEPTIONS 0
#endif

// Standard C++ includes
ACPH_SUPPRESS_WARNINGS_STD_BEGIN
#include <float.h>
#include <limits.h>
#include <string.h>
#include <utility>
#include <cmath>
#include <sstream>
#include <functional>
#include <algorithm>
#include <cstdint>
#if defined(ACPH_COMPILER_MSVC) || (defined(ACPH_COMPILER_CLANG) && defined(_MSC_VER)) // MSVC or clang-cl
	#include <malloc.h> // for alloca
#endif
#if defined(ACPH_USE_SSE)
	#include <immintrin.h>
#elif defined(ACPH_USE_NEON)
	#ifdef ACPH_COMPILER_MSVC
		#include <intrin.h>
		#include <arm64_neon.h>
	#else
		#include <arm_neon.h>
	#endif
#endif
ACPH_SUPPRESS_WARNINGS_STD_END

ACPH_NAMESPACE_BEGIN

// Commonly used STL types
using std::min;
using std::max;
using std::abs;
using std::sqrt;
using std::ceil;
using std::floor;
using std::trunc;
using std::round;
using std::fmod;
using std::string_view;
using std::function;
using std::numeric_limits;
using std::isfinite;
using std::isnan;
using std::ostream;
using std::istream;

// Standard types
using uint = unsigned int;
using uint8 = std::uint8_t;
using uint16 = std::uint16_t;
using uint32 = std::uint32_t;
using uint64 = std::uint64_t;

// Assert sizes of types
static_assert(sizeof(uint) >= 4, "Invalid size of uint");
static_assert(sizeof(uint8) == 1, "Invalid size of uint8");
static_assert(sizeof(uint16) == 2, "Invalid size of uint16");
static_assert(sizeof(uint32) == 4, "Invalid size of uint32");
static_assert(sizeof(uint64) == 8, "Invalid size of uint64");
static_assert(sizeof(void *) == (ACPH_CPU_ADDRESS_BITS == 64? 8 : 4), "Invalid size of pointer" );

// Determine if we want extra debugging code to be active
#if !defined(NDEBUG) && !defined(ACPH_NO_DEBUG)
	#define ACPH_DEBUG
#endif

// Define inline macro
#if defined(ACPH_NO_FORCE_INLINE)
	#define ACPH_INLINE inline
#elif defined(ACPH_COMPILER_CLANG)
	#define ACPH_INLINE __inline__ __attribute__((always_inline))
#elif defined(ACPH_COMPILER_GCC)
	// On gcc 14 using always_inline in debug mode causes error: "inlining failed in call to 'always_inline' 'XXX': function not considered for inlining"
	// See: https://github.com/jrouwe/JoltPhysics/issues/1096
	#if __GNUC__ >= 14 && defined(ACPH_DEBUG)
		#define ACPH_INLINE inline
	#else
		#define ACPH_INLINE __inline__ __attribute__((always_inline))
	#endif
#elif defined(ACPH_COMPILER_MSVC)
	#define ACPH_INLINE __forceinline
#else
	#error Undefined
#endif

// Cache line size (used for aligning to cache line)
#ifndef ACPH_CACHE_LINE_SIZE
	#define ACPH_CACHE_LINE_SIZE 64
#endif

// Define macro to get current function name
#if defined(ACPH_COMPILER_CLANG) || defined(ACPH_COMPILER_GCC)
	#define ACPH_FUNCTION_NAME	__PRETTY_FUNCTION__
#elif defined(ACPH_COMPILER_MSVC)
	#define ACPH_FUNCTION_NAME	__FUNCTION__
#else
	#error Undefined
#endif

// Stack allocation
#define ACPH_STACK_ALLOC(n)		alloca(n)

// Shorthand for #ifdef ACPH_DEBUG / #endif
#ifdef ACPH_DEBUG
	#define ACPH_IF_DEBUG(...)	__VA_ARGS__
	#define ACPH_IF_NOT_DEBUG(...)
#else
	#define ACPH_IF_DEBUG(...)
	#define ACPH_IF_NOT_DEBUG(...) __VA_ARGS__
#endif

// Shorthand for #ifdef ACPH_FLOATING_POINT_EXCEPTIONS_ENABLED / #endif
#ifdef ACPH_FLOATING_POINT_EXCEPTIONS_ENABLED
	#define ACPH_IF_FLOATING_POINT_EXCEPTIONS_ENABLED(...)	__VA_ARGS__
#else
	#define ACPH_IF_FLOATING_POINT_EXCEPTIONS_ENABLED(...)
#endif

// Helper macros to detect if we're running in single or double precision mode
#ifdef ACPH_DOUBLE_PRECISION
	#define ACPH_IF_SINGLE_PRECISION(...)
	#define ACPH_IF_SINGLE_PRECISION_ELSE(s, d) d
	#define ACPH_IF_DOUBLE_PRECISION(...) __VA_ARGS__
#else
	#define ACPH_IF_SINGLE_PRECISION(...) __VA_ARGS__
	#define ACPH_IF_SINGLE_PRECISION_ELSE(s, d) s
	#define ACPH_IF_DOUBLE_PRECISION(...)
#endif

// Helper macro to detect if the debug renderer is active
#ifdef ACPH_DEBUG_RENDERER
	#define ACPH_IF_DEBUG_RENDERER(...) __VA_ARGS__
	#define ACPH_IF_NOT_DEBUG_RENDERER(...)
#else
	#define ACPH_IF_DEBUG_RENDERER(...)
	#define ACPH_IF_NOT_DEBUG_RENDERER(...) __VA_ARGS__
#endif

// Macro to indicate that a parameter / variable is unused
#define ACPH_UNUSED(x)			(void)x

// Macro to enable floating point precise mode and to disable fused multiply add instructions
#if defined(ACPH_COMPILER_GCC) || defined(ACPH_CROSS_PLATFORM_DETERMINISTIC)
	// We compile without -ffast-math and -ffp-contract=fast, so we don't need to disable anything
	#define ACPH_PRECISE_MATH_ON
	#define ACPH_PRECISE_MATH_OFF
#elif defined(ACPH_COMPILER_CLANG)
	// We compile without -ffast-math because pragma float_control(precise, on) doesn't seem to actually negate all of the -ffast-math effects and causes the unit tests to fail (even if the pragma is added to all files)
	// On clang 14 and later we can turn off float contraction through a pragma (before it was buggy), so if FMA is on we can disable it through this macro
	#if (defined(ACPH_CPU_ARM) && !defined(ACPH_PLATFORM_ANDROID) && __clang_major__ >= 16) || (defined(ACPH_CPU_X86) && __clang_major__ >= 14)
		#define ACPH_PRECISE_MATH_ON						\
			_Pragma("float_control(precise, on, push)")	\
			_Pragma("clang fp contract(off)")
		#define ACPH_PRECISE_MATH_OFF					\
			_Pragma("float_control(pop)")
	#elif __clang_major__ >= 14 && (defined(ACPH_USE_FMADD) || defined(FP_FAST_FMA))
		#define ACPH_PRECISE_MATH_ON						\
			_Pragma("clang fp contract(off)")
		#define ACPH_PRECISE_MATH_OFF					\
			_Pragma("clang fp contract(on)")
	#else
		#define ACPH_PRECISE_MATH_ON
		#define ACPH_PRECISE_MATH_OFF
	#endif
#elif defined(ACPH_COMPILER_MSVC)
	// Unfortunately there is no way to push the state of fp_contract, so we have to assume it was turned on before ACPH_PRECISE_MATH_ON
	#define ACPH_PRECISE_MATH_ON							\
		__pragma(float_control(precise, on, push))		\
		__pragma(fp_contract(off))
	#define ACPH_PRECISE_MATH_OFF						\
		__pragma(fp_contract(on))						\
		__pragma(float_control(pop))
#else
	#error Undefined
#endif

// Check if Thread Sanitizer is enabled
#ifdef __has_feature
	#if __has_feature(thread_sanitizer)
		#define ACPH_TSAN_ENABLED
	#endif
#else
	#ifdef __SANITIZE_THREAD__
		#define ACPH_TSAN_ENABLED
	#endif
#endif

// Attribute to disable Thread Sanitizer for a particular function
#ifdef ACPH_TSAN_ENABLED
	#define ACPH_TSAN_NO_SANITIZE __attribute__((no_sanitize("thread")))
#else
	#define ACPH_TSAN_NO_SANITIZE
#endif

ACPH_NAMESPACE_END
