// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/Core/FPControlWord.h>

ACPH_NAMESPACE_BEGIN

#if defined(ACPH_CPU_WASM) || defined(ACPH_CPU_RISCV) || defined(ACPH_CPU_PPC) || defined(ACPH_CPU_LOONGARCH)

// Not supported
class FPFlushDenormals { };

#elif defined(ACPH_USE_SSE)

/// Helper class that needs to be put on the stack to enable flushing denormals to zero
/// This can make floating point operations much faster when working with very small numbers
class FPFlushDenormals : public FPControlWord<_MM_FLUSH_ZERO_ON, _MM_FLUSH_ZERO_MASK> { };

#elif defined(ACPH_CPU_ARM) && defined(ACPH_COMPILER_MSVC)

/// Helper class that needs to be put on the stack to enable flushing denormals to zero
/// This can make floating point operations much faster when working with very small numbers
class FPFlushDenormals : public FPControlWord<_DN_FLUSH, _MCW_DN> { };

#elif defined(ACPH_CPU_ARM)

/// Flush denormals to zero bit
static constexpr uint64 FP_FZ = 1 << 24;

/// Helper class that needs to be put on the stack to enable flushing denormals to zero
/// This can make floating point operations much faster when working with very small numbers
class FPFlushDenormals : public FPControlWord<FP_FZ, FP_FZ> { };

#else

#error Unsupported CPU architecture

#endif

ACPH_NAMESPACE_END
