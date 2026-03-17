// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Core/TickCounter.h>

#if defined(ACPH_PLATFORM_WINDOWS)
	ACPH_SUPPRESS_WARNING_PUSH
	ACPH_MSVC_SUPPRESS_WARNING(5039) // winbase.h(13179): warning C5039: 'TpSetCallbackCleanupGroup': pointer or reference to potentially throwing function passed to 'extern "C"' function under -EHc. Undefined behavior may occur if this function throws an exception.
	#ifndef WIN32_LEAN_AND_MEAN
		#define WIN32_LEAN_AND_MEAN
	#endif
#ifndef ACPH_COMPILER_MINGW
	#include <Windows.h>
#else
	#include <windows.h>
#endif
	ACPH_SUPPRESS_WARNING_POP
#endif

ACPH_NAMESPACE_BEGIN

#if defined(ACPH_PLATFORM_WINDOWS_UWP) || (defined(ACPH_PLATFORM_WINDOWS) && defined(ACPH_CPU_ARM))

uint64 GetProcessorTickCount()
{
	LARGE_INTEGER count;
	QueryPerformanceCounter(&count);
	return uint64(count.QuadPart);
}

#endif // ACPH_PLATFORM_WINDOWS_UWP || (ACPH_PLATFORM_WINDOWS && ACPH_CPU_ARM)

ACPH_NAMESPACE_END
