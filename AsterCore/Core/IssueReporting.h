// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

ACPH_NAMESPACE_BEGIN

/// Trace function, needs to be overridden by application. This should output a line of text to the log / TTY.
using TraceFunction = void (*)(const char *inFMT, ...);
ACPH_EXPORT extern TraceFunction Trace;

// Always turn on asserts in Debug mode
#if defined(ACPH_DEBUG) && !defined(ACPH_ENABLE_ASSERTS)
	#define ACPH_ENABLE_ASSERTS
#endif

#ifdef ACPH_ENABLE_ASSERTS
	/// Function called when an assertion fails. This function should return true if a breakpoint needs to be triggered
	using AssertFailedFunction = bool(*)(const char *inExpression, const char *inMessage, const char *inFile, uint inLine);
	ACPH_EXPORT extern AssertFailedFunction AssertFailed;

	// Helper functions to pass message on to failed function
	struct AssertLastParam { };
	inline bool AssertFailedParamHelper(const char *inExpression, const char *inFile, uint inLine, AssertLastParam) { return AssertFailed(inExpression, nullptr, inFile, inLine); }
	inline bool AssertFailedParamHelper(const char *inExpression, const char *inFile, uint inLine, const char *inMessage, AssertLastParam) { return AssertFailed(inExpression, inMessage, inFile, inLine); }

	/// Main assert macro, usage: ACPH_ASSERT(condition, message) or ACPH_ASSERT(condition)
	#define ACPH_ASSERT(inExpression, ...)	do { if (!(inExpression) && AssertFailedParamHelper(#inExpression, __FILE__, ACPH::uint(__LINE__), ##__VA_ARGS__, ACPH::AssertLastParam())) ACPH_BREAKPOINT; } while (false)

	#define ACPH_IF_ENABLE_ASSERTS(...)		__VA_ARGS__
#else
	#define ACPH_ASSERT(...)					((void)0)

	#define ACPH_IF_ENABLE_ASSERTS(...)
#endif // ACPH_ENABLE_ASSERTS

ACPH_NAMESPACE_END
