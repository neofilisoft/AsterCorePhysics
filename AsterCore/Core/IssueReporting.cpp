// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

ACPH_NAMESPACE_BEGIN

static void DummyTrace([[maybe_unused]] const char *inFMT, ...)
{
	ACPH_ASSERT(false);
};

TraceFunction Trace = DummyTrace;

#ifdef ACPH_ENABLE_ASSERTS

static bool DummyAssertFailed(const char *inExpression, const char *inMessage, const char *inFile, uint inLine)
{
	return true; // Trigger breakpoint
};

AssertFailedFunction AssertFailed = DummyAssertFailed;

#endif // ACPH_ENABLE_ASSERTS

ACPH_NAMESPACE_END
