// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2024 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/Core/NonCopyable.h>

ACPH_NAMESPACE_BEGIN

/// Class that calls a function when it goes out of scope
template <class F>
class ScopeExit : public NonCopyable
{
public:
	/// Constructor specifies the exit function
	ACPH_INLINE explicit	ScopeExit(F &&inFunction) : mFunction(std::move(inFunction)) { }

	/// Destructor calls the exit function
	ACPH_INLINE			~ScopeExit() { if (!mInvoked) mFunction(); }

	/// Call the exit function now instead of when going out of scope
	ACPH_INLINE void		Invoke()
	{
		if (!mInvoked)
		{
			mFunction();
			mInvoked = true;
		}
	}

	/// No longer call the exit function when going out of scope
	ACPH_INLINE void		Release()
	{
		mInvoked = true;
	}

private:
	F					mFunction;
	bool				mInvoked = false;
};

#define ACPH_SCOPE_EXIT_TAG2(line)			scope_exit##line
#define ACPH_SCOPE_EXIT_TAG(line)			ACPH_SCOPE_EXIT_TAG2(line)

/// Usage: ACPH_SCOPE_EXIT([]{ code to call on scope exit });
#define ACPH_SCOPE_EXIT(...) ScopeExit ACPH_SCOPE_EXIT_TAG(__LINE__)(__VA_ARGS__)

ACPH_NAMESPACE_END
