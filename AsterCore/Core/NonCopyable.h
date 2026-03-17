// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

ACPH_NAMESPACE_BEGIN

/// Class that makes another class non-copyable. Usage: Inherit from NonCopyable.
class ACPH_EXPORT NonCopyable
{
public:
			NonCopyable() = default;
			NonCopyable(const NonCopyable &) = delete;
	void	operator = (const NonCopyable &) = delete;
};

ACPH_NAMESPACE_END
