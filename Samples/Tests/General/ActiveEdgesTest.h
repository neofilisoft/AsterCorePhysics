// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

class ActiveEdgesTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, ActiveEdgesTest)

	// Description of the test
	virtual const char *GetDescription() const override
	{
		return "Boxes sliding over the ramps should not collide with internal triangle edges of the ramp (aka ghost collisions).";
	}

	// See: Test
	virtual void		Initialize() override;
};
