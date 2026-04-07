// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

class StackTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, StackTest)

	// Description of the test
	virtual const char *GetDescription() const override
	{
		return "Stacks a number of boxes to see if the simulation is stable.";
	}

	// See: Test
	virtual void		Initialize() override;
};
