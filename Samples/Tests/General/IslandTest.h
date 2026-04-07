// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

class IslandTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, IslandTest)

	// Description of the test
	virtual const char *GetDescription() const override
	{
		return "Creates a number of disjoint piles of blocks to see if the islands are properly determined and that the simulation spreads them out over multiple CPUs.";
	}

	// See: Test
	virtual void		Initialize() override;
};
