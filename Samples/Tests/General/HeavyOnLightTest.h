// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

class HeavyOnLightTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, HeavyOnLightTest)

	// Description of the test
	virtual const char *GetDescription() const override
	{
		return	"This test spawns a number of heavy boxes (with increasing weight) on light boxes.\n"
				"Shows that iterative solvers have issues with large mass differences.";
	}

	// See: Test
	virtual void		Initialize() override;
};
