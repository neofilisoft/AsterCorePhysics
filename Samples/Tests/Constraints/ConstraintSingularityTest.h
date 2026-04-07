// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

class ConstraintSingularityTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, ConstraintSingularityTest)

	// Description of the test
	virtual const char *GetDescription() const override
	{
		return "Starts constraints in a configuration where there are multiple directions to move in to satisfy the constraint.";
	}

	// See: Test
	virtual void		Initialize() override;
};
