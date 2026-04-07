// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

class RestitutionTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, RestitutionTest)

	// Description of the test
	virtual const char *GetDescription() const override
	{
		return "Bodies with varying restitutions.";
	}

	// See: Test
	virtual void		Initialize() override;
};
