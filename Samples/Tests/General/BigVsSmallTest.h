// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

class BigVsSmallTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, BigVsSmallTest)

	// Description of the test
	virtual const char *GetDescription() const override
	{
		return "A small box falling on a big triangle to test for numerical precision errors.";
	}

	// See: Test
	virtual void		Initialize() override;
};
