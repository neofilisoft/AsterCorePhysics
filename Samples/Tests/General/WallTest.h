// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

class WallTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, WallTest)

	// Description of the test
	virtual const char *GetDescription() const override
	{
		return	"Tests a large pile of boxes to check stacking and performance behavior.";
	}

	// See: Test
	virtual void		Initialize() override;
};
