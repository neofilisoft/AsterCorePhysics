// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2023 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

class SoftBodyShapesTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, SoftBodyShapesTest)

	// Description of the test
	virtual const char *GetDescription() const override
	{
		return "Shows interaction between various collision shapes and soft bodies.";
	}

	// See: Test
	virtual void		Initialize() override;
};
