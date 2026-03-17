// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2023 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

class SoftBodyRestitutionTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, SoftBodyRestitutionTest)

	// Description of the test
	virtual const char *GetDescription() const override
	{
		return "Tests soft bodies with various values for restitution. Note that this has very little effect.";
	}

	// See: Test
	virtual void		Initialize() override;
};
