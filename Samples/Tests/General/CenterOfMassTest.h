// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

class CenterOfMassTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, CenterOfMassTest)

	// Description of the test
	virtual const char *GetDescription() const override
	{
		return "Spawns various shapes with the center of mass not in the center of the object.";
	}

	// See: Test
	virtual void		Initialize() override;
};
