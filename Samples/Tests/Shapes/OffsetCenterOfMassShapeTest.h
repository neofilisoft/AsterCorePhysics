// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

// Tests the OffsetCenterOfMass shape
class OffsetCenterOfMassShapeTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, OffsetCenterOfMassShapeTest)

	// See: Test
	virtual void	Initialize() override;
};
