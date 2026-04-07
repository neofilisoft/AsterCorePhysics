// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

// Tests the RotatedTranslated shape
class RotatedTranslatedShapeTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, RotatedTranslatedShapeTest)

	// See: Test
	virtual void	Initialize() override;
};
