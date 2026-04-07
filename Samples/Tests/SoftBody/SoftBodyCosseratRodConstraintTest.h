// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2025 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

class SoftBodyCosseratRodConstraintTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, SoftBodyCosseratRodConstraintTest)

	// Description of the test
	virtual const char *	GetDescription() const override
	{
		return "Shows the effect of Cosserat rod constraints in a soft body that control bend, twist and shear between particles.";
	}

	// See: Test
	virtual void			Initialize() override;
	virtual void			PrePhysicsUpdate(const PreUpdateParams &inParams) override;

private:
	BodyIDVector			mSoftBodies;
};
