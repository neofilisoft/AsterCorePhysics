// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2024 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

class SoftBodyBendConstraintTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, SoftBodyBendConstraintTest)

	// Description of the test
	virtual const char *	GetDescription() const override
	{
		return "Shows the effect of bend constraint type in a soft body.";
	}

	// See: Test
	virtual void			Initialize() override;
	virtual void			PrePhysicsUpdate(const PreUpdateParams &inParams) override;

private:
	// Size and spacing of the cloth
	static constexpr int	cNumVerticesX = 10;
	static constexpr int	cNumVerticesZ = 10;
	static constexpr float	cVertexSpacing = 0.5f;
};
