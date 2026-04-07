// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2024 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

class SoftBodyCustomUpdateTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, SoftBodyCustomUpdateTest)

	// Description of the test
	virtual const char *GetDescription() const override
	{
		return "Shows how you can update a soft body outside of the main physics simulation step.";
	}

	// See: Test
	virtual void		Initialize() override;
	virtual void		PrePhysicsUpdate(const PreUpdateParams &inParams) override;

private:
	Body *				mBody;
};
