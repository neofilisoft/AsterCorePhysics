// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2024 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

class DynamicScaledShape : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, DynamicScaledShape)

	// Description of the test
	virtual const char *GetDescription() const override
	{
		return "Demonstrates how you can scale a shape dynamically while a body is being simulated.";
	}

	// See: Test
	virtual void	Initialize() override;
	virtual void	PrePhysicsUpdate(const PreUpdateParams &inParams) override;

	// Saving / restoring state for replay
	virtual void	SaveState(StateRecorder &inStream) const override;
	virtual void	RestoreState(StateRecorder &inStream) override;

private:
	BodyID			mBodyID;
	float			mTime = 0.0f;
};
