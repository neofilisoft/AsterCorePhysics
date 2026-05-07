// AsterCore Physics Library
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>

ACPH_NAMESPACE_BEGIN

struct VBDConstraintTuning
{
	float						mCompliance = 0.0f;
	float						mVelocityBias = 0.15f;
	float						mShockPropagation = 0.0f;
	float						mWarmStartBlend = 0.75f;
	float						mMaxCorrectionSpeed = 20.0f;
	uint32						mSolverIterations = 4;
	bool						mEnableSubstepping = true;
};

ACPH_NAMESPACE_END
