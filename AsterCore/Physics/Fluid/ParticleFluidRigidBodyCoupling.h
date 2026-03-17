// AsterCore Physics Library
// SPDX-FileCopyrightText: 2026 OpenAI
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>
#include <AsterCore/Physics/Body/BodyID.h>

ACPH_NAMESPACE_BEGIN

class BodyInterface;
class JobSystem;
class ParticleFluidSystem;

struct ParticleFluidRigidBodyCouplingSettings
{
	float						mInteractionRadiusScale = 2.0f;
	float						mBuoyancyScale = 1.0f;
	float						mPressureScale = 0.015f;
	uint32						mMinBatchSize = 32;
};

class ACPH_EXPORT ParticleFluidRigidBodyCoupling final
{
public:
	static void					sApply(const ParticleFluidSystem &inFluidSystem, const BodyID *inBodies, uint32 inBodyCount, BodyInterface &ioBodyInterface, JobSystem &inJobSystem, const ParticleFluidRigidBodyCouplingSettings &inSettings);
};

ACPH_NAMESPACE_END