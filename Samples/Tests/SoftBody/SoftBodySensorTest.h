// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2024 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>
#include <AsterCore/Physics/SoftBody/SoftBodyContactListener.h>

class SoftBodySensorTest : public Test, public SoftBodyContactListener
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, SoftBodySensorTest)

	// Description of the test
	virtual const char *GetDescription() const override
	{
		return "Shows interaction between a soft body and a sensor.";
	}

	// See: Test
	virtual void		Initialize() override;

	// See: SoftBodyContactListener
	virtual void		OnSoftBodyContactAdded(const Body &inSoftBody, const SoftBodyManifold &inManifold) override;
};
