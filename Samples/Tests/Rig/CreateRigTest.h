// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2022 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>
#include <AsterCore/Physics/Ragdoll/Ragdoll.h>

class CreateRigTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, CreateRigTest)

	// Description of the test
	virtual const char *			GetDescription() const override
	{
		return "Demonstrates how to create a ragdoll from code.";
	}

	// Destructor
	virtual							~CreateRigTest() override;

	// Number used to scale the terrain and camera movement to the scene
	virtual float					GetWorldScale() const override								{ return 0.2f; }

	virtual void					Initialize() override;

private:
	// Our ragdoll
	Ref<Ragdoll>					mRagdoll;
};
