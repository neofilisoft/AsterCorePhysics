// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>
#include <AsterCore/Skeleton/Skeleton.h>
#include <AsterCore/Skeleton/SkeletalAnimation.h>
#include <AsterCore/Skeleton/SkeletonPose.h>
#include <AsterCore/Physics/Ragdoll/Ragdoll.h>

class PoweredRigTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, PoweredRigTest)

	// Description of the test
	virtual const char *	GetDescription() const override
	{
		return "Demonstrates how to use motors to drive a ragdoll to a pose.";
	}

	// Destructor
	virtual					~PoweredRigTest() override;

	// Number used to scale the terrain and camera movement to the scene
	virtual float			GetWorldScale() const override								{ return 0.2f; }

	virtual void			Initialize() override;
	virtual void			PrePhysicsUpdate(const PreUpdateParams &inParams) override;

	// Optional settings menu
	virtual bool			HasSettingsMenu() const override							{ return true; }
	virtual void			CreateSettingsMenu(DebugUI *inUI, UIElement *inSubMenu) override;

	// Saving / restoring state for replay
	virtual void			SaveState(StateRecorder &inStream) const override;
	virtual void			RestoreState(StateRecorder &inStream) override;

private:
	// List of possible animation names
	static const char *		sAnimations[];

	// Filename of animation to load for this test
	static const char *		sAnimationName;

	float					mTime = 0.0f;
	Ref<RagdollSettings>	mRagdollSettings;
	Ref<Ragdoll>			mRagdoll;
	Ref<SkeletalAnimation>	mAnimation;
	SkeletonPose			mPose;
};
