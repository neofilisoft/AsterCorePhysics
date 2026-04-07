// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2022 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>
#include <AsterCore/Skeleton/Skeleton.h>
#include <AsterCore/Skeleton/SkeletalAnimation.h>
#include <AsterCore/Skeleton/SkeletonPose.h>
#include <AsterCore/Skeleton/SkeletonMapper.h>
#include <Utils/RagdollLoader.h>
#include <AsterCore/Physics/Ragdoll/Ragdoll.h>

class SkeletonMapperTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, SkeletonMapperTest)

	// Description of the test
	virtual const char *	GetDescription() const override
	{
		return "Shows how you can map a high detail animation skeleton on a low detail physics skeleton and back.";
	}

	// Destructor
	virtual					~SkeletonMapperTest() override;

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
	inline static bool		sLockTranslations = false;

	void					CalculateRagdollPose();

	float					mTime = 0.0f;
	Ref<RagdollSettings>	mRagdollSettings;
	Ref<Ragdoll>			mRagdoll;
	Ref<SkeletalAnimation>	mAnimation;
	SkeletonMapper			mRagdollToAnimated;
	SkeletonPose			mAnimatedPose;
	SkeletonPose			mRagdollPose;
};
