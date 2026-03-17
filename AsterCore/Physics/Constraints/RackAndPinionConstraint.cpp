// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Constraints/RackAndPinionConstraint.h>
#include <AsterCore/Physics/Constraints/HingeConstraint.h>
#include <AsterCore/Physics/Constraints/SliderConstraint.h>
#include <AsterCore/Physics/Body/Body.h>
#include <AsterCore/ObjectStream/TypeDeclarations.h>
#include <AsterCore/Core/StreamIn.h>
#include <AsterCore/Core/StreamOut.h>
#ifdef ACPH_DEBUG_RENDERER
	#include <AsterCore/Renderer/DebugRenderer.h>
#endif // ACPH_DEBUG_RENDERER

ACPH_NAMESPACE_BEGIN

ACPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(RackAndPinionConstraintSettings)
{
	ACPH_ADD_BASE_CLASS(RackAndPinionConstraintSettings, TwoBodyConstraintSettings)

	ACPH_ADD_ENUM_ATTRIBUTE(RackAndPinionConstraintSettings, mSpace)
	ACPH_ADD_ATTRIBUTE(RackAndPinionConstraintSettings, mHingeAxis)
	ACPH_ADD_ATTRIBUTE(RackAndPinionConstraintSettings, mSliderAxis)
	ACPH_ADD_ATTRIBUTE(RackAndPinionConstraintSettings, mRatio)
}

void RackAndPinionConstraintSettings::SaveBinaryState(StreamOut &inStream) const
{
	ConstraintSettings::SaveBinaryState(inStream);

	inStream.Write(mSpace);
	inStream.Write(mHingeAxis);
	inStream.Write(mSliderAxis);
	inStream.Write(mRatio);
}

void RackAndPinionConstraintSettings::RestoreBinaryState(StreamIn &inStream)
{
	ConstraintSettings::RestoreBinaryState(inStream);

	inStream.Read(mSpace);
	inStream.Read(mHingeAxis);
	inStream.Read(mSliderAxis);
	inStream.Read(mRatio);
}

TwoBodyConstraint *RackAndPinionConstraintSettings::Create(Body &inBody1, Body &inBody2) const
{
	return new RackAndPinionConstraint(inBody1, inBody2, *this);
}

RackAndPinionConstraint::RackAndPinionConstraint(Body &inBody1, Body &inBody2, const RackAndPinionConstraintSettings &inSettings) :
	TwoBodyConstraint(inBody1, inBody2, inSettings),
	mLocalSpaceHingeAxis(inSettings.mHingeAxis),
	mLocalSpaceSliderAxis(inSettings.mSliderAxis),
	mRatio(inSettings.mRatio)
{
	if (inSettings.mSpace == EConstraintSpace::WorldSpace)
	{
		// If all properties were specified in world space, take them to local space now
		mLocalSpaceHingeAxis = inBody1.GetInverseCenterOfMassTransform().Multiply3x3(mLocalSpaceHingeAxis).Normalized();
		mLocalSpaceSliderAxis = inBody2.GetInverseCenterOfMassTransform().Multiply3x3(mLocalSpaceSliderAxis).Normalized();
	}
}

void RackAndPinionConstraint::CalculateConstraintProperties(Mat44Arg inRotation1, Mat44Arg inRotation2)
{
	// Calculate world space normals
	mWorldSpaceHingeAxis = inRotation1 * mLocalSpaceHingeAxis;
	mWorldSpaceSliderAxis = inRotation2 * mLocalSpaceSliderAxis;

	mRackAndPinionConstraintPart.CalculateConstraintProperties(*mBody1, mWorldSpaceHingeAxis, *mBody2, mWorldSpaceSliderAxis, mRatio);
}

void RackAndPinionConstraint::SetupVelocityConstraint(float inDeltaTime)
{
	// Calculate constraint properties that are constant while bodies don't move
	Mat44 rotation1 = Mat44::sRotation(mBody1->GetRotation());
	Mat44 rotation2 = Mat44::sRotation(mBody2->GetRotation());
	CalculateConstraintProperties(rotation1, rotation2);
}

void RackAndPinionConstraint::ResetWarmStart()
{
	mRackAndPinionConstraintPart.Deactivate();
}

void RackAndPinionConstraint::WarmStartVelocityConstraint(float inWarmStartImpulseRatio)
{
	// Warm starting: Apply previous frame impulse
	mRackAndPinionConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
}

bool RackAndPinionConstraint::SolveVelocityConstraint(float inDeltaTime)
{
	return mRackAndPinionConstraintPart.SolveVelocityConstraint(*mBody1, mWorldSpaceHingeAxis, *mBody2, mWorldSpaceSliderAxis, mRatio);
}

bool RackAndPinionConstraint::SolvePositionConstraint(float inDeltaTime, float inBaumgarte)
{
	if (mRackConstraint == nullptr || mPinionConstraint == nullptr)
		return false;

	float rotation;
	if (mPinionConstraint->GetSubType() == EConstraintSubType::Hinge)
	{
		rotation = StaticCast<HingeConstraint>(mPinionConstraint)->GetCurrentAngle();
	}
	else
	{
		ACPH_ASSERT(false, "Unsupported");
		return false;
	}

	float translation;
	if (mRackConstraint->GetSubType() == EConstraintSubType::Slider)
	{
		translation = StaticCast<SliderConstraint>(mRackConstraint)->GetCurrentPosition();
	}
	else
	{
		ACPH_ASSERT(false, "Unsupported");
		return false;
	}

	float error = CenterAngleAroundZero(fmod(rotation - mRatio * translation, 2.0f * ACPH_PI));
	if (error == 0.0f)
		return false;

	Mat44 rotation1 = Mat44::sRotation(mBody1->GetRotation());
	Mat44 rotation2 = Mat44::sRotation(mBody2->GetRotation());
	CalculateConstraintProperties(rotation1, rotation2);
	return mRackAndPinionConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, error, inBaumgarte);
}

#ifdef ACPH_DEBUG_RENDERER
void RackAndPinionConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	RMat44 transform1 = mBody1->GetCenterOfMassTransform();
	RMat44 transform2 = mBody2->GetCenterOfMassTransform();

	// Draw constraint axis
	inRenderer->DrawArrow(transform1.GetTranslation(), transform1 * mLocalSpaceHingeAxis, Color::sGreen, 0.01f);
	inRenderer->DrawArrow(transform2.GetTranslation(), transform2 * mLocalSpaceSliderAxis, Color::sBlue, 0.01f);
}

#endif // ACPH_DEBUG_RENDERER

void RackAndPinionConstraint::SaveState(StateRecorder &inStream) const
{
	TwoBodyConstraint::SaveState(inStream);

	mRackAndPinionConstraintPart.SaveState(inStream);
}

void RackAndPinionConstraint::RestoreState(StateRecorder &inStream)
{
	TwoBodyConstraint::RestoreState(inStream);

	mRackAndPinionConstraintPart.RestoreState(inStream);
}

Ref<ConstraintSettings> RackAndPinionConstraint::GetConstraintSettings() const
{
	RackAndPinionConstraintSettings *settings = new RackAndPinionConstraintSettings;
	ToConstraintSettings(*settings);
	settings->mSpace = EConstraintSpace::LocalToBodyCOM;
	settings->mHingeAxis = mLocalSpaceHingeAxis;
	settings->mSliderAxis = mLocalSpaceSliderAxis;
	settings->mRatio = mRatio;
	return settings;
}

Mat44 RackAndPinionConstraint::GetConstraintToBody1Matrix() const
{
	Vec3 perp = mLocalSpaceHingeAxis.GetNormalizedPerpendicular();
	return Mat44(Vec4(mLocalSpaceHingeAxis, 0), Vec4(perp, 0), Vec4(mLocalSpaceHingeAxis.Cross(perp), 0), Vec4(0, 0, 0, 1));
}

Mat44 RackAndPinionConstraint::GetConstraintToBody2Matrix() const
{
	Vec3 perp = mLocalSpaceSliderAxis.GetNormalizedPerpendicular();
	return Mat44(Vec4(mLocalSpaceSliderAxis, 0), Vec4(perp, 0), Vec4(mLocalSpaceSliderAxis.Cross(perp), 0), Vec4(0, 0, 0, 1));
}

ACPH_NAMESPACE_END
