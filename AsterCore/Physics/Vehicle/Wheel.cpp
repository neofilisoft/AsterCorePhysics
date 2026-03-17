// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Vehicle/Wheel.h>
#include <AsterCore/Physics/Vehicle/VehicleConstraint.h>
#include <AsterCore/ObjectStream/TypeDeclarations.h>

ACPH_NAMESPACE_BEGIN

ACPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(WheelSettings)
{
	ACPH_ADD_ATTRIBUTE(WheelSettings, mSuspensionForcePoint)
	ACPH_ADD_ATTRIBUTE(WheelSettings, mPosition)
	ACPH_ADD_ATTRIBUTE(WheelSettings, mSuspensionDirection)
	ACPH_ADD_ATTRIBUTE(WheelSettings, mSteeringAxis)
	ACPH_ADD_ATTRIBUTE(WheelSettings, mWheelForward)
	ACPH_ADD_ATTRIBUTE(WheelSettings, mWheelUp)
	ACPH_ADD_ATTRIBUTE(WheelSettings, mSuspensionMinLength)
	ACPH_ADD_ATTRIBUTE(WheelSettings, mSuspensionMaxLength)
	ACPH_ADD_ATTRIBUTE(WheelSettings, mSuspensionPreloadLength)
	ACPH_ADD_ENUM_ATTRIBUTE_WITH_ALIAS(WheelSettings, mSuspensionSpring.mMode, "mSuspensionSpringMode")
	ACPH_ADD_ATTRIBUTE_WITH_ALIAS(WheelSettings, mSuspensionSpring.mFrequency, "mSuspensionFrequency") // Renaming attributes to stay compatible with old versions of the library
	ACPH_ADD_ATTRIBUTE_WITH_ALIAS(WheelSettings, mSuspensionSpring.mDamping, "mSuspensionDamping")
	ACPH_ADD_ATTRIBUTE(WheelSettings, mRadius)
	ACPH_ADD_ATTRIBUTE(WheelSettings, mWidth)
	ACPH_ADD_ATTRIBUTE(WheelSettings, mEnableSuspensionForcePoint)
}

void WheelSettings::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(mSuspensionForcePoint);
	inStream.Write(mPosition);
	inStream.Write(mSuspensionDirection);
	inStream.Write(mSteeringAxis);
	inStream.Write(mWheelForward);
	inStream.Write(mWheelUp);
	inStream.Write(mSuspensionMinLength);
	inStream.Write(mSuspensionMaxLength);
	inStream.Write(mSuspensionPreloadLength);
	mSuspensionSpring.SaveBinaryState(inStream);
	inStream.Write(mRadius);
	inStream.Write(mWidth);
	inStream.Write(mEnableSuspensionForcePoint);
}

void WheelSettings::RestoreBinaryState(StreamIn &inStream)
{
	inStream.Read(mSuspensionForcePoint);
	inStream.Read(mPosition);
	inStream.Read(mSuspensionDirection);
	inStream.Read(mSteeringAxis);
	inStream.Read(mWheelForward);
	inStream.Read(mWheelUp);
	inStream.Read(mSuspensionMinLength);
	inStream.Read(mSuspensionMaxLength);
	inStream.Read(mSuspensionPreloadLength);
	mSuspensionSpring.RestoreBinaryState(inStream);
	inStream.Read(mRadius);
	inStream.Read(mWidth);
	inStream.Read(mEnableSuspensionForcePoint);
}

Wheel::Wheel(const WheelSettings &inSettings) :
	mSettings(&inSettings),
	mSuspensionLength(inSettings.mSuspensionMaxLength)
{
	ACPH_ASSERT(inSettings.mSuspensionDirection.IsNormalized());
	ACPH_ASSERT(inSettings.mSteeringAxis.IsNormalized());
	ACPH_ASSERT(inSettings.mWheelForward.IsNormalized());
	ACPH_ASSERT(inSettings.mWheelUp.IsNormalized());
	ACPH_ASSERT(inSettings.mSuspensionMinLength >= 0.0f);
	ACPH_ASSERT(inSettings.mSuspensionMaxLength >= inSettings.mSuspensionMinLength);
	ACPH_ASSERT(inSettings.mSuspensionPreloadLength >= 0.0f);
	ACPH_ASSERT(inSettings.mSuspensionSpring.mFrequency > 0.0f);
	ACPH_ASSERT(inSettings.mSuspensionSpring.mDamping >= 0.0f);
	ACPH_ASSERT(inSettings.mRadius > 0.0f);
	ACPH_ASSERT(inSettings.mWidth >= 0.0f);
}

bool Wheel::SolveLongitudinalConstraintPart(const VehicleConstraint &inConstraint, float inMinImpulse, float inMaxImpulse)
{
	return mLongitudinalPart.SolveVelocityConstraint(*inConstraint.GetVehicleBody(), *mContactBody, -mContactLongitudinal, inMinImpulse, inMaxImpulse);
}

bool Wheel::SolveLateralConstraintPart(const VehicleConstraint &inConstraint, float inMinImpulse, float inMaxImpulse)
{
	return mLateralPart.SolveVelocityConstraint(*inConstraint.GetVehicleBody(), *mContactBody, -mContactLateral, inMinImpulse, inMaxImpulse);
}

ACPH_NAMESPACE_END
