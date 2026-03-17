// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Constraints/MotorSettings.h>
#include <AsterCore/ObjectStream/TypeDeclarations.h>
#include <AsterCore/Core/StreamIn.h>
#include <AsterCore/Core/StreamOut.h>

ACPH_NAMESPACE_BEGIN

ACPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(MotorSettings)
{
	ACPH_ADD_ENUM_ATTRIBUTE_WITH_ALIAS(MotorSettings, mSpringSettings.mMode, "mSpringMode")
	ACPH_ADD_ATTRIBUTE_WITH_ALIAS(MotorSettings, mSpringSettings.mFrequency, "mFrequency") // Renaming attributes to stay compatible with old versions of the library
	ACPH_ADD_ATTRIBUTE_WITH_ALIAS(MotorSettings, mSpringSettings.mDamping, "mDamping")
	ACPH_ADD_ATTRIBUTE(MotorSettings, mMinForceLimit)
	ACPH_ADD_ATTRIBUTE(MotorSettings, mMaxForceLimit)
	ACPH_ADD_ATTRIBUTE(MotorSettings, mMinTorqueLimit)
	ACPH_ADD_ATTRIBUTE(MotorSettings, mMaxTorqueLimit)
}

void MotorSettings::SaveBinaryState(StreamOut &inStream) const
{
	mSpringSettings.SaveBinaryState(inStream);
	inStream.Write(mMinForceLimit);
	inStream.Write(mMaxForceLimit);
	inStream.Write(mMinTorqueLimit);
	inStream.Write(mMaxTorqueLimit);
}

void MotorSettings::RestoreBinaryState(StreamIn &inStream)
{
	mSpringSettings.RestoreBinaryState(inStream);
	inStream.Read(mMinForceLimit);
	inStream.Read(mMaxForceLimit);
	inStream.Read(mMinTorqueLimit);
	inStream.Read(mMaxTorqueLimit);
}

ACPH_NAMESPACE_END
