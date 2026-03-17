// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Vehicle/VehicleAntiRollBar.h>
#include <AsterCore/ObjectStream/TypeDeclarations.h>

ACPH_NAMESPACE_BEGIN

ACPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleAntiRollBar)
{
	ACPH_ADD_ATTRIBUTE(VehicleAntiRollBar, mLeftWheel)
	ACPH_ADD_ATTRIBUTE(VehicleAntiRollBar, mRightWheel)
	ACPH_ADD_ATTRIBUTE(VehicleAntiRollBar, mStiffness)
}

void VehicleAntiRollBar::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(mLeftWheel);
	inStream.Write(mRightWheel);
	inStream.Write(mStiffness);
}

void VehicleAntiRollBar::RestoreBinaryState(StreamIn &inStream)
{
	inStream.Read(mLeftWheel);
	inStream.Read(mRightWheel);
	inStream.Read(mStiffness);
}

ACPH_NAMESPACE_END
