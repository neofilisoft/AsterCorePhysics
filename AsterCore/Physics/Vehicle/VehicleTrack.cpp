// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Vehicle/VehicleTrack.h>
#include <AsterCore/ObjectStream/TypeDeclarations.h>

ACPH_NAMESPACE_BEGIN

ACPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleTrackSettings)
{
	ACPH_ADD_ATTRIBUTE(VehicleTrackSettings, mDrivenWheel)
	ACPH_ADD_ATTRIBUTE(VehicleTrackSettings, mWheels)
	ACPH_ADD_ATTRIBUTE(VehicleTrackSettings, mInertia)
	ACPH_ADD_ATTRIBUTE(VehicleTrackSettings, mAngularDamping)
	ACPH_ADD_ATTRIBUTE(VehicleTrackSettings, mMaxBrakeTorque)
	ACPH_ADD_ATTRIBUTE(VehicleTrackSettings, mDifferentialRatio)
}

void VehicleTrackSettings::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(mDrivenWheel);
	inStream.Write(mWheels);
	inStream.Write(mInertia);
	inStream.Write(mAngularDamping);
	inStream.Write(mMaxBrakeTorque);
	inStream.Write(mDifferentialRatio);
}

void VehicleTrackSettings::RestoreBinaryState(StreamIn &inStream)
{
	inStream.Read(mDrivenWheel);
	inStream.Read(mWheels);
	inStream.Read(mInertia);
	inStream.Read(mAngularDamping);
	inStream.Read(mMaxBrakeTorque);
	inStream.Read(mDifferentialRatio);
}

void VehicleTrack::SaveState(StateRecorder &inStream) const
{
	inStream.Write(mAngularVelocity);
}

void VehicleTrack::RestoreState(StateRecorder &inStream)
{
	inStream.Read(mAngularVelocity);
}

ACPH_NAMESPACE_END
