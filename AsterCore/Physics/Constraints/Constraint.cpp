// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Constraints/Constraint.h>
#include <AsterCore/Physics/StateRecorder.h>
#include <AsterCore/ObjectStream/TypeDeclarations.h>
#include <AsterCore/Core/StreamUtils.h>

ACPH_NAMESPACE_BEGIN

ACPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(ConstraintSettings)
{
	ACPH_ADD_BASE_CLASS(ConstraintSettings, SerializableObject)

	ACPH_ADD_ATTRIBUTE(ConstraintSettings, mEnabled)
	ACPH_ADD_ATTRIBUTE(ConstraintSettings, mDrawConstraintSize)
	ACPH_ADD_ATTRIBUTE(ConstraintSettings, mConstraintPriority)
	ACPH_ADD_ATTRIBUTE(ConstraintSettings, mNumVelocityStepsOverride)
	ACPH_ADD_ATTRIBUTE(ConstraintSettings, mNumPositionStepsOverride)
	ACPH_ADD_ATTRIBUTE(ConstraintSettings, mUserData)
}

void ConstraintSettings::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(GetRTTI()->GetHash());
	inStream.Write(mEnabled);
	inStream.Write(mDrawConstraintSize);
	inStream.Write(mConstraintPriority);
	inStream.Write(mNumVelocityStepsOverride);
	inStream.Write(mNumPositionStepsOverride);
}

void ConstraintSettings::RestoreBinaryState(StreamIn &inStream)
{
	// Type hash read by sRestoreFromBinaryState
	inStream.Read(mEnabled);
	inStream.Read(mDrawConstraintSize);
	inStream.Read(mConstraintPriority);
	inStream.Read(mNumVelocityStepsOverride);
	inStream.Read(mNumPositionStepsOverride);
}

ConstraintSettings::ConstraintResult ConstraintSettings::sRestoreFromBinaryState(StreamIn &inStream)
{
	return StreamUtils::RestoreObject<ConstraintSettings>(inStream, &ConstraintSettings::RestoreBinaryState);
}

void Constraint::SaveState(StateRecorder &inStream) const
{
	inStream.Write(mEnabled);
}

void Constraint::RestoreState(StateRecorder &inStream)
{
	inStream.Read(mEnabled);
}

void Constraint::ToConstraintSettings(ConstraintSettings &outSettings) const
{
	outSettings.mEnabled = mEnabled;
	outSettings.mConstraintPriority = mConstraintPriority;
	outSettings.mNumVelocityStepsOverride = mNumVelocityStepsOverride;
	outSettings.mNumPositionStepsOverride = mNumPositionStepsOverride;
	outSettings.mUserData = mUserData;
#ifdef ACPH_DEBUG_RENDERER
	outSettings.mDrawConstraintSize = mDrawConstraintSize;
#endif // ACPH_DEBUG_RENDERER
}

ACPH_NAMESPACE_END
