// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2023 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Constraints/SpringSettings.h>
#include <AsterCore/ObjectStream/TypeDeclarations.h>
#include <AsterCore/Core/StreamIn.h>
#include <AsterCore/Core/StreamOut.h>

ACPH_NAMESPACE_BEGIN

ACPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SpringSettings)
{
	ACPH_ADD_ENUM_ATTRIBUTE(SpringSettings, mMode)
	ACPH_ADD_ATTRIBUTE(SpringSettings, mFrequency)
	ACPH_ADD_ATTRIBUTE(SpringSettings, mDamping)
}

void SpringSettings::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(mMode);
	inStream.Write(mFrequency);
	inStream.Write(mDamping);
}

void SpringSettings::RestoreBinaryState(StreamIn &inStream)
{
	inStream.Read(mMode);
	inStream.Read(mFrequency);
	inStream.Read(mDamping);
}

ACPH_NAMESPACE_END
