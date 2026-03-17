// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Collision/PhysicsMaterialSimple.h>
#include <AsterCore/ObjectStream/TypeDeclarations.h>
#include <AsterCore/Core/StreamIn.h>
#include <AsterCore/Core/StreamOut.h>

ACPH_NAMESPACE_BEGIN

ACPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(PhysicsMaterialSimple)
{
	ACPH_ADD_BASE_CLASS(PhysicsMaterialSimple, PhysicsMaterial)

	ACPH_ADD_ATTRIBUTE(PhysicsMaterialSimple, mDebugName)
	ACPH_ADD_ATTRIBUTE(PhysicsMaterialSimple, mDebugColor)
}

void PhysicsMaterialSimple::SaveBinaryState(StreamOut &inStream) const
{
	PhysicsMaterial::SaveBinaryState(inStream);

	inStream.Write(mDebugName);
	inStream.Write(mDebugColor);
}

void PhysicsMaterialSimple::RestoreBinaryState(StreamIn &inStream)
{
	PhysicsMaterial::RestoreBinaryState(inStream);

	inStream.Read(mDebugName);
	inStream.Read(mDebugColor);
}

ACPH_NAMESPACE_END
