// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Collision/PhysicsMaterial.h>
#include <AsterCore/Physics/Collision/PhysicsMaterialSimple.h>
#include <AsterCore/Core/StreamUtils.h>

ACPH_NAMESPACE_BEGIN

RefConst<PhysicsMaterial> PhysicsMaterial::sDefault;

ACPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(PhysicsMaterial)
{
	ACPH_ADD_BASE_CLASS(PhysicsMaterial, SerializableObject)
}

void PhysicsMaterial::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(GetRTTI()->GetHash());
}

void PhysicsMaterial::RestoreBinaryState(StreamIn &inStream)
{
	// RTTI hash is read in sRestoreFromBinaryState
}

PhysicsMaterial::PhysicsMaterialResult PhysicsMaterial::sRestoreFromBinaryState(StreamIn &inStream)
{
	return StreamUtils::RestoreObject<PhysicsMaterial>(inStream, &PhysicsMaterial::RestoreBinaryState);
}

ACPH_NAMESPACE_END
