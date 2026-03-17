// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Collision/CollisionGroup.h>
#include <AsterCore/ObjectStream/TypeDeclarations.h>
#include <AsterCore/Core/StreamIn.h>
#include <AsterCore/Core/StreamOut.h>

ACPH_NAMESPACE_BEGIN

ACPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(CollisionGroup)
{
	ACPH_ADD_ATTRIBUTE(CollisionGroup, mGroupFilter)
	ACPH_ADD_ATTRIBUTE(CollisionGroup, mGroupID)
	ACPH_ADD_ATTRIBUTE(CollisionGroup, mSubGroupID)
}

const CollisionGroup CollisionGroup::sInvalid;

void CollisionGroup::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(mGroupID);
	inStream.Write(mSubGroupID);
}

void CollisionGroup::RestoreBinaryState(StreamIn &inStream)
{
	inStream.Read(mGroupID);
	inStream.Read(mSubGroupID);
}

ACPH_NAMESPACE_END
