// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Collision/GroupFilter.h>
#include <AsterCore/Core/StreamUtils.h>

ACPH_NAMESPACE_BEGIN

ACPH_IMPLEMENT_SERIALIZABLE_ABSTRACT_BASE(GroupFilter)
{
	ACPH_ADD_BASE_CLASS(GroupFilter, SerializableObject)
}

void GroupFilter::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(GetRTTI()->GetHash());
}

void GroupFilter::RestoreBinaryState(StreamIn &inStream)
{
	// RTTI hash is read in sRestoreFromBinaryState
}

GroupFilter::GroupFilterResult GroupFilter::sRestoreFromBinaryState(StreamIn &inStream)
{
	return StreamUtils::RestoreObject<GroupFilter>(inStream, &GroupFilter::RestoreBinaryState);
}

ACPH_NAMESPACE_END
