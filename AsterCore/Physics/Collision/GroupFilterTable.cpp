// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Collision/GroupFilterTable.h>
#include <AsterCore/ObjectStream/TypeDeclarations.h>
#include <AsterCore/Core/StreamIn.h>
#include <AsterCore/Core/StreamOut.h>

ACPH_NAMESPACE_BEGIN

ACPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(GroupFilterTable)
{
	ACPH_ADD_BASE_CLASS(GroupFilterTable, GroupFilter)

	ACPH_ADD_ATTRIBUTE(GroupFilterTable, mNumSubGroups)
	ACPH_ADD_ATTRIBUTE(GroupFilterTable, mTable)
}

void GroupFilterTable::SaveBinaryState(StreamOut &inStream) const
{
	GroupFilter::SaveBinaryState(inStream);

	inStream.Write(mNumSubGroups);
	inStream.Write(mTable);
}

void GroupFilterTable::RestoreBinaryState(StreamIn &inStream)
{
	GroupFilter::RestoreBinaryState(inStream);

	inStream.Read(mNumSubGroups);
	inStream.Read(mTable);
}

ACPH_NAMESPACE_END
