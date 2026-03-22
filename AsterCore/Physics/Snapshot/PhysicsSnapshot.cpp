// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#include <AsterCore/Physics/Snapshot/PhysicsSnapshot.h>

#include <AsterCore/Physics/Body/BodyInterface.h>
#include <AsterCore/Physics/PhysicsSystem.h>

ACPH_SUPPRESS_WARNINGS_STD_BEGIN
#include <algorithm>
ACPH_SUPPRESS_WARNINGS_STD_END

ACPH_NAMESPACE_BEGIN

void PhysicsSnapshot::AddParticipant(PhysicsSnapshotParticipant *inParticipant)
{
	ACPH_ASSERT(inParticipant != nullptr);
	if (std::find(mParticipants.begin(), mParticipants.end(), inParticipant) == mParticipants.end())
		mParticipants.push_back(inParticipant);
}

void PhysicsSnapshot::RemoveParticipant(PhysicsSnapshotParticipant *inParticipant)
{
	Array<PhysicsSnapshotParticipant *>::iterator it = std::find(mParticipants.begin(), mParticipants.end(), inParticipant);
	if (it != mParticipants.end())
		mParticipants.erase(it);
}

void PhysicsSnapshot::ClearParticipants()
{
	mParticipants.clear();
}

void PhysicsSnapshot::Save(StateRecorder &inStream, const PhysicsSystem &inPhysicsSystem, EStateRecorderState inState, const StateRecorderFilter *inFilter) const
{
	inStream.Write(cMagic);
	inStream.Write(cVersion);

	const uint32 num_participants = uint32(mParticipants.size());
	inStream.Write(num_participants);
	for (const PhysicsSnapshotParticipant *participant : mParticipants)
	{
		const uint32 tag = participant->GetSnapshotParticipantTag();
		inStream.Write(tag);
		participant->SaveSnapshotState(inStream, inPhysicsSystem.GetBodyInterface());
	}

	inPhysicsSystem.SaveState(inStream, inState, inFilter);
}

bool PhysicsSnapshot::Restore(StateRecorder &inStream, PhysicsSystem &ioPhysicsSystem, const StateRecorderFilter *inFilter) const
{
	uint32 magic = 0;
	inStream.Read(magic);
	if (magic != cMagic)
		return false;

	uint32 version = 0;
	inStream.Read(version);
	if (version != cVersion)
		return false;

	uint32 num_participants = 0;
	inStream.Read(num_participants);
	if (num_participants != mParticipants.size())
		return false;

	for (uint32 participant_index = 0; participant_index < num_participants; ++participant_index)
	{
		uint32 tag = 0;
		inStream.Read(tag);
		if (tag != mParticipants[participant_index]->GetSnapshotParticipantTag())
			return false;

		if (!mParticipants[participant_index]->RestoreSnapshotState(inStream, ioPhysicsSystem.GetBodyInterface()))
			return false;
	}

	return ioPhysicsSystem.RestoreState(inStream, inFilter);
}

ACPH_NAMESPACE_END
