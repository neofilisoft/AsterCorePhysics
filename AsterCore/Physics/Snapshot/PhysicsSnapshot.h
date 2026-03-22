// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>

#include <AsterCore/Core/Array.h>
#include <AsterCore/Core/NonCopyable.h>
#include <AsterCore/Physics/Snapshot/PhysicsSnapshotParticipant.h>
#include <AsterCore/Physics/StateRecorder.h>

ACPH_NAMESPACE_BEGIN

class PhysicsSystem;

/// High level snapshot helper that captures the physics system together with optional runtime systems.
/// This is intended for rollback / replay flows where body topology may need to be prepared before restoring the core physics state.
class ACPH_EXPORT PhysicsSnapshot final : public NonCopyable
{
public:
	void						AddParticipant(PhysicsSnapshotParticipant *inParticipant);
	void						RemoveParticipant(PhysicsSnapshotParticipant *inParticipant);
	void						ClearParticipants();

	void						Save(StateRecorder &inStream, const PhysicsSystem &inPhysicsSystem, EStateRecorderState inState = EStateRecorderState::All, const StateRecorderFilter *inFilter = nullptr) const;
	bool						Restore(StateRecorder &inStream, PhysicsSystem &ioPhysicsSystem, const StateRecorderFilter *inFilter = nullptr) const;

private:
	static constexpr uint32		cMagic = 0x41435053; // 'ACPS'
	static constexpr uint32		cVersion = 1;

	Array<PhysicsSnapshotParticipant *> mParticipants;
};

ACPH_NAMESPACE_END
