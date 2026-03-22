// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>

ACPH_NAMESPACE_BEGIN

class BodyInterface;
class StateRecorder;

/// Extension point for systems that need to participate in rollback / replay snapshots.
/// Implementations should keep configuration stable between save and restore and only serialize runtime state here.
class ACPH_EXPORT PhysicsSnapshotParticipant
{
public:
	virtual						~PhysicsSnapshotParticipant() = default;

	virtual uint32				GetSnapshotParticipantTag() const = 0;
	virtual void				SaveSnapshotState(StateRecorder &inStream, const BodyInterface &inBodyInterface) const = 0;
	virtual bool				RestoreSnapshotState(StateRecorder &inStream, BodyInterface &inBodyInterface) = 0;
};

ACPH_NAMESPACE_END
