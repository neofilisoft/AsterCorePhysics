// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>

#include <AsterCore/Core/Array.h>
#include <AsterCore/Core/UnorderedMap.h>
#include <AsterCore/Physics/Body/BodyID.h>
#include <AsterCore/Physics/Destruction/PreFracturedMesh.h>
#include <AsterCore/Physics/EActivation.h>
#include <AsterCore/Physics/Snapshot/PhysicsSnapshotParticipant.h>

ACPH_NAMESPACE_BEGIN

class BodyInterface;
class StateRecorder;

enum class EDestructionEventType : uint8
{
	Fractured,
	ChunkReleased,
	BuildingCollapsed
};

struct ACPH_EXPORT DestructionConnection
{
	uint32						mChunkA = 0;
	uint32						mChunkB = 0;
	float						mBreakDamageThreshold = 35.0f;
};

struct ACPH_EXPORT BreakableBodySettings
{
	PreFracturedMesh			mPreFracturedMesh;
	Array<DestructionConnection> mConnections;
	float						mFractureDamageThreshold = 125.0f;
	float						mFractureImpulseThreshold = 60.0f;
	float						mDamageToImpulseScale = 0.4f;
	float						mAngularImpulseScale = 0.075f;
	float						mPendingDamageWeight = 1.0f;
	float						mPendingImpulseWeight = 1.0f;
	float						mPendingRadiusWeight = 0.25f;
	float						mPendingForceFractureBias = 100000.0f;
	bool						mSpawnSupportedChunksAsStatic = true;
	bool						mInheritIntactLinearVelocity = true;
	bool						mInheritIntactAngularVelocity = true;
};

struct ACPH_EXPORT DestructionDamage
{
	RVec3						mPosition = RVec3::sZero();
	Vec3						mImpulse = Vec3::sZero();
	float						mDamage = 0.0f;
	float						mRadius = 0.0f;
	bool						mForceFracture = false;
};

struct ACPH_EXPORT DestructionEvent
{
	EDestructionEventType		mType = EDestructionEventType::Fractured;
	BodyID						mSourceBodyID;
	BodyID						mChunkBodyID;
	uint32						mChunkIndex = 0xffffffff;
};

class ACPH_EXPORT DestructionSystem final : public NonCopyable, public PhysicsSnapshotParticipant
{
public:
	void						Clear();
	bool						RegisterBreakableBody(const BodyID &inBodyID, const BreakableBodySettings &inSettings);
	bool						UnregisterBreakableBody(const BodyID &inBodyID);
	bool						HasBreakableBody(const BodyID &inBodyID) const;
	bool						IsFractured(const BodyID &inBodyID) const;

	bool						ApplyDamage(BodyInterface &inBodyInterface, const BodyID &inBodyID, const DestructionDamage &inDamage, EActivation inActivationMode = EActivation::Activate);
	bool						ProcessPendingFractures(BodyInterface &inBodyInterface, EActivation inActivationMode = EActivation::Activate);
	bool						BreakConnection(BodyInterface &inBodyInterface, const BodyID &inBodyID, uint32 inConnectionIndex, Vec3Arg inExtraImpulse = Vec3::sZero(), EActivation inActivationMode = EActivation::Activate);
	bool						ReleaseChunk(BodyInterface &inBodyInterface, const BodyID &inBodyID, uint32 inChunkIndex, Vec3Arg inExtraImpulse = Vec3::sZero(), EActivation inActivationMode = EActivation::Activate);

	Array<BodyID>				GetChunkBodies(const BodyID &inBodyID) const;
	Array<DestructionEvent>		ConsumeEvents();

	virtual uint32				GetSnapshotParticipantTag() const override;
	virtual void				SaveSnapshotState(StateRecorder &inStream, const BodyInterface &inBodyInterface) const override;
	virtual bool				RestoreSnapshotState(StateRecorder &inStream, BodyInterface &inBodyInterface) override;

private:
	struct BreakableBodyState
	{
		BreakableBodySettings	mSettings;
		float					mAccumulatedDamage = 0.0f;
		float					mPendingFractureScore = 0.0f;
		bool					mHasPendingDamage = false;
		bool					mPendingFracture = false;
		bool					mIsFractured = false;
		bool					mCollapseEventSent = false;
		bool					mHasIntactBodyTemplate = false;
		bool					mIntactBodyAdded = false;
		DestructionDamage		mPendingDamage;
		BodyCreationSettings	mIntactBodyTemplate;
		Array<BodyID>			mChunkBodies;
		Array<uint8>			mChunkReleased;
		Array<uint8>			mConnectionBroken;
		Vec3					mInheritedLinearVelocity = Vec3::sZero();
		Vec3					mInheritedAngularVelocity = Vec3::sZero();
		RVec3					mFracturePosition = RVec3::sZero();
		Quat					mFractureRotation = Quat::sIdentity();
	};

	BreakableBodyState *		FindMutable(const BodyID &inBodyID);
	const BreakableBodyState *	Find(const BodyID &inBodyID) const;
	bool						ShouldFracture(const BreakableBodyState &inState, const DestructionDamage &inDamage) const;
	float						ComputeFractureScore(const BreakableBodyState &inState, const DestructionDamage &inDamage) const;
	bool						HasAnchoredChunks(const BreakableBodyState &inState) const;
	bool							AllocateDeterministicChunkBodyIDs(const BodyInterface &inBodyInterface, const BodyID &inBodyID, uint32 inChunkCount, Array<BodyID> &outChunkBodyIDs) const;
	void						CaptureIntactBodyTemplate(const BodyInterface &inBodyInterface, const BodyID &inBodyID, BreakableBodyState &ioState) const;
	bool						FractureBody(BodyInterface &inBodyInterface, const BodyID &inBodyID, BreakableBodyState &ioState, EActivation inActivationMode);
	void						BreakConnectionsByDamage(BodyInterface *inBodyInterface, const BreakableBodyState &inState, const DestructionDamage &inDamage, Array<uint8> &ioConnectionBroken) const;
	void						EvaluateReleasedChunks(BodyInterface &inBodyInterface, const BodyID &inBodyID, BreakableBodyState &ioState, const DestructionDamage &inDamage, Vec3Arg inExtraImpulse, EActivation inActivationMode);
	void						ReleaseChunkInternal(BodyInterface &inBodyInterface, const BodyID &inBodyID, BreakableBodyState &ioState, uint32 inChunkIndex, const DestructionDamage &inDamage, Vec3Arg inExtraImpulse, EActivation inActivationMode);
	Vec3						ComputeChunkImpulse(BodyInterface &inBodyInterface, const BreakableBodyState &inState, uint32 inChunkIndex, const DestructionDamage &inDamage) const;
	void						DestroyBodyIfPresent(BodyInterface &inBodyInterface, const BodyID &inBodyID) const;
	void						DestroyBodiesIfPresent(BodyInterface &inBodyInterface, const Array<BodyID> &inBodyIDs) const;
	bool						RestoreIntactTopology(BodyInterface &inBodyInterface, const BodyID &inBodyID, const Array<BodyID> &inExistingChunkBodies, BreakableBodyState &ioState);
	bool						RestoreFracturedTopology(BodyInterface &inBodyInterface, const BodyID &inBodyID, const Array<BodyID> &inExistingChunkBodies, BreakableBodyState &ioState);
	EMotionType					GetChunkMotionTypeForState(const BreakableBodyState &inState, uint32 inChunkIndex) const;

	UnorderedMap<BodyID, BreakableBodyState> mBreakableBodies;
	Array<DestructionEvent>		mEvents;
};

ACPH_NAMESPACE_END
