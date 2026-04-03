// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#include <AsterCore/Physics/Destruction/DestructionSystem.h>

#include <AsterCore/Physics/Body/Body.h>
#include <AsterCore/Physics/Body/BodyInterface.h>
#include <AsterCore/Physics/Collision/PhysicsMaterial.h>
#include <AsterCore/Physics/Snapshot/PhysicsSnapshotTags.h>
#include <AsterCore/Physics/StateRecorder.h>

ACPH_SUPPRESS_WARNINGS_STD_BEGIN
#include <algorithm>
ACPH_SUPPRESS_WARNINGS_STD_END

ACPH_NAMESPACE_BEGIN

namespace
{
	constexpr float cMinVectorLengthSq = 1.0e-12f;
	constexpr uint8 cSnapshotHasPendingDamageBit = 1 << 0;
	constexpr uint8 cSnapshotPendingFractureBit = 1 << 1;
	constexpr uint8 cSnapshotIsFracturedBit = 1 << 2;
	constexpr uint8 cSnapshotCollapseEventSentBit = 1 << 3;
	constexpr uint8 cSnapshotIntactBodyAddedBit = 1 << 4;
	constexpr uint8 cSnapshotHasIntactTemplateBit = 1 << 5;

	static void sWriteDamage(StateRecorder &inStream, const DestructionDamage &inDamage)
	{
		inStream.Write(inDamage.mPosition);
		inStream.Write(inDamage.mImpulse);
		inStream.Write(inDamage.mDamage);
		inStream.Write(inDamage.mRadius);
		inStream.Write(inDamage.mForceFracture);
	}

	static void sReadDamage(StateRecorder &inStream, DestructionDamage &outDamage)
	{
		inStream.Read(outDamage.mPosition);
		inStream.Read(outDamage.mImpulse);
		inStream.Read(outDamage.mDamage);
		inStream.Read(outDamage.mRadius);
		inStream.Read(outDamage.mForceFracture);
	}

	template <class T>
	static void sWriteArray(StateRecorder &inStream, const Array<T> &inValues)
	{
		const uint32 count = uint32(inValues.size());
		inStream.Write(count);
		for (const T &value : inValues)
			inStream.Write(value);
	}

	template <class T>
	static bool sReadArray(StateRecorder &inStream, Array<T> &outValues)
	{
		uint32 count = 0;
		inStream.Read(count);
		outValues.resize(count);
	for (uint32 index = 0; index < count; ++index)
		inStream.Read(outValues[index]);
	return !static_cast<StreamIn &>(inStream).IsFailed();
	}
}

void DestructionSystem::Clear()
{
	mBreakableBodies.clear();
	mEvents.clear();
}

bool DestructionSystem::RegisterBreakableBody(const BodyID &inBodyID, const BreakableBodySettings &inSettings)
{
	if (!inSettings.mPreFracturedMesh.IsValid())
		return false;

	const uint32 chunk_count = inSettings.mPreFracturedMesh.GetChunkCount();
	for (const DestructionConnection &connection : inSettings.mConnections)
		if (connection.mChunkA >= chunk_count || connection.mChunkB >= chunk_count)
			return false;

	auto inserted = mBreakableBodies.try_emplace(inBodyID);
	BreakableBodyState &state = inserted.first->second;
	state = BreakableBodyState();
	state.mSettings = inSettings;
	state.mChunkBodies.reserve(chunk_count);
	state.mChunkReleased.reserve(chunk_count);
	state.mConnectionBroken.reserve(inSettings.mConnections.size());
	return true;
}

bool DestructionSystem::UnregisterBreakableBody(const BodyID &inBodyID)
{
	return mBreakableBodies.erase(inBodyID) != 0;
}

bool DestructionSystem::HasBreakableBody(const BodyID &inBodyID) const
{
	return Find(inBodyID) != nullptr;
}

bool DestructionSystem::IsFractured(const BodyID &inBodyID) const
{
	const BreakableBodyState *state = Find(inBodyID);
	return state != nullptr && state->mIsFractured;
}

bool DestructionSystem::ApplyDamage(BodyInterface &inBodyInterface, const BodyID &inBodyID, const DestructionDamage &inDamage, EActivation inActivationMode)
{
	BreakableBodyState *state = FindMutable(inBodyID);
	if (state == nullptr)
		return false;

	if (!state->mIsFractured)
	{
		state->mAccumulatedDamage += inDamage.mDamage;

		const float fracture_score = ComputeFractureScore(*state, inDamage);
		const bool replace_pending_damage = !state->mHasPendingDamage
			|| (inDamage.mForceFracture && !state->mPendingDamage.mForceFracture)
			|| (inDamage.mForceFracture == state->mPendingDamage.mForceFracture && fracture_score > state->mPendingFractureScore);
		if (replace_pending_damage)
		{
			state->mPendingDamage = inDamage;
			state->mPendingFractureScore = fracture_score;
			state->mHasPendingDamage = true;
		}

		if (ShouldFracture(*state, inDamage))
			state->mPendingFracture = true;
		return true;
	}

	BreakConnectionsByDamage(&inBodyInterface, *state, inDamage, state->mConnectionBroken);
	EvaluateReleasedChunks(inBodyInterface, inBodyID, *state, inDamage, Vec3::sZero(), inActivationMode);
	return true;
}

bool DestructionSystem::ProcessPendingFractures(BodyInterface &inBodyInterface, EActivation inActivationMode)
{
	bool fractured_any = false;
	for (auto it = mBreakableBodies.begin(); it != mBreakableBodies.end(); ++it)
		if (it->second.mPendingFracture && !it->second.mIsFractured)
			fractured_any = FractureBody(inBodyInterface, it->first, it->second, inActivationMode) || fractured_any;
	return fractured_any;
}

bool DestructionSystem::BreakConnection(BodyInterface &inBodyInterface, const BodyID &inBodyID, uint32 inConnectionIndex, Vec3Arg inExtraImpulse, EActivation inActivationMode)
{
	BreakableBodyState *state = FindMutable(inBodyID);
	if (state == nullptr || !state->mIsFractured || inConnectionIndex >= state->mConnectionBroken.size() || state->mConnectionBroken[inConnectionIndex] != 0)
		return false;

	state->mConnectionBroken[inConnectionIndex] = 1;
	EvaluateReleasedChunks(inBodyInterface, inBodyID, *state, DestructionDamage(), inExtraImpulse, inActivationMode);
	return true;
}

bool DestructionSystem::ReleaseChunk(BodyInterface &inBodyInterface, const BodyID &inBodyID, uint32 inChunkIndex, Vec3Arg inExtraImpulse, EActivation inActivationMode)
{
	BreakableBodyState *state = FindMutable(inBodyID);
	if (state == nullptr || !state->mIsFractured || inChunkIndex >= state->mChunkBodies.size() || state->mChunkReleased[inChunkIndex] != 0)
		return false;

	ReleaseChunkInternal(inBodyInterface, inBodyID, *state, inChunkIndex, DestructionDamage(), inExtraImpulse, inActivationMode);
	return true;
}

Array<BodyID> DestructionSystem::GetChunkBodies(const BodyID &inBodyID) const
{
	const BreakableBodyState *state = Find(inBodyID);
	return state != nullptr ? state->mChunkBodies : Array<BodyID>();
}

Array<DestructionEvent> DestructionSystem::ConsumeEvents()
{
	Array<DestructionEvent> result;
	result.swap(mEvents);
	return result;
}

uint32 DestructionSystem::GetSnapshotParticipantTag() const
{
	return PhysicsSnapshotTags::cDestructionSystem;
}

void DestructionSystem::SaveSnapshotState(StateRecorder &inStream, const BodyInterface &inBodyInterface) const
{
	Array<BodyID> body_ids;
	body_ids.reserve(mBreakableBodies.size());
	for (const auto &entry : mBreakableBodies)
		body_ids.push_back(entry.first);
	std::sort(body_ids.begin(), body_ids.end());

	inStream.Write(uint32(body_ids.size()));

	BodyCreationSettings::ShapeToIDMap shape_map;
	BodyCreationSettings::MaterialToIDMap material_map;
	BodyCreationSettings::GroupFilterToIDMap group_filter_map;

	for (const BodyID &body_id : body_ids)
	{
		const BreakableBodyState *state = Find(body_id);
		ACPH_ASSERT(state != nullptr);
		inStream.Write(body_id);
		inStream.Write(state->mAccumulatedDamage);
		inStream.Write(state->mPendingFractureScore);

		BodyCreationSettings intact_template;
		bool intact_body_added = state->mIntactBodyAdded;
		bool has_intact_template = false;
		if (state->mIsFractured)
		{
			intact_template = state->mIntactBodyTemplate;
			has_intact_template = state->mHasIntactBodyTemplate;
		}
		else if (state->mHasIntactBodyTemplate)
		{
			intact_template = state->mIntactBodyTemplate;
			has_intact_template = intact_template.GetShape() != nullptr;
		}
		else if (inBodyInterface.HasBody(body_id))
		{
			intact_template = inBodyInterface.GetBodyCreationSettings(body_id);
			has_intact_template = intact_template.GetShape() != nullptr;
			intact_body_added = inBodyInterface.IsAdded(body_id);
		}

		uint8 snapshot_flags = 0;
		if (state->mHasPendingDamage)
			snapshot_flags |= cSnapshotHasPendingDamageBit;
		if (state->mPendingFracture)
			snapshot_flags |= cSnapshotPendingFractureBit;
		if (state->mIsFractured)
			snapshot_flags |= cSnapshotIsFracturedBit;
		if (state->mCollapseEventSent)
			snapshot_flags |= cSnapshotCollapseEventSentBit;
		if (intact_body_added)
			snapshot_flags |= cSnapshotIntactBodyAddedBit;
		if (has_intact_template)
			snapshot_flags |= cSnapshotHasIntactTemplateBit;
		inStream.Write(snapshot_flags);
		sWriteDamage(inStream, state->mPendingDamage);
		inStream.Write(state->mInheritedLinearVelocity);
		inStream.Write(state->mInheritedAngularVelocity);
		inStream.Write(state->mFracturePosition);
		inStream.Write(state->mFractureRotation);
		if (has_intact_template)
			intact_template.SaveWithChildren(inStream, &shape_map, &material_map, &group_filter_map);

		sWriteArray(inStream, state->mChunkBodies);
		sWriteArray(inStream, state->mChunkReleased);
		sWriteArray(inStream, state->mConnectionBroken);
	}
}

bool DestructionSystem::RestoreSnapshotState(StateRecorder &inStream, BodyInterface &inBodyInterface)
{
	uint32 num_breakable_bodies = 0;
	inStream.Read(num_breakable_bodies);
	if (num_breakable_bodies != mBreakableBodies.size())
		return false;

	BodyCreationSettings::IDToShapeMap shape_map;
	BodyCreationSettings::IDToMaterialMap material_map;
	BodyCreationSettings::IDToGroupFilterMap group_filter_map;

	for (uint32 index = 0; index < num_breakable_bodies; ++index)
	{
		BodyID body_id;
		inStream.Read(body_id);

		BreakableBodyState *state = FindMutable(body_id);
		if (state == nullptr)
			return false;

		Array<BodyID> existing_chunk_bodies = state->mChunkBodies;

		inStream.Read(state->mAccumulatedDamage);
		inStream.Read(state->mPendingFractureScore);
		uint8 snapshot_flags = 0;
		inStream.Read(snapshot_flags);
		state->mHasPendingDamage = (snapshot_flags & cSnapshotHasPendingDamageBit) != 0;
		state->mPendingFracture = (snapshot_flags & cSnapshotPendingFractureBit) != 0;
		state->mIsFractured = (snapshot_flags & cSnapshotIsFracturedBit) != 0;
		state->mCollapseEventSent = (snapshot_flags & cSnapshotCollapseEventSentBit) != 0;
		state->mIntactBodyAdded = (snapshot_flags & cSnapshotIntactBodyAddedBit) != 0;
		state->mHasIntactBodyTemplate = (snapshot_flags & cSnapshotHasIntactTemplateBit) != 0;
		sReadDamage(inStream, state->mPendingDamage);
		inStream.Read(state->mInheritedLinearVelocity);
		inStream.Read(state->mInheritedAngularVelocity);
		inStream.Read(state->mFracturePosition);
		inStream.Read(state->mFractureRotation);

		if (state->mHasIntactBodyTemplate)
		{
			BodyCreationSettings::BCSResult intact_template = BodyCreationSettings::sRestoreWithChildren(inStream, shape_map, material_map, group_filter_map);
			if (intact_template.HasError())
				return false;
			state->mIntactBodyTemplate = intact_template.Get();
		}
		else
		{
			state->mIntactBodyTemplate = BodyCreationSettings();
		}

		if (!sReadArray(inStream, state->mChunkBodies)
			|| !sReadArray(inStream, state->mChunkReleased)
			|| !sReadArray(inStream, state->mConnectionBroken))
			return false;

		const uint32 chunk_count = state->mSettings.mPreFracturedMesh.GetChunkCount();
		if (state->mIsFractured)
		{
			if (state->mChunkBodies.size() != chunk_count
				|| state->mChunkReleased.size() != chunk_count
				|| state->mConnectionBroken.size() != state->mSettings.mConnections.size())
				return false;

			if (!RestoreFracturedTopology(inBodyInterface, body_id, existing_chunk_bodies, *state))
				return false;
		}
		else
		{
			if (!state->mChunkBodies.empty() || !state->mChunkReleased.empty() || !state->mConnectionBroken.empty())
				return false;

			if (!RestoreIntactTopology(inBodyInterface, body_id, existing_chunk_bodies, *state))
				return false;
		}
	}

	mEvents.clear();
	return !static_cast<StreamIn &>(inStream).IsFailed();
}

DestructionSystem::BreakableBodyState *DestructionSystem::FindMutable(const BodyID &inBodyID)
{
	auto it = mBreakableBodies.find(inBodyID);
	return it != mBreakableBodies.end() ? &it->second : nullptr;
}

const DestructionSystem::BreakableBodyState *DestructionSystem::Find(const BodyID &inBodyID) const
{
	auto it = mBreakableBodies.find(inBodyID);
	return it != mBreakableBodies.end() ? &it->second : nullptr;
}

bool DestructionSystem::ShouldFracture(const BreakableBodyState &inState, const DestructionDamage &inDamage) const
{
	if (inDamage.mForceFracture)
		return true;

	if (inState.mAccumulatedDamage >= inState.mSettings.mFractureDamageThreshold)
		return true;

	const float impulse_threshold = inState.mSettings.mFractureImpulseThreshold;
	return inDamage.mImpulse.LengthSq() >= impulse_threshold * impulse_threshold;
}

float DestructionSystem::ComputeFractureScore(const BreakableBodyState &inState, const DestructionDamage &inDamage) const
{
	float score = std::max(0.0f, inDamage.mDamage) * inState.mSettings.mPendingDamageWeight;
	score += inDamage.mImpulse.Length() * inState.mSettings.mPendingImpulseWeight;
	score += std::max(0.0f, inDamage.mRadius) * inState.mSettings.mPendingRadiusWeight;
	if (inDamage.mForceFracture)
		score += inState.mSettings.mPendingForceFractureBias;
	return score;
}

bool DestructionSystem::HasAnchoredChunks(const BreakableBodyState &inState) const
{
	for (uint32 chunk_index = 0; chunk_index < inState.mSettings.mPreFracturedMesh.GetChunkCount(); ++chunk_index)
		if (inState.mSettings.mPreFracturedMesh.GetChunk(chunk_index).mAnchored)
			return true;
	return false;
}

bool DestructionSystem::AllocateDeterministicChunkBodyIDs(const BodyInterface &inBodyInterface, const BodyID &inBodyID, uint32 inChunkCount, Array<BodyID> &outChunkBodyIDs) const
{
	outChunkBodyIDs.clear();
	outChunkBodyIDs.reserve(inChunkCount);

	const uint32 source_body_index = inBodyID.GetIndex();
	const uint32 max_bodies = uint32(inBodyInterface.GetMaxBodies());
	for (uint32 candidate_index = 0; candidate_index < max_bodies && outChunkBodyIDs.size() < inChunkCount; ++candidate_index)
	{
		if (candidate_index == source_body_index)
			continue;

		const BodyID candidate_id(candidate_index, 1);
		if (!inBodyInterface.IsBodyIDAvailable(candidate_id))
			continue;

		outChunkBodyIDs.push_back(candidate_id);
	}

	return outChunkBodyIDs.size() == inChunkCount;
}

void DestructionSystem::CaptureIntactBodyTemplate(const BodyInterface &inBodyInterface, const BodyID &inBodyID, BreakableBodyState &ioState) const
{
	if (!inBodyInterface.HasBody(inBodyID))
		return;

	ioState.mIntactBodyAdded = inBodyInterface.IsAdded(inBodyID);
	if (ioState.mHasIntactBodyTemplate && ioState.mIntactBodyTemplate.GetShape() != nullptr)
		return;

	ioState.mIntactBodyTemplate = inBodyInterface.GetBodyCreationSettings(inBodyID);
	ioState.mHasIntactBodyTemplate = ioState.mIntactBodyTemplate.GetShape() != nullptr;
}

bool DestructionSystem::FractureBody(BodyInterface &inBodyInterface, const BodyID &inBodyID, BreakableBodyState &ioState, EActivation inActivationMode)
{
	const uint32 chunk_count = ioState.mSettings.mPreFracturedMesh.GetChunkCount();
	if (chunk_count == 0)
		return false;

	CaptureIntactBodyTemplate(inBodyInterface, inBodyID, ioState);
	if (!ioState.mHasIntactBodyTemplate)
		return false;

	ioState.mFracturePosition = inBodyInterface.GetPosition(inBodyID);
	ioState.mFractureRotation = inBodyInterface.GetRotation(inBodyID);
	ioState.mInheritedLinearVelocity = ioState.mSettings.mInheritIntactLinearVelocity ? inBodyInterface.GetLinearVelocity(inBodyID) : Vec3::sZero();
	ioState.mInheritedAngularVelocity = ioState.mSettings.mInheritIntactAngularVelocity ? inBodyInterface.GetAngularVelocity(inBodyID) : Vec3::sZero();

	const DestructionDamage fracture_damage = ioState.mPendingDamage;
	const bool spawn_supported_chunks_as_static = ioState.mSettings.mSpawnSupportedChunksAsStatic && HasAnchoredChunks(ioState);

	Array<BodyID> deterministic_chunk_body_ids;
	if (!AllocateDeterministicChunkBodyIDs(inBodyInterface, inBodyID, chunk_count, deterministic_chunk_body_ids))
		return false;

	Array<BodyID> created_chunk_bodies;
	created_chunk_bodies.reserve(chunk_count);
	for (uint32 chunk_index = 0; chunk_index < chunk_count; ++chunk_index)
	{
		const EMotionType initial_motion_type = spawn_supported_chunks_as_static ? EMotionType::Static : EMotionType::Dynamic;
		BodyCreationSettings settings = ioState.mSettings.mPreFracturedMesh.CreateChunkCreationSettings(chunk_index, ioState.mIntactBodyTemplate, ioState.mFracturePosition, ioState.mFractureRotation, initial_motion_type);
		if (spawn_supported_chunks_as_static)
			settings.mAllowDynamicOrKinematic = true;
		settings.mLinearVelocity = ioState.mInheritedLinearVelocity;
		settings.mAngularVelocity = ioState.mInheritedAngularVelocity;

		Body *body = inBodyInterface.CreateBodyWithID(deterministic_chunk_body_ids[chunk_index], settings);
		if (body == nullptr)
		{
			for (const BodyID &created_body : created_chunk_bodies)
				inBodyInterface.DestroyBody(created_body);
			return false;
		}

		created_chunk_bodies.push_back(body->GetID());
	}

	DestroyBodyIfPresent(inBodyInterface, inBodyID);

	for (const BodyID &chunk_body : created_chunk_bodies)
		inBodyInterface.AddBody(chunk_body, spawn_supported_chunks_as_static ? EActivation::DontActivate : inActivationMode);

	ioState.mChunkBodies = created_chunk_bodies;
	ioState.mChunkReleased.resize(chunk_count);
	for (uint32 chunk_index = 0; chunk_index < chunk_count; ++chunk_index)
		ioState.mChunkReleased[chunk_index] = 0;
	ioState.mConnectionBroken.resize(ioState.mSettings.mConnections.size());
	for (uint32 connection_index = 0; connection_index < ioState.mConnectionBroken.size(); ++connection_index)
		ioState.mConnectionBroken[connection_index] = 0;
	ioState.mHasPendingDamage = false;
	ioState.mPendingFracture = false;
	ioState.mPendingFractureScore = 0.0f;
	ioState.mPendingDamage = DestructionDamage();
	ioState.mIsFractured = true;
	ioState.mCollapseEventSent = false;
	ioState.mIntactBodyAdded = false;

	BreakConnectionsByDamage(&inBodyInterface, ioState, fracture_damage, ioState.mConnectionBroken);
	mEvents.push_back({ EDestructionEventType::Fractured, inBodyID, BodyID(), 0xffffffff });
	EvaluateReleasedChunks(inBodyInterface, inBodyID, ioState, fracture_damage, Vec3::sZero(), inActivationMode);
	return true;
}

void DestructionSystem::BreakConnectionsByDamage(BodyInterface *inBodyInterface, const BreakableBodyState &inState, const DestructionDamage &inDamage, Array<uint8> &ioConnectionBroken) const
{
	if (inState.mSettings.mConnections.empty())
		return;

	const bool has_radius = inDamage.mRadius > 0.0f;
	const double radius_sq = double(inDamage.mRadius) * double(inDamage.mRadius);
	for (uint32 connection_index = 0; connection_index < inState.mSettings.mConnections.size(); ++connection_index)
	{
		if (ioConnectionBroken[connection_index] != 0)
			continue;

		const DestructionConnection &connection = inState.mSettings.mConnections[connection_index];
		if (!inDamage.mForceFracture && inDamage.mDamage < connection.mBreakDamageThreshold)
			continue;

		bool should_break = !has_radius;
		if (has_radius)
		{
			RVec3 position_a;
			RVec3 position_b;
			if (inBodyInterface != nullptr && inState.mIsFractured)
			{
				position_a = inBodyInterface->GetPosition(inState.mChunkBodies[connection.mChunkA]);
				position_b = inBodyInterface->GetPosition(inState.mChunkBodies[connection.mChunkB]);
			}
			else
			{
				position_a = inState.mSettings.mPreFracturedMesh.GetChunkWorldPosition(connection.mChunkA, inState.mFracturePosition, inState.mFractureRotation);
				position_b = inState.mSettings.mPreFracturedMesh.GetChunkWorldPosition(connection.mChunkB, inState.mFracturePosition, inState.mFractureRotation);
			}

			should_break = (position_a - inDamage.mPosition).LengthSq() <= radius_sq || (position_b - inDamage.mPosition).LengthSq() <= radius_sq;
		}

		if (should_break)
			ioConnectionBroken[connection_index] = 1;
	}
}

void DestructionSystem::EvaluateReleasedChunks(BodyInterface &inBodyInterface, const BodyID &inBodyID, BreakableBodyState &ioState, const DestructionDamage &inDamage, Vec3Arg inExtraImpulse, EActivation inActivationMode)
{
	const uint32 chunk_count = uint32(ioState.mChunkBodies.size());
	if (chunk_count == 0)
		return;

	bool released_any = false;
	if (!ioState.mSettings.mSpawnSupportedChunksAsStatic || !HasAnchoredChunks(ioState))
	{
		for (uint32 chunk_index = 0; chunk_index < chunk_count; ++chunk_index)
			if (ioState.mChunkReleased[chunk_index] == 0)
			{
				ReleaseChunkInternal(inBodyInterface, inBodyID, ioState, chunk_index, inDamage, inExtraImpulse, inActivationMode);
				released_any = true;
			}
	}
	else
	{
		Array<uint8> supported(chunk_count);
		for (uint32 chunk_index = 0; chunk_index < chunk_count; ++chunk_index)
			supported[chunk_index] = 0;

		Array<uint32> stack;
		for (uint32 chunk_index = 0; chunk_index < chunk_count; ++chunk_index)
			if (ioState.mSettings.mPreFracturedMesh.GetChunk(chunk_index).mAnchored && ioState.mChunkReleased[chunk_index] == 0)
			{
				supported[chunk_index] = 1;
				stack.push_back(chunk_index);
			}

		while (!stack.empty())
		{
			const uint32 current = stack.back();
			stack.pop_back();
			for (uint32 connection_index = 0; connection_index < ioState.mSettings.mConnections.size(); ++connection_index)
			{
				if (ioState.mConnectionBroken[connection_index] != 0)
					continue;

				const DestructionConnection &connection = ioState.mSettings.mConnections[connection_index];
				uint32 other = 0xffffffff;
				if (connection.mChunkA == current)
					other = connection.mChunkB;
				else if (connection.mChunkB == current)
					other = connection.mChunkA;

				if (other == 0xffffffff || supported[other] != 0 || ioState.mChunkReleased[other] != 0)
					continue;

				supported[other] = 1;
				stack.push_back(other);
			}
		}

		for (uint32 chunk_index = 0; chunk_index < chunk_count; ++chunk_index)
			if (supported[chunk_index] == 0 && ioState.mChunkReleased[chunk_index] == 0)
			{
				ReleaseChunkInternal(inBodyInterface, inBodyID, ioState, chunk_index, inDamage, inExtraImpulse, inActivationMode);
				released_any = true;
			}
	}

	if (released_any && !ioState.mCollapseEventSent)
	{
		mEvents.push_back({ EDestructionEventType::BuildingCollapsed, inBodyID, BodyID(), 0xffffffff });
		ioState.mCollapseEventSent = true;
	}
}

void DestructionSystem::ReleaseChunkInternal(BodyInterface &inBodyInterface, const BodyID &inBodyID, BreakableBodyState &ioState, uint32 inChunkIndex, const DestructionDamage &inDamage, Vec3Arg inExtraImpulse, EActivation inActivationMode)
{
	if (ioState.mChunkReleased[inChunkIndex] != 0)
		return;

	const BodyID chunk_body = ioState.mChunkBodies[inChunkIndex];
	if (inBodyInterface.GetMotionType(chunk_body) != EMotionType::Dynamic)
		inBodyInterface.SetMotionType(chunk_body, EMotionType::Dynamic, inActivationMode);

	inBodyInterface.SetLinearAndAngularVelocity(chunk_body, ioState.mInheritedLinearVelocity, ioState.mInheritedAngularVelocity);

	Vec3 impulse = ComputeChunkImpulse(inBodyInterface, ioState, inChunkIndex, inDamage) + inExtraImpulse;
	if (impulse.LengthSq() > cMinVectorLengthSq)
	{
		inBodyInterface.AddImpulse(chunk_body, impulse);
		const Vec3 angular_impulse = ioState.mSettings.mPreFracturedMesh.GetChunk(inChunkIndex).mLocalPosition.Cross(impulse) * ioState.mSettings.mAngularImpulseScale;
		if (angular_impulse.LengthSq() > cMinVectorLengthSq)
			inBodyInterface.AddAngularImpulse(chunk_body, angular_impulse);
	}

	ioState.mChunkReleased[inChunkIndex] = 1;
	mEvents.push_back({ EDestructionEventType::ChunkReleased, inBodyID, chunk_body, inChunkIndex });
}

Vec3 DestructionSystem::ComputeChunkImpulse(BodyInterface &inBodyInterface, const BreakableBodyState &inState, uint32 inChunkIndex, const DestructionDamage &inDamage) const
{
	float magnitude = inDamage.mImpulse.Length();
	if (inDamage.mDamage > 0.0f)
		magnitude += inDamage.mDamage * inState.mSettings.mDamageToImpulseScale;
	if (magnitude <= 0.0f)
		return Vec3::sZero();

	Vec3 direction = inDamage.mImpulse.NormalizedOr(Vec3::sAxisY());
	if (inDamage.mRadius > 0.0f)
	{
		const Vec3 radial = Vec3(inBodyInterface.GetCenterOfMassPosition(inState.mChunkBodies[inChunkIndex]) - inDamage.mPosition);
		direction = radial.NormalizedOr(direction);
	}

	return direction * (magnitude * inState.mSettings.mPreFracturedMesh.GetChunk(inChunkIndex).mImpulseScale);
}

void DestructionSystem::DestroyBodyIfPresent(BodyInterface &inBodyInterface, const BodyID &inBodyID) const
{
	if (!inBodyInterface.HasBody(inBodyID))
		return;

	if (inBodyInterface.IsAdded(inBodyID))
		inBodyInterface.RemoveBody(inBodyID);
	inBodyInterface.DestroyBody(inBodyID);
}

void DestructionSystem::DestroyBodiesIfPresent(BodyInterface &inBodyInterface, const Array<BodyID> &inBodyIDs) const
{
	for (uint32 index = uint32(inBodyIDs.size()); index > 0; --index)
		DestroyBodyIfPresent(inBodyInterface, inBodyIDs[index - 1]);
}

bool DestructionSystem::RestoreIntactTopology(BodyInterface &inBodyInterface, const BodyID &inBodyID, const Array<BodyID> &inExistingChunkBodies, BreakableBodyState &ioState)
{
	if (!ioState.mHasIntactBodyTemplate || ioState.mIntactBodyTemplate.GetShape() == nullptr)
		return false;

	DestroyBodiesIfPresent(inBodyInterface, inExistingChunkBodies);
	DestroyBodyIfPresent(inBodyInterface, inBodyID);

	Body *intact_body = inBodyInterface.CreateBodyWithID(inBodyID, ioState.mIntactBodyTemplate);
	if (intact_body == nullptr)
		return false;

	if (ioState.mIntactBodyAdded)
		inBodyInterface.AddBody(intact_body->GetID(), EActivation::DontActivate);

	ioState.mChunkBodies.clear();
	ioState.mChunkReleased.clear();
	ioState.mConnectionBroken.clear();
	return true;
}

bool DestructionSystem::RestoreFracturedTopology(BodyInterface &inBodyInterface, const BodyID &inBodyID, const Array<BodyID> &inExistingChunkBodies, BreakableBodyState &ioState)
{
	const uint32 chunk_count = ioState.mSettings.mPreFracturedMesh.GetChunkCount();
	if (!ioState.mHasIntactBodyTemplate || ioState.mIntactBodyTemplate.GetShape() == nullptr || ioState.mChunkBodies.size() != chunk_count)
		return false;

	DestroyBodiesIfPresent(inBodyInterface, inExistingChunkBodies);
	DestroyBodyIfPresent(inBodyInterface, inBodyID);

	Array<BodyID> created_chunk_bodies;
	created_chunk_bodies.reserve(chunk_count);
	const bool spawn_supported_chunks_as_static = ioState.mSettings.mSpawnSupportedChunksAsStatic && HasAnchoredChunks(ioState);

	for (uint32 chunk_index = 0; chunk_index < chunk_count; ++chunk_index)
	{
		BodyCreationSettings settings = ioState.mSettings.mPreFracturedMesh.CreateChunkCreationSettings(chunk_index, ioState.mIntactBodyTemplate, ioState.mFracturePosition, ioState.mFractureRotation, GetChunkMotionTypeForState(ioState, chunk_index));
		if (spawn_supported_chunks_as_static)
			settings.mAllowDynamicOrKinematic = true;
		settings.mLinearVelocity = ioState.mInheritedLinearVelocity;
		settings.mAngularVelocity = ioState.mInheritedAngularVelocity;

		Body *chunk_body = inBodyInterface.CreateBodyWithID(ioState.mChunkBodies[chunk_index], settings);
		if (chunk_body == nullptr)
		{
			for (const BodyID &created_body : created_chunk_bodies)
				inBodyInterface.DestroyBody(created_body);
			return false;
		}

		created_chunk_bodies.push_back(chunk_body->GetID());
	}

	for (const BodyID &chunk_body : created_chunk_bodies)
		inBodyInterface.AddBody(chunk_body, EActivation::DontActivate);

	return true;
}

EMotionType DestructionSystem::GetChunkMotionTypeForState(const BreakableBodyState &inState, uint32 inChunkIndex) const
{
	const bool spawn_supported_chunks_as_static = inState.mSettings.mSpawnSupportedChunksAsStatic && HasAnchoredChunks(inState);
	if (!spawn_supported_chunks_as_static)
		return EMotionType::Dynamic;

	return inState.mChunkReleased[inChunkIndex] != 0 ? EMotionType::Dynamic : EMotionType::Static;
}

ACPH_NAMESPACE_END
