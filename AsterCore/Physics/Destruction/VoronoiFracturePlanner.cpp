// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#include <AsterCore/Physics/Destruction/VoronoiFracturePlanner.h>

#include <AsterCore/Physics/Collision/Shape/BoxShape.h>
#include <AsterCore/Physics/Collision/Shape/ConvexHullShape.h>

ACPH_SUPPRESS_WARNINGS_STD_BEGIN
#include <algorithm>
#include <limits>
#include <random>
ACPH_SUPPRESS_WARNINGS_STD_END

ACPH_NAMESPACE_BEGIN

Array<VoronoiFractureCell> VoronoiFracturePlanner::Generate(const VoronoiFractureSettings &inSettings) const
{
	const uint32 cell_count = std::max<uint32>(inSettings.mCellCount, 1);
	const uint32 samples_x = std::max<uint32>(inSettings.mSamplesX, 1);
	const uint32 samples_y = std::max<uint32>(inSettings.mSamplesY, 1);
	const uint32 samples_z = std::max<uint32>(inSettings.mSamplesZ, 1);
	const float min_edge_length = std::max(inSettings.mMinimumChunkExtent * 2.0f, 0.02f);

	Array<VoronoiFractureCell> cells(cell_count);
	std::mt19937_64 rng(inSettings.mRandomSeed != 0 ? inSettings.mRandomSeed : 0xA57EC0DEULL);
	std::uniform_real_distribution<float> dist_x(inSettings.mLocalBounds.mMin.GetX(), inSettings.mLocalBounds.mMax.GetX());
	std::uniform_real_distribution<float> dist_y(inSettings.mLocalBounds.mMin.GetY(), inSettings.mLocalBounds.mMax.GetY());
	std::uniform_real_distribution<float> dist_z(inSettings.mLocalBounds.mMin.GetZ(), inSettings.mLocalBounds.mMax.GetZ());

	for (VoronoiFractureCell &cell : cells)
	{
		cell.mSite = Vec3(dist_x(rng), dist_y(rng), dist_z(rng));
		cell.mCentroid = Vec3::sZero();
		cell.mBounds.SetEmpty();
		cell.mSamples.clear();
		cell.mSampleCount = 0;
	}

	const Vec3 size = inSettings.mLocalBounds.GetSize();
	for (uint32 z = 0; z < samples_z; ++z)
		for (uint32 y = 0; y < samples_y; ++y)
			for (uint32 x = 0; x < samples_x; ++x)
			{
				const float fx = (float(x) + 0.5f) / float(samples_x);
				const float fy = (float(y) + 0.5f) / float(samples_y);
				const float fz = (float(z) + 0.5f) / float(samples_z);
				const Vec3 sample = inSettings.mLocalBounds.mMin + Vec3(fx * size.GetX(), fy * size.GetY(), fz * size.GetZ());

				uint32 closest_index = 0;
				float closest_distance_sq = std::numeric_limits<float>::max();
				for (uint32 cell_index = 0; cell_index < cell_count; ++cell_index)
				{
					const float distance_sq = (cells[cell_index].mSite - sample).LengthSq();
					if (distance_sq < closest_distance_sq)
					{
						closest_distance_sq = distance_sq;
						closest_index = cell_index;
					}
				}

				VoronoiFractureCell &cell = cells[closest_index];
				cell.mBounds.Encapsulate(sample);
				cell.mCentroid += sample;
				cell.mSamples.push_back(sample);
				++cell.mSampleCount;
			}

	for (VoronoiFractureCell &cell : cells)
	{
		if (cell.mSampleCount > 0)
			cell.mCentroid *= 1.0f / float(cell.mSampleCount);
		else
		{
			cell.mCentroid = cell.mSite;
			cell.mBounds = AABox(cell.mSite, cell.mSite);
			cell.mSamples.push_back(cell.mSite);
		}

		cell.mBounds.EnsureMinimalEdgeLength(min_edge_length);
	}

	return cells;
}

void VoronoiFracturePlanner::AppendCellsAsBoxChunks(const VoronoiFractureSettings &inSettings, const BodyCreationSettings &inChunkTemplate, PreFracturedMesh &ioMesh, bool inAnchorLowestLayer) const
{
	ioMesh.SetChunkTemplate(inChunkTemplate);

	const Array<VoronoiFractureCell> cells = Generate(inSettings);
	const float anchor_limit = inSettings.mLocalBounds.mMin.GetY() + std::max(inSettings.mAnchorBandHeight, 0.0f);
	for (const VoronoiFractureCell &cell : cells)
	{
		PreFracturedChunk chunk;
		chunk.mShape = new BoxShape(cell.mBounds.GetExtent());
		chunk.mLocalPosition = cell.mBounds.GetCenter();
		chunk.mMass = std::max(chunk.mShape->GetVolume() * inSettings.mDensity, 0.1f);
		chunk.mAnchored = inAnchorLowestLayer && cell.mBounds.mMin.GetY() <= anchor_limit;
		ioMesh.AddChunk(chunk);
	}
}

void VoronoiFracturePlanner::AppendCellsAsConvexChunks(const VoronoiFractureSettings &inSettings, const BodyCreationSettings &inChunkTemplate, PreFracturedMesh &ioMesh, bool inAnchorLowestLayer) const
{
	ioMesh.SetChunkTemplate(inChunkTemplate);

	const Array<VoronoiFractureCell> cells = Generate(inSettings);
	const float anchor_limit = inSettings.mLocalBounds.mMin.GetY() + std::max(inSettings.mAnchorBandHeight, 0.0f);
	for (const VoronoiFractureCell &cell : cells)
	{
		PreFracturedChunk chunk;
		Array<Vec3> local_samples;
		local_samples.reserve(cell.mSamples.size());
		for (const Vec3 &sample : cell.mSamples)
			local_samples.push_back(sample - cell.mCentroid);

		if (local_samples.size() >= 4)
		{
			ConvexHullShapeSettings hull_settings(local_samples, std::max(inSettings.mConvexRadius, 0.0f));
			Shape::ShapeResult hull_result = hull_settings.Create();
			if (hull_result.IsValid())
			{
				chunk.mShape = hull_result.Get();
				chunk.mLocalPosition = cell.mCentroid;
			}
		}

		if (chunk.mShape == nullptr)
		{
			chunk.mShape = new BoxShape(cell.mBounds.GetExtent());
			chunk.mLocalPosition = cell.mBounds.GetCenter();
		}

		chunk.mMass = std::max(chunk.mShape->GetVolume() * inSettings.mDensity, 0.1f);
		chunk.mAnchored = inAnchorLowestLayer && cell.mBounds.mMin.GetY() <= anchor_limit;
		ioMesh.AddChunk(chunk);
	}
}

ACPH_NAMESPACE_END
