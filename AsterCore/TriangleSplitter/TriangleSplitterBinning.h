// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/TriangleSplitter/TriangleSplitter.h>
#include <AsterCore/Geometry/AABox.h>

ACPH_NAMESPACE_BEGIN

/// Binning splitter approach taken from: Realtime Ray Tracing on GPU with BVH-based Packet Traversal by Johannes Gunther et al.
class ACPH_EXPORT TriangleSplitterBinning : public TriangleSplitter
{
public:
	/// Constructor
							TriangleSplitterBinning(const VertexList &inVertices, const IndexedTriangleList &inTriangles, uint inMinNumBins = 8, uint inMaxNumBins = 128, uint inNumTrianglesPerBin = 6);

	// See TriangleSplitter::GetStats
	virtual void			GetStats(Stats &outStats) const override
	{
		outStats.mSplitterName = "TriangleSplitterBinning";
	}

	// See TriangleSplitter::Split
	virtual bool			Split(const Range &inTriangles, Range &outLeft, Range &outRight) override;

private:
	// Configuration
	const uint				mMinNumBins;
	const uint				mMaxNumBins;
	const uint				mNumTrianglesPerBin;

	struct Bin
	{
		// Properties of this bin
		AABox				mBounds;
		float				mMinCentroid;
		uint				mNumTriangles;

		// Accumulated data from left most / right most bin to current (including this bin)
		AABox				mBoundsAccumulatedLeft;
		AABox				mBoundsAccumulatedRight;
		uint				mNumTrianglesAccumulatedLeft;
		uint				mNumTrianglesAccumulatedRight;
	};

	// Scratch area to store the bins
	Array<Bin>				mBins;
};

ACPH_NAMESPACE_END
