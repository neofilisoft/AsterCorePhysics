// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/TriangleSplitter/TriangleSplitter.h>

ACPH_NAMESPACE_BEGIN

/// Splitter using mean of axis with biggest centroid deviation
class ACPH_EXPORT TriangleSplitterMean : public TriangleSplitter
{
public:
	/// Constructor
							TriangleSplitterMean(const VertexList &inVertices, const IndexedTriangleList &inTriangles);

	// See TriangleSplitter::GetStats
	virtual void			GetStats(Stats &outStats) const override
	{
		outStats.mSplitterName = "TriangleSplitterMean";
	}

	// See TriangleSplitter::Split
	virtual bool			Split(const Range &inTriangles, Range &outLeft, Range &outRight) override;
};

ACPH_NAMESPACE_END
