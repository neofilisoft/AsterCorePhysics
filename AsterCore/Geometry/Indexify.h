// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/Geometry/Triangle.h>
#include <AsterCore/Geometry/IndexedTriangle.h>

ACPH_NAMESPACE_BEGIN

/// Take a list of triangles and get the unique set of vertices and use them to create indexed triangles.
/// Vertices that are less than inVertexWeldDistance apart will be combined to a single vertex.
ACPH_EXPORT void Indexify(const TriangleList &inTriangles, VertexList &outVertices, IndexedTriangleList &outTriangles, float inVertexWeldDistance = 1.0e-4f);

/// Take a list of indexed triangles and unpack them
ACPH_EXPORT void Deindexify(const VertexList &inVertices, const IndexedTriangleList &inTriangles, TriangleList &outTriangles);

ACPH_NAMESPACE_END
