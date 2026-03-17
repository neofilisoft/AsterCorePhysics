// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

namespace ACPH {
	class RagdollSettings;
	enum class EMotionType : uint8;
}

#ifdef ACPH_OBJECT_STREAM

enum class EConstraintOverride
{
	TypeFixed,
	TypePoint,
	TypeHinge,
	TypeSlider,
	TypeCone,
	TypeRagdoll,
};

#endif // ACPH_OBJECT_STREAM

class RagdollLoader
{
public:
#ifdef ACPH_OBJECT_STREAM
	/// Load a ragdoll from an ObjectStream file
	static RagdollSettings *		sLoad(const char *inFileName, EMotionType inMotionType, EConstraintOverride inConstraintOverride = EConstraintOverride::TypeRagdoll);
#endif // ACPH_OBJECT_STREAM

	/// Create a ragdoll from code
	static RagdollSettings *		sCreate();
};
