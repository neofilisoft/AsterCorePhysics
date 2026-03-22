// AsterCore Physics Library (https://github.com/jrouwe/AsterCorePhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

ACPH_NAMESPACE_BEGIN

/// Enum used by AddBody to determine if the body needs to be initially active
enum class EActivation
{
	Activate,				///< Activate the body, making it part of the simulation
	DontActivate			///< Leave activation state as it is (will not deactivate an active body)
};

ACPH_NAMESPACE_END
