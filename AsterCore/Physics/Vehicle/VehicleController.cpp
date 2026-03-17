// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Vehicle/VehicleController.h>
#include <AsterCore/ObjectStream/TypeDeclarations.h>

ACPH_NAMESPACE_BEGIN

ACPH_IMPLEMENT_SERIALIZABLE_ABSTRACT(VehicleControllerSettings)
{
	ACPH_ADD_BASE_CLASS(VehicleControllerSettings, SerializableObject)
}

ACPH_NAMESPACE_END
