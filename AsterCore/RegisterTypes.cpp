// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/RegisterTypes.h>
#include <AsterCore/Core/Factory.h>
#include <AsterCore/Core/RTTI.h>
#include <AsterCore/Core/TickCounter.h>
#include <AsterCore/Physics/Collision/CollisionDispatch.h>
#include <AsterCore/Physics/Collision/Shape/TriangleShape.h>
#include <AsterCore/Physics/Collision/Shape/PlaneShape.h>
#include <AsterCore/Physics/Collision/Shape/SphereShape.h>
#include <AsterCore/Physics/Collision/Shape/BoxShape.h>
#include <AsterCore/Physics/Collision/Shape/CapsuleShape.h>
#include <AsterCore/Physics/Collision/Shape/TaperedCapsuleShape.h>
#include <AsterCore/Physics/Collision/Shape/CylinderShape.h>
#include <AsterCore/Physics/Collision/Shape/TaperedCylinderShape.h>
#include <AsterCore/Physics/Collision/Shape/ScaledShape.h>
#include <AsterCore/Physics/Collision/Shape/MeshShape.h>
#include <AsterCore/Physics/Collision/Shape/ConvexHullShape.h>
#include <AsterCore/Physics/Collision/Shape/HeightFieldShape.h>
#include <AsterCore/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <AsterCore/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <AsterCore/Physics/Collision/Shape/MutableCompoundShape.h>
#include <AsterCore/Physics/Collision/Shape/StaticCompoundShape.h>
#include <AsterCore/Physics/Collision/Shape/EmptyShape.h>
#include <AsterCore/Physics/Collision/PhysicsMaterialSimple.h>
#include <AsterCore/Physics/SoftBody/SoftBodyShape.h>

ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, Skeleton)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, SkeletalAnimation)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, RagdollSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, PointConstraintSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, SixDOFConstraintSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, SliderConstraintSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, SwingTwistConstraintSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, DistanceConstraintSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, HingeConstraintSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, FixedConstraintSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, ConeConstraintSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, PathConstraintSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, PathConstraintPath)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, PathConstraintPathHermite)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, VehicleConstraintSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, WheeledVehicleControllerSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, WheelSettingsWV)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, TrackedVehicleControllerSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, WheelSettingsTV)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, MotorcycleControllerSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, RackAndPinionConstraintSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, GearConstraintSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, PulleyConstraintSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, MotorSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, PhysicsScene)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, PhysicsMaterial)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, GroupFilter)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, GroupFilterTable)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, BodyCreationSettings)
ACPH_DECLARE_RTTI_WITH_NAMESPACE_FOR_FACTORY(ACPH_EXPORT, ACPH, SoftBodyCreationSettings)

ACPH_NAMESPACE_BEGIN

bool VerifyAsterCoreVersionIDInternal(uint64 inVersionID)
{
	return inVersionID == ACPH_VERSION_ID;
}

void RegisterTypesInternal(uint64 inVersionID)
{
	// Version check
	if (!VerifyAsterCoreVersionIDInternal(inVersionID))
	{
		Trace("Version mismatch, make sure you compile the client code with the same AsterCore version and compiler definitions!");
		uint64 mismatch = ACPH_VERSION_ID ^ inVersionID;
		auto check_bit = [mismatch](int inBit, const char *inLabel) { if (mismatch & (uint64(1) << (inBit + 23))) Trace("Mismatching define %s.", inLabel); };
		check_bit(1, "ACPH_DOUBLE_PRECISION");
		check_bit(2, "ACPH_CROSS_PLATFORM_DETERMINISTIC");
		check_bit(3, "ACPH_FLOATING_POINT_EXCEPTIONS_ENABLED");
		check_bit(4, "ACPH_PROFILE_ENABLED");
		check_bit(5, "ACPH_EXTERNAL_PROFILE");
		check_bit(6, "ACPH_DEBUG_RENDERER");
		check_bit(7, "ACPH_DISABLE_TEMP_ALLOCATOR");
		check_bit(8, "ACPH_DISABLE_CUSTOM_ALLOCATOR");
		check_bit(9, "ACPH_OBJECT_LAYER_BITS");
		check_bit(10, "ACPH_ENABLE_ASSERTS");
		check_bit(11, "ACPH_OBJECT_STREAM");
		std::abort();
	}

#ifndef ACPH_DISABLE_CUSTOM_ALLOCATOR
	ACPH_ASSERT(Allocate != nullptr && Reallocate != nullptr && Free != nullptr && AlignedAllocate != nullptr && AlignedFree != nullptr, "Need to supply an allocator first or call RegisterDefaultAllocator()");
#endif // !ACPH_DISABLE_CUSTOM_ALLOCATOR

	ACPH_ASSERT(Factory::sInstance != nullptr, "Need to create a factory first!");

	// Initialize dispatcher
	CollisionDispatch::sInit();

	// Register base classes first so that we can specialize them later
	CompoundShape::sRegister();
	ConvexShape::sRegister();

	// Register compounds before others so that we can specialize them later (register them in reverse order of collision complexity)
	MutableCompoundShape::sRegister();
	StaticCompoundShape::sRegister();

	// Leaf classes
	TriangleShape::sRegister();
	PlaneShape::sRegister();
	SphereShape::sRegister();
	BoxShape::sRegister();
	CapsuleShape::sRegister();
	TaperedCapsuleShape::sRegister();
	CylinderShape::sRegister();
	TaperedCylinderShape::sRegister();
	MeshShape::sRegister();
	ConvexHullShape::sRegister();
	HeightFieldShape::sRegister();
	SoftBodyShape::sRegister();

	// Register these last because their collision functions are simple so we want to execute them first (register them in reverse order of collision complexity)
	RotatedTranslatedShape::sRegister();
	OffsetCenterOfMassShape::sRegister();
	ScaledShape::sRegister();
	EmptyShape::sRegister();

	// Create list of all types
	const RTTI *types[] = {
		ACPH_RTTI(SkeletalAnimation),
		ACPH_RTTI(Skeleton),
		ACPH_RTTI(CompoundShapeSettings),
		ACPH_RTTI(StaticCompoundShapeSettings),
		ACPH_RTTI(MutableCompoundShapeSettings),
		ACPH_RTTI(TriangleShapeSettings),
		ACPH_RTTI(PlaneShapeSettings),
		ACPH_RTTI(SphereShapeSettings),
		ACPH_RTTI(BoxShapeSettings),
		ACPH_RTTI(CapsuleShapeSettings),
		ACPH_RTTI(TaperedCapsuleShapeSettings),
		ACPH_RTTI(CylinderShapeSettings),
		ACPH_RTTI(TaperedCylinderShapeSettings),
		ACPH_RTTI(ScaledShapeSettings),
		ACPH_RTTI(MeshShapeSettings),
		ACPH_RTTI(ConvexHullShapeSettings),
		ACPH_RTTI(HeightFieldShapeSettings),
		ACPH_RTTI(RotatedTranslatedShapeSettings),
		ACPH_RTTI(OffsetCenterOfMassShapeSettings),
		ACPH_RTTI(EmptyShapeSettings),
		ACPH_RTTI(RagdollSettings),
		ACPH_RTTI(PointConstraintSettings),
		ACPH_RTTI(SixDOFConstraintSettings),
		ACPH_RTTI(SliderConstraintSettings),
		ACPH_RTTI(SwingTwistConstraintSettings),
		ACPH_RTTI(DistanceConstraintSettings),
		ACPH_RTTI(HingeConstraintSettings),
		ACPH_RTTI(FixedConstraintSettings),
		ACPH_RTTI(ConeConstraintSettings),
		ACPH_RTTI(PathConstraintSettings),
		ACPH_RTTI(VehicleConstraintSettings),
		ACPH_RTTI(WheeledVehicleControllerSettings),
		ACPH_RTTI(WheelSettingsWV),
		ACPH_RTTI(TrackedVehicleControllerSettings),
		ACPH_RTTI(WheelSettingsTV),
		ACPH_RTTI(MotorcycleControllerSettings),
		ACPH_RTTI(PathConstraintPath),
		ACPH_RTTI(PathConstraintPathHermite),
		ACPH_RTTI(RackAndPinionConstraintSettings),
		ACPH_RTTI(GearConstraintSettings),
		ACPH_RTTI(PulleyConstraintSettings),
		ACPH_RTTI(MotorSettings),
		ACPH_RTTI(PhysicsScene),
		ACPH_RTTI(PhysicsMaterial),
		ACPH_RTTI(PhysicsMaterialSimple),
		ACPH_RTTI(GroupFilter),
		ACPH_RTTI(GroupFilterTable),
		ACPH_RTTI(BodyCreationSettings),
		ACPH_RTTI(SoftBodyCreationSettings)
	};

	// Register them all
	Factory::sInstance->Register(types, (uint)std::size(types));

	// Initialize default physics material
	if (PhysicsMaterial::sDefault == nullptr)
		PhysicsMaterial::sDefault = new PhysicsMaterialSimple("Default", Color::sGrey);
}

void UnregisterTypes()
{
	// Unregister all types
	if (Factory::sInstance != nullptr)
		Factory::sInstance->Clear();

	// Delete default physics material
	PhysicsMaterial::sDefault = nullptr;
}

ACPH_NAMESPACE_END
