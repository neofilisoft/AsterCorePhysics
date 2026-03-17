// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Body/BodyCreationSettings.h>
#include <AsterCore/ObjectStream/TypeDeclarations.h>
#include <AsterCore/Core/StreamIn.h>
#include <AsterCore/Core/StreamOut.h>

ACPH_NAMESPACE_BEGIN

ACPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(BodyCreationSettings)
{
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mPosition)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mRotation)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mLinearVelocity)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mAngularVelocity)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mUserData)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mShape)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mCollisionGroup)
	ACPH_ADD_ENUM_ATTRIBUTE(BodyCreationSettings, mObjectLayer)
	ACPH_ADD_ENUM_ATTRIBUTE(BodyCreationSettings, mMotionType)
	ACPH_ADD_ENUM_ATTRIBUTE(BodyCreationSettings, mAllowedDOFs)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mAllowDynamicOrKinematic)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mIsSensor)
	ACPH_ADD_ATTRIBUTE_WITH_ALIAS(BodyCreationSettings, mCollideKinematicVsNonDynamic, "mSensorDetectsStatic") // This is the old name to keep backwards compatibility
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mUseManifoldReduction)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mApplyGyroscopicForce)
	ACPH_ADD_ENUM_ATTRIBUTE(BodyCreationSettings, mMotionQuality)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mEnhancedInternalEdgeRemoval)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mAllowSleeping)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mFriction)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mRestitution)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mLinearDamping)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mAngularDamping)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mMaxLinearVelocity)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mMaxAngularVelocity)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mGravityFactor)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mNumVelocityStepsOverride)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mNumPositionStepsOverride)
	ACPH_ADD_ENUM_ATTRIBUTE(BodyCreationSettings, mOverrideMassProperties)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mInertiaMultiplier)
	ACPH_ADD_ATTRIBUTE(BodyCreationSettings, mMassPropertiesOverride)
}

void BodyCreationSettings::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(mPosition);
	inStream.Write(mRotation);
	inStream.Write(mLinearVelocity);
	inStream.Write(mAngularVelocity);
	mCollisionGroup.SaveBinaryState(inStream);
	inStream.Write(mObjectLayer);
	inStream.Write(mMotionType);
	inStream.Write(mAllowedDOFs);
	inStream.Write(mAllowDynamicOrKinematic);
	inStream.Write(mIsSensor);
	inStream.Write(mCollideKinematicVsNonDynamic);
	inStream.Write(mUseManifoldReduction);
	inStream.Write(mApplyGyroscopicForce);
	inStream.Write(mMotionQuality);
	inStream.Write(mEnhancedInternalEdgeRemoval);
	inStream.Write(mAllowSleeping);
	inStream.Write(mFriction);
	inStream.Write(mRestitution);
	inStream.Write(mLinearDamping);
	inStream.Write(mAngularDamping);
	inStream.Write(mMaxLinearVelocity);
	inStream.Write(mMaxAngularVelocity);
	inStream.Write(mGravityFactor);
	inStream.Write(mNumVelocityStepsOverride);
	inStream.Write(mNumPositionStepsOverride);
	inStream.Write(mOverrideMassProperties);
	inStream.Write(mInertiaMultiplier);
	mMassPropertiesOverride.SaveBinaryState(inStream);
}

void BodyCreationSettings::RestoreBinaryState(StreamIn &inStream)
{
	inStream.Read(mPosition);
	inStream.Read(mRotation);
	inStream.Read(mLinearVelocity);
	inStream.Read(mAngularVelocity);
	mCollisionGroup.RestoreBinaryState(inStream);
	inStream.Read(mObjectLayer);
	inStream.Read(mMotionType);
	inStream.Read(mAllowedDOFs);
	inStream.Read(mAllowDynamicOrKinematic);
	inStream.Read(mIsSensor);
	inStream.Read(mCollideKinematicVsNonDynamic);
	inStream.Read(mUseManifoldReduction);
	inStream.Read(mApplyGyroscopicForce);
	inStream.Read(mMotionQuality);
	inStream.Read(mEnhancedInternalEdgeRemoval);
	inStream.Read(mAllowSleeping);
	inStream.Read(mFriction);
	inStream.Read(mRestitution);
	inStream.Read(mLinearDamping);
	inStream.Read(mAngularDamping);
	inStream.Read(mMaxLinearVelocity);
	inStream.Read(mMaxAngularVelocity);
	inStream.Read(mGravityFactor);
	inStream.Read(mNumVelocityStepsOverride);
	inStream.Read(mNumPositionStepsOverride);
	inStream.Read(mOverrideMassProperties);
	inStream.Read(mInertiaMultiplier);
	mMassPropertiesOverride.RestoreBinaryState(inStream);
}

Shape::ShapeResult BodyCreationSettings::ConvertShapeSettings()
{
	// If we already have a shape, return it
	if (mShapePtr != nullptr)
	{
		mShape = nullptr;

		Shape::ShapeResult result;
		result.Set(const_cast<Shape *>(mShapePtr.GetPtr()));
		return result;
	}

	// Check if we have shape settings
	if (mShape == nullptr)
	{
		Shape::ShapeResult result;
		result.SetError("No shape present!");
		return result;
	}

	// Create the shape
	Shape::ShapeResult result = mShape->Create();
	if (result.IsValid())
		mShapePtr = result.Get();
	mShape = nullptr;
	return result;
}

const Shape *BodyCreationSettings::GetShape() const
{
	// If we already have a shape, return it
	if (mShapePtr != nullptr)
		return mShapePtr;

	// Check if we have shape settings
	if (mShape == nullptr)
		return nullptr;

	// Create the shape
	Shape::ShapeResult result = mShape->Create();
	if (result.IsValid())
		return result.Get();

	Trace("Error: %s", result.GetError().c_str());
	ACPH_ASSERT(false, "An error occurred during shape creation. Use ConvertShapeSettings() to convert the shape and get the error!");
	return nullptr;
}

MassProperties BodyCreationSettings::GetMassProperties() const
{
	// Calculate mass properties
	MassProperties mass_properties;
	switch (mOverrideMassProperties)
	{
	case EOverrideMassProperties::CalculateMassAndInertia:
		mass_properties = GetShape()->GetMassProperties();
		mass_properties.mInertia *= mInertiaMultiplier;
		mass_properties.mInertia(3, 3) = 1.0f;
		break;
	case EOverrideMassProperties::CalculateInertia:
		mass_properties = GetShape()->GetMassProperties();
		mass_properties.ScaleToMass(mMassPropertiesOverride.mMass);
		mass_properties.mInertia *= mInertiaMultiplier;
		mass_properties.mInertia(3, 3) = 1.0f;
		break;
	case EOverrideMassProperties::MassAndInertiaProvided:
		mass_properties = mMassPropertiesOverride;
		break;
	}
	return mass_properties;
}

void BodyCreationSettings::SaveWithChildren(StreamOut &inStream, ShapeToIDMap *ioShapeMap, MaterialToIDMap *ioMaterialMap, GroupFilterToIDMap *ioGroupFilterMap) const
{
	// Save creation settings
	SaveBinaryState(inStream);

	// Save shape
	if (ioShapeMap != nullptr && ioMaterialMap != nullptr)
		GetShape()->SaveWithChildren(inStream, *ioShapeMap, *ioMaterialMap);
	else
		inStream.Write(~uint32(0));

	// Save group filter
	StreamUtils::SaveObjectReference(inStream, mCollisionGroup.GetGroupFilter(), ioGroupFilterMap);
}

BodyCreationSettings::BCSResult BodyCreationSettings::sRestoreWithChildren(StreamIn &inStream, IDToShapeMap &ioShapeMap, IDToMaterialMap &ioMaterialMap, IDToGroupFilterMap &ioGroupFilterMap)
{
	BCSResult result;

	// Read creation settings
	BodyCreationSettings settings;
	settings.RestoreBinaryState(inStream);
	if (inStream.IsEOF() || inStream.IsFailed())
	{
		result.SetError("Error reading body creation settings");
		return result;
	}

	// Read shape
	Shape::ShapeResult shape_result = Shape::sRestoreWithChildren(inStream, ioShapeMap, ioMaterialMap);
	if (shape_result.HasError())
	{
		result.SetError(shape_result.GetError());
		return result;
	}
	settings.SetShape(shape_result.Get());

	// Read group filter
	Result gfresult = StreamUtils::RestoreObjectReference(inStream, ioGroupFilterMap);
	if (gfresult.HasError())
	{
		result.SetError(gfresult.GetError());
		return result;
	}
	settings.mCollisionGroup.SetGroupFilter(gfresult.Get());

	result.Set(settings);
	return result;
}

ACPH_NAMESPACE_END
