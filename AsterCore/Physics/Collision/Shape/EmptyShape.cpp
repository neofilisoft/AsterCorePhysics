// AsterCore Physics Library (https://github.com/jrouwe/AsterCorePhysics)
// SPDX-FileCopyrightText: 2024 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Collision/Shape/EmptyShape.h>
#include <AsterCore/Physics/Collision/CollisionDispatch.h>
#include <AsterCore/ObjectStream/TypeDeclarations.h>
#ifdef ACPH_DEBUG_RENDERER
	#include <AsterCore/Renderer/DebugRenderer.h>
#endif // ACPH_DEBUG_RENDERER

ACPH_NAMESPACE_BEGIN

ACPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(EmptyShapeSettings)
{
	ACPH_ADD_BASE_CLASS(EmptyShapeSettings, ShapeSettings)

	ACPH_ADD_ATTRIBUTE(EmptyShapeSettings, mCenterOfMass)
}

ShapeSettings::ShapeResult EmptyShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
		Ref<Shape> shape = new EmptyShape(*this, mCachedResult);
	return mCachedResult;
}

MassProperties EmptyShape::GetMassProperties() const
{
	MassProperties mass_properties;
	mass_properties.mMass = 1.0f;
	mass_properties.mInertia = Mat44::sIdentity();
	return mass_properties;
}

#ifdef ACPH_DEBUG_RENDERER
void EmptyShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, [[maybe_unused]] bool inUseMaterialColors, [[maybe_unused]] bool inDrawWireframe) const
{
	inRenderer->DrawMarker(inCenterOfMassTransform.GetTranslation(), inColor, abs(inScale.GetX()) * 0.1f);
}
#endif // ACPH_DEBUG_RENDERER

void EmptyShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::Empty);
	f.mConstruct = []() -> Shape * { return new EmptyShape; };
	f.mColor = Color::sBlack;

	auto collide_empty = []([[maybe_unused]] const Shape *inShape1, [[maybe_unused]] const Shape *inShape2, [[maybe_unused]] Vec3Arg inScale1, [[maybe_unused]] Vec3Arg inScale2, [[maybe_unused]] Mat44Arg inCenterOfMassTransform1, [[maybe_unused]] Mat44Arg inCenterOfMassTransform2, [[maybe_unused]] const SubShapeIDCreator &inSubShapeIDCreator1, [[maybe_unused]] const SubShapeIDCreator &inSubShapeIDCreator2, [[maybe_unused]] const CollideShapeSettings &inCollideShapeSettings, [[maybe_unused]] CollideShapeCollector &ioCollector, [[maybe_unused]] const ShapeFilter &inShapeFilter) { /* Do Nothing */ };
	auto cast_empty = []([[maybe_unused]] const ShapeCast &inShapeCast, [[maybe_unused]] const ShapeCastSettings &inShapeCastSettings, [[maybe_unused]] const Shape *inShape, [[maybe_unused]] Vec3Arg inScale, [[maybe_unused]] const ShapeFilter &inShapeFilter, [[maybe_unused]] Mat44Arg inCenterOfMassTransform2, [[maybe_unused]] const SubShapeIDCreator &inSubShapeIDCreator1, [[maybe_unused]] const SubShapeIDCreator &inSubShapeIDCreator2, [[maybe_unused]] CastShapeCollector &ioCollector) { /* Do nothing */ };

	for (const EShapeSubType s : sAllSubShapeTypes)
	{
		CollisionDispatch::sRegisterCollideShape(EShapeSubType::Empty, s, collide_empty);
		CollisionDispatch::sRegisterCollideShape(s, EShapeSubType::Empty, collide_empty);

		CollisionDispatch::sRegisterCastShape(EShapeSubType::Empty, s, cast_empty);
		CollisionDispatch::sRegisterCastShape(s, EShapeSubType::Empty, cast_empty);
	}
}

ACPH_NAMESPACE_END
