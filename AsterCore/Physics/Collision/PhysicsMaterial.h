// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/Core/Reference.h>
#include <AsterCore/Core/Color.h>
#include <AsterCore/Core/Result.h>
#include <AsterCore/ObjectStream/SerializableObject.h>

ACPH_NAMESPACE_BEGIN

class StreamIn;
class StreamOut;

/// This structure describes the surface of (part of) a shape. You should inherit from it to define additional
/// information that is interesting for the simulation. The 2 materials involved in a contact could be used
/// to decide which sound or particle effects to play.
///
/// If you inherit from this material, don't forget to create a suitable default material in sDefault
class ACPH_EXPORT PhysicsMaterial : public SerializableObject, public RefTarget<PhysicsMaterial>
{
	ACPH_DECLARE_SERIALIZABLE_VIRTUAL(ACPH_EXPORT, PhysicsMaterial)

public:
	/// Constructor
											PhysicsMaterial() = default;
	virtual									~PhysicsMaterial() override = default;

	/// Default material that is used when a shape has no materials defined
	static RefConst<PhysicsMaterial>		sDefault;

	// Properties
	virtual const char *					GetDebugName() const			{ return "Unknown"; }
	virtual Color							GetDebugColor() const			{ return Color::sGrey; }

	/// Saves the contents of the material in binary form to inStream.
	virtual void							SaveBinaryState(StreamOut &inStream) const;

	using PhysicsMaterialResult = Result<Ref<PhysicsMaterial>>;

	/// Creates a PhysicsMaterial of the correct type and restores its contents from the binary stream inStream.
	static PhysicsMaterialResult			sRestoreFromBinaryState(StreamIn &inStream);

protected:
	/// Don't allow copy constructing this base class, but allow derived classes to copy themselves
											PhysicsMaterial(const PhysicsMaterial &) = default;
	PhysicsMaterial &						operator = (const PhysicsMaterial &) = default;

	/// This function should not be called directly, it is used by sRestoreFromBinaryState.
	virtual void							RestoreBinaryState(StreamIn &inStream);
};

using PhysicsMaterialList = Array<RefConst<PhysicsMaterial>>;

ACPH_NAMESPACE_END
