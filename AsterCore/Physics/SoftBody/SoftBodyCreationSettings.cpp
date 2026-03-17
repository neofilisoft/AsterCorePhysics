#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <AsterCore/ObjectStream/TypeDeclarations.h>
#include <AsterCore/Core/StreamIn.h>
#include <AsterCore/Core/StreamOut.h>

ACPH_NAMESPACE_BEGIN

ACPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodyCreationSettings)
{
	ACPH_ADD_ATTRIBUTE(SoftBodyCreationSettings, mSettings)
	ACPH_ADD_ATTRIBUTE(SoftBodyCreationSettings, mPosition)
	ACPH_ADD_ATTRIBUTE(SoftBodyCreationSettings, mRotation)
	ACPH_ADD_ATTRIBUTE(SoftBodyCreationSettings, mUserData)
	ACPH_ADD_ENUM_ATTRIBUTE(SoftBodyCreationSettings, mObjectLayer)
	ACPH_ADD_ATTRIBUTE(SoftBodyCreationSettings, mCollisionGroup)
	ACPH_ADD_ATTRIBUTE(SoftBodyCreationSettings, mNumIterations)
	ACPH_ADD_ATTRIBUTE(SoftBodyCreationSettings, mLinearDamping)
	ACPH_ADD_ATTRIBUTE(SoftBodyCreationSettings, mMaxLinearVelocity)
	ACPH_ADD_ATTRIBUTE(SoftBodyCreationSettings, mRestitution)
	ACPH_ADD_ATTRIBUTE(SoftBodyCreationSettings, mFriction)
	ACPH_ADD_ATTRIBUTE(SoftBodyCreationSettings, mPressure)
	ACPH_ADD_ATTRIBUTE(SoftBodyCreationSettings, mGravityFactor)
	ACPH_ADD_ATTRIBUTE(SoftBodyCreationSettings, mVertexRadius)
	ACPH_ADD_ATTRIBUTE(SoftBodyCreationSettings, mUpdatePosition)
	ACPH_ADD_ATTRIBUTE(SoftBodyCreationSettings, mMakeRotationIdentity)
	ACPH_ADD_ATTRIBUTE(SoftBodyCreationSettings, mAllowSleeping)
	ACPH_ADD_ATTRIBUTE(SoftBodyCreationSettings, mFacesDoubleSided)
}

void SoftBodyCreationSettings::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(mPosition);
	inStream.Write(mRotation);
	inStream.Write(mUserData);
	inStream.Write(mObjectLayer);
	mCollisionGroup.SaveBinaryState(inStream);
	inStream.Write(mNumIterations);
	inStream.Write(mLinearDamping);
	inStream.Write(mMaxLinearVelocity);
	inStream.Write(mRestitution);
	inStream.Write(mFriction);
	inStream.Write(mPressure);
	inStream.Write(mGravityFactor);
	inStream.Write(mVertexRadius);
	inStream.Write(mUpdatePosition);
	inStream.Write(mMakeRotationIdentity);
	inStream.Write(mAllowSleeping);
	inStream.Write(mFacesDoubleSided);
}

void SoftBodyCreationSettings::RestoreBinaryState(StreamIn &inStream)
{
	inStream.Read(mPosition);
	inStream.Read(mRotation);
	inStream.Read(mUserData);
	inStream.Read(mObjectLayer);
	mCollisionGroup.RestoreBinaryState(inStream);
	inStream.Read(mNumIterations);
	inStream.Read(mLinearDamping);
	inStream.Read(mMaxLinearVelocity);
	inStream.Read(mRestitution);
	inStream.Read(mFriction);
	inStream.Read(mPressure);
	inStream.Read(mGravityFactor);
	inStream.Read(mVertexRadius);
	inStream.Read(mUpdatePosition);
	inStream.Read(mMakeRotationIdentity);
	inStream.Read(mAllowSleeping);
	inStream.Read(mFacesDoubleSided);
}

void SoftBodyCreationSettings::SaveWithChildren(StreamOut &inStream, SharedSettingsToIDMap *ioSharedSettingsMap, MaterialToIDMap *ioMaterialMap, GroupFilterToIDMap *ioGroupFilterMap) const
{
	// Save creation settings
	SaveBinaryState(inStream);

	// Save shared settings
	if (ioSharedSettingsMap != nullptr && ioMaterialMap != nullptr)
		mSettings->SaveWithMaterials(inStream, *ioSharedSettingsMap, *ioMaterialMap);
	else
		inStream.Write(~uint32(0));

	// Save group filter
	StreamUtils::SaveObjectReference(inStream, mCollisionGroup.GetGroupFilter(), ioGroupFilterMap);
}

SoftBodyCreationSettings::SBCSResult SoftBodyCreationSettings::sRestoreWithChildren(StreamIn &inStream, IDToSharedSettingsMap &ioSharedSettingsMap, IDToMaterialMap &ioMaterialMap, IDToGroupFilterMap &ioGroupFilterMap)
{
	SBCSResult result;

	// Read creation settings
	SoftBodyCreationSettings settings;
	settings.RestoreBinaryState(inStream);
	if (inStream.IsEOF() || inStream.IsFailed())
	{
		result.SetError("Error reading body creation settings");
		return result;
	}

	// Read shared settings
	SoftBodySharedSettings::SettingsResult settings_result = SoftBodySharedSettings::sRestoreWithMaterials(inStream, ioSharedSettingsMap, ioMaterialMap);
	if (settings_result.HasError())
	{
		result.SetError(settings_result.GetError());
		return result;
	}
	settings.mSettings = settings_result.Get();

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
