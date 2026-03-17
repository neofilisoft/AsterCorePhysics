// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Vehicle/VehicleTransmission.h>
#include <AsterCore/ObjectStream/TypeDeclarations.h>

ACPH_NAMESPACE_BEGIN

ACPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleTransmissionSettings)
{
	ACPH_ADD_ENUM_ATTRIBUTE(VehicleTransmissionSettings, mMode)
	ACPH_ADD_ATTRIBUTE(VehicleTransmissionSettings, mGearRatios)
	ACPH_ADD_ATTRIBUTE(VehicleTransmissionSettings, mReverseGearRatios)
	ACPH_ADD_ATTRIBUTE(VehicleTransmissionSettings, mSwitchTime)
	ACPH_ADD_ATTRIBUTE(VehicleTransmissionSettings, mClutchReleaseTime)
	ACPH_ADD_ATTRIBUTE(VehicleTransmissionSettings, mSwitchLatency)
	ACPH_ADD_ATTRIBUTE(VehicleTransmissionSettings, mShiftUpRPM)
	ACPH_ADD_ATTRIBUTE(VehicleTransmissionSettings, mShiftDownRPM)
	ACPH_ADD_ATTRIBUTE(VehicleTransmissionSettings, mClutchStrength)
}

void VehicleTransmissionSettings::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(mMode);
	inStream.Write(mGearRatios);
	inStream.Write(mReverseGearRatios);
	inStream.Write(mSwitchTime);
	inStream.Write(mClutchReleaseTime);
	inStream.Write(mSwitchLatency);
	inStream.Write(mShiftUpRPM);
	inStream.Write(mShiftDownRPM);
	inStream.Write(mClutchStrength);
}

void VehicleTransmissionSettings::RestoreBinaryState(StreamIn &inStream)
{
	inStream.Read(mMode);
	inStream.Read(mGearRatios);
	inStream.Read(mReverseGearRatios);
	inStream.Read(mSwitchTime);
	inStream.Read(mClutchReleaseTime);
	inStream.Read(mSwitchLatency);
	inStream.Read(mShiftUpRPM);
	inStream.Read(mShiftDownRPM);
	inStream.Read(mClutchStrength);
}

void VehicleTransmission::Update(float inDeltaTime, float inCurrentRPM, float inForwardInput, bool inCanShiftUp)
{
	// Update current gear and calculate clutch friction
	if (mMode == ETransmissionMode::Auto)
	{
		// Switch gears based on rpm
		int old_gear = mCurrentGear;
		if (mCurrentGear == 0 // In neutral
			|| inForwardInput * float(mCurrentGear) < 0.0f) // Changing between forward / reverse
		{
			// Switch to first gear or reverse depending on input
			mCurrentGear = inForwardInput > 0.0f? 1 : (inForwardInput < 0.0f? -1 : 0);
		}
		else if (mGearSwitchLatencyTimeLeft == 0.0f) // If not in the timout after switching gears
		{
			if (inCanShiftUp && inCurrentRPM > mShiftUpRPM)
			{
				if (mCurrentGear < 0)
				{
					// Shift up, reverse
					if (mCurrentGear > -(int)mReverseGearRatios.size())
						mCurrentGear--;
				}
				else
				{
					// Shift up, forward
					if (mCurrentGear < (int)mGearRatios.size())
						mCurrentGear++;
				}
			}
			else if (inCurrentRPM < mShiftDownRPM)
			{
				if (mCurrentGear < 0)
				{
					// Shift down, reverse
					int max_gear = inForwardInput != 0.0f? -1 : 0;
					if (mCurrentGear < max_gear)
						mCurrentGear++;
				}
				else
				{
					// Shift down, forward
					int min_gear = inForwardInput != 0.0f? 1 : 0;
					if (mCurrentGear > min_gear)
						mCurrentGear--;
				}
			}
		}

		if (old_gear != mCurrentGear)
		{
			// We've shifted gear, start switch countdown
			mGearSwitchTimeLeft = old_gear != 0? mSwitchTime : 0.0f;
			mClutchReleaseTimeLeft = mClutchReleaseTime;
			mGearSwitchLatencyTimeLeft = mSwitchLatency;
			mClutchFriction = 0.0f;
		}
		else if (mGearSwitchTimeLeft > 0.0f)
		{
			// If still switching gears, count down
			mGearSwitchTimeLeft = max(0.0f, mGearSwitchTimeLeft - inDeltaTime);
			mClutchFriction = 0.0f;
		}
		else if (mClutchReleaseTimeLeft > 0.0f)
		{
			// After switching the gears we slowly release the clutch
			mClutchReleaseTimeLeft = max(0.0f, mClutchReleaseTimeLeft - inDeltaTime);
			mClutchFriction = 1.0f - mClutchReleaseTimeLeft / mClutchReleaseTime;
		}
		else
		{
			// Clutch has full friction
			mClutchFriction = 1.0f;

			// Count down switch latency
			mGearSwitchLatencyTimeLeft = max(0.0f, mGearSwitchLatencyTimeLeft - inDeltaTime);
		}
	}
}

float VehicleTransmission::GetCurrentRatio() const
{
	if (mCurrentGear < 0)
		return mReverseGearRatios[-mCurrentGear - 1];
	else if (mCurrentGear == 0)
		return 0.0f;
	else
		return mGearRatios[mCurrentGear - 1];
}

void VehicleTransmission::SaveState(StateRecorder &inStream) const
{
	inStream.Write(mCurrentGear);
	inStream.Write(mClutchFriction);
	inStream.Write(mGearSwitchTimeLeft);
	inStream.Write(mClutchReleaseTimeLeft);
	inStream.Write(mGearSwitchLatencyTimeLeft);
}

void VehicleTransmission::RestoreState(StateRecorder &inStream)
{
	inStream.Read(mCurrentGear);
	inStream.Read(mClutchFriction);
	inStream.Read(mGearSwitchTimeLeft);
	inStream.Read(mClutchReleaseTimeLeft);
	inStream.Read(mGearSwitchLatencyTimeLeft);
}

ACPH_NAMESPACE_END
