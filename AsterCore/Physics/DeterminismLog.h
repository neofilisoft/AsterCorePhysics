// AsterCore Physics Library (https://github.com/jrouwe/AsterCorePhysics)
// SPDX-FileCopyrightText: 2022 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

//#define ACPH_ENABLE_DETERMINISM_LOG
#ifdef ACPH_ENABLE_DETERMINISM_LOG

#include <AsterCore/Physics/Body/BodyID.h>
#include <AsterCore/Physics/Collision/Shape/SubShapeID.h>

ACPH_SUPPRESS_WARNINGS_STD_BEGIN
#include <iomanip>
#include <fstream>
ACPH_SUPPRESS_WARNINGS_STD_END

ACPH_NAMESPACE_BEGIN

/// A simple class that logs the state of the simulation. The resulting text file can be used to diff between platforms and find issues in determinism.
class DeterminismLog
{
private:
	ACPH_INLINE uint32		Convert(float inValue) const
	{
		return *(uint32 *)&inValue;
	}

	ACPH_INLINE uint64		Convert(double inValue) const
	{
		return *(uint64 *)&inValue;
	}

public:
							DeterminismLog()
	{
		mLog.open("detlog.txt", std::ios::out | std::ios::trunc | std::ios::binary); // Binary because we don't want a difference between Unix and Windows line endings.
		mLog.fill('0');
	}

	DeterminismLog &		operator << (char inValue)
	{
		mLog << inValue;
		return *this;
	}

	DeterminismLog &		operator << (const char *inValue)
	{
		mLog << std::dec << inValue;
		return *this;
	}

	DeterminismLog &		operator << (const string &inValue)
	{
		mLog << std::dec << inValue;
		return *this;
	}

	DeterminismLog &		operator << (const BodyID &inValue)
	{
		mLog << std::hex << std::setw(8) << inValue.GetIndexAndSequenceNumber();
		return *this;
	}

	DeterminismLog &		operator << (const SubShapeID &inValue)
	{
		mLog << std::hex << std::setw(8) << inValue.GetValue();
		return *this;
	}

	DeterminismLog &		operator << (float inValue)
	{
		mLog << std::hex << std::setw(8) << Convert(inValue);
		return *this;
	}

	DeterminismLog &		operator << (int inValue)
	{
		mLog << inValue;
		return *this;
	}

	DeterminismLog &		operator << (uint32 inValue)
	{
		mLog << std::hex << std::setw(8) << inValue;
		return *this;
	}

	DeterminismLog &		operator << (uint64 inValue)
	{
		mLog << std::hex << std::setw(16) << inValue;
		return *this;
	}

	DeterminismLog &		operator << (Vec3Arg inValue)
	{
		mLog << std::hex << std::setw(8) << Convert(inValue.GetX()) << " " << std::setw(8) << Convert(inValue.GetY()) << " " << std::setw(8) << Convert(inValue.GetZ());
		return *this;
	}

	DeterminismLog &		operator << (DVec3Arg inValue)
	{
		mLog << std::hex << std::setw(16) << Convert(inValue.GetX()) << " " << std::setw(16) << Convert(inValue.GetY()) << " " << std::setw(16) << Convert(inValue.GetZ());
		return *this;
	}

	DeterminismLog &		operator << (Vec4Arg inValue)
	{
		mLog << std::hex << std::setw(8) << Convert(inValue.GetX()) << " " << std::setw(8) << Convert(inValue.GetY()) << " " << std::setw(8) << Convert(inValue.GetZ()) << " " << std::setw(8) << Convert(inValue.GetW());
		return *this;
	}

	DeterminismLog &		operator << (const Float3 &inValue)
	{
		mLog << std::hex << std::setw(8) << Convert(inValue.x) << " " << std::setw(8) << Convert(inValue.y) << " " << std::setw(8) << Convert(inValue.z);
		return *this;
	}

	DeterminismLog &		operator << (Mat44Arg inValue)
	{
		*this << inValue.GetColumn4(0) << " " << inValue.GetColumn4(1) << " " << inValue.GetColumn4(2) << " " << inValue.GetColumn4(3);
		return *this;
	}

	DeterminismLog &		operator << (DMat44Arg inValue)
	{
		*this << inValue.GetColumn4(0) << " " << inValue.GetColumn4(1) << " " << inValue.GetColumn4(2) << " " << inValue.GetTranslation();
		return *this;
	}

	DeterminismLog &		operator << (QuatArg inValue)
	{
		*this << inValue.GetXYZW();
		return *this;
	}

	// Singleton instance
	static DeterminismLog	sLog;

private:
	std::ofstream			mLog;
};

/// Will log something to the determinism log, usage: ACPH_DET_LOG("label " << value);
#define ACPH_DET_LOG(...)	DeterminismLog::sLog << __VA_ARGS__ << '\n'

ACPH_NAMESPACE_END

#else

ACPH_SUPPRESS_WARNING_PUSH
ACPH_SUPPRESS_WARNINGS

/// By default we log nothing
#define ACPH_DET_LOG(...)

ACPH_SUPPRESS_WARNING_POP

#endif // ACPH_ENABLE_DETERMINISM_LOG
