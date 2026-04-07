// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: Copyright 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>
#include <AsterCore/Math/Vec3.h>
#include <string>
#include <vector>

namespace AsterTools
{
	struct ACDCounter
	{
		std::string mName;
		double mValue = 0.0;
		std::string mUnit;
	};

	struct ACDMarker
	{
		std::string mLabel;
		ACPH::Vec3 mPosition = ACPH::Vec3::sZero();
		float mScalar = 0.0f;
	};

	struct ACDFrameRecord
	{
		ACPH::uint32 mFrameIndex = 0;
		double mStepTimeMs = 0.0;
		double mMemoryUsedKB = 0.0;
		ACPH::uint64 mDeterminismChecksum = 0;
		std::vector<ACDCounter> mCounters;
		std::vector<ACDMarker> mMarkers;
		std::vector<std::string> mEvents;
	};

	struct ACDComputeBackendToggle
	{
		std::string mRequestedBackend = "Auto";
		std::string mResolvedBackend = "CPU";
		bool mFallbackToCPU = false;
		bool mVulkanAvailable = true;
		bool mDirectComputeAvailable = false;
	};

	class ACDCaptureSession
	{
	public:
		void SetMetadata(const std::string &inSessionName, const std::string &inSceneName, const std::string &inBuildConfiguration);
		void SetComputeBackendToggle(const ACDComputeBackendToggle &inToggleState);
		ACDFrameRecord &AddFrame(ACPH::uint32 inFrameIndex);
		std::string ExportToJson() const;
		bool WriteToFile(const char *inPath) const;

	private:
		std::string mSessionName = "UnnamedSession";
		std::string mSceneName = "UnknownScene";
		std::string mBuildConfiguration = "UnknownBuild";
		ACDComputeBackendToggle mComputeBackendToggle;
		std::vector<ACDFrameRecord> mFrames;
	};
}
