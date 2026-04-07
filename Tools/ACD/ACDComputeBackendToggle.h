// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: Copyright 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>
#include <AsterCore/Physics/GPU/PhysicsComputePipelineDescriptor.h>
#include <string>

namespace AsterTools
{
	enum class EACDComputeBackend : ACPH::uint32
	{
		Auto,
		CPU,
		Vulkan,
		DirectCompute,
	};

	struct ACDComputeBackendAvailability
	{
		bool mVulkanAvailable = true;
#if defined(ACPH_PLATFORM_WINDOWS)
		bool mDirectComputeAvailable = true;
#else
		bool mDirectComputeAvailable = false;
#endif
	};

	struct ACDComputeBackendSelection
	{
		EACDComputeBackend mRequested = EACDComputeBackend::Auto;
		EACDComputeBackend mResolved = EACDComputeBackend::CPU;
		bool mFallbackToCPU = false;
		bool mRequestedBackendAvailable = false;
	};

	const char *			ToString(EACDComputeBackend inBackend);
	bool				TryParseComputeBackend(const std::string &inValue, EACDComputeBackend &outBackend);
	ACDComputeBackendSelection ResolveComputeBackend(EACDComputeBackend inRequestedBackend, const ACDComputeBackendAvailability &inAvailability);
	ACPH::EPhysicsComputeDeviceAPI ToComputeDeviceAPI(EACDComputeBackend inBackend);
	void				ApplyComputeBackendToggle(ACPH::PhysicsComputePipelineDescriptor &ioDescriptor, const ACDComputeBackendSelection &inSelection);
}
