#pragma once

#include <AsterCore/AsterCore.h>
#include <AsterCore/Physics/GPU/PhysicsComputeBackend.h>

ACPH_NAMESPACE_BEGIN

enum class EPhysicsComputeDeviceAPI : uint32
{
	CPU,
	D3D12,
	Vulkan,
	Metal,
	CUDA
};

struct PhysicsComputeKernelDescriptor
{
	EPhysicsComputeStage mStage = EPhysicsComputeStage::BroadPhase;
	EPhysicsComputeDeviceAPI mDeviceAPI = EPhysicsComputeDeviceAPI::CPU;
	const char *mEntryPoint = "main";
	uint32 mThreadGroupSizeX = 64;
	uint32 mThreadGroupSizeY = 1;
	uint32 mThreadGroupSizeZ = 1;
};

struct PhysicsComputePipelineDescriptor
{
	const char *mName = "AsterComputePipeline";
	PhysicsComputeKernelDescriptor mKernel;
	bool mAllowCPUFallback = true;
	bool mAllowAsyncReadback = true;
};

ACPH_NAMESPACE_END
