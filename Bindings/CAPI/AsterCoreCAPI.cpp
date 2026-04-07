#include <Bindings/CAPI/AsterCoreCAPI.h>

#include <AsterCore/AsterCore.h>
#include <AsterCore/RegisterTypes.h>
#include <AsterCore/Core/Factory.h>
#include <AsterCore/Core/JobSystemSingleThreaded.h>
#include <AsterCore/Core/JobSystemThreadPool.h>
#include <AsterCore/Core/TempAllocator.h>
#include <AsterCore/Math/Quat.h>
#include <AsterCore/Physics/Body/BodyCreationSettings.h>
#include <AsterCore/Physics/Body/BodyID.h>
#include <AsterCore/Physics/Body/BodyInterface.h>
#include <AsterCore/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <AsterCore/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <AsterCore/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <AsterCore/Physics/Collision/CastResult.h>
#include <AsterCore/Physics/Collision/NarrowPhaseQuery.h>
#include <AsterCore/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <AsterCore/Physics/Collision/RayCast.h>
#include <AsterCore/Physics/Collision/Shape/BoxShape.h>
#include <AsterCore/Physics/Collision/Shape/SphereShape.h>
#include <AsterCore/Physics/PhysicsSystem.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>

using namespace ACPH;
using namespace ACPH::literals;

namespace
{
    namespace CapiLayers
    {
        static constexpr ObjectLayer kStatic = 4;
        static constexpr ObjectLayer kDynamic = 5;
        static constexpr ObjectLayer kCount = 8;
    }

    namespace CapiBroadPhaseLayers
    {
        static constexpr BroadPhaseLayer kStatic(0);
        static constexpr BroadPhaseLayer kDynamic(1);
        static constexpr BroadPhaseLayer kUnused(2);
        static constexpr uint kCount = 3;
    }

    class CapiObjectLayerPairFilter final : public ObjectLayerPairFilter
    {
    public:
        virtual bool ShouldCollide(ObjectLayer inObject1, ObjectLayer inObject2) const override
        {
            if (inObject1 == CapiLayers::kStatic)
                return inObject2 == CapiLayers::kDynamic;
            if (inObject1 == CapiLayers::kDynamic)
                return inObject2 == CapiLayers::kStatic || inObject2 == CapiLayers::kDynamic;
            return false;
        }
    };

    class CapiBroadPhaseLayerInterface final : public BroadPhaseLayerInterface
    {
    public:
        CapiBroadPhaseLayerInterface()
        {
            for (ObjectLayer layer = 0; layer < CapiLayers::kCount; ++layer)
                mObjectToBroadPhase[layer] = CapiBroadPhaseLayers::kUnused;

            mObjectToBroadPhase[CapiLayers::kStatic] = CapiBroadPhaseLayers::kStatic;
            mObjectToBroadPhase[CapiLayers::kDynamic] = CapiBroadPhaseLayers::kDynamic;
        }

        virtual uint GetNumBroadPhaseLayers() const override
        {
            return CapiBroadPhaseLayers::kCount;
        }

        virtual BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer inLayer) const override
        {
            ACPH_ASSERT(inLayer < CapiLayers::kCount);
            return mObjectToBroadPhase[inLayer];
        }

    private:
        BroadPhaseLayer mObjectToBroadPhase[CapiLayers::kCount];
    };

    class CapiObjectVsBroadPhaseLayerFilter final : public ObjectVsBroadPhaseLayerFilter
    {
    public:
        virtual bool ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override
        {
            if (inLayer1 == CapiLayers::kStatic)
                return inLayer2 == CapiBroadPhaseLayers::kDynamic;
            if (inLayer1 == CapiLayers::kDynamic)
                return inLayer2 == CapiBroadPhaseLayers::kStatic || inLayer2 == CapiBroadPhaseLayers::kDynamic;
            return false;
        }
    };

    std::mutex gRuntimeMutex;
    uint32_t gRuntimeRefCount = 0;

    inline ACPH_CAPIVec3 MakeVec3(float x = 0.0f, float y = 0.0f, float z = 0.0f)
    {
        ACPH_CAPIVec3 result { x, y, z };
        return result;
    }

    inline ACPH_CAPIQuat MakeQuat(float x = 0.0f, float y = 0.0f, float z = 0.0f, float w = 1.0f)
    {
        ACPH_CAPIQuat result { x, y, z, w };
        return result;
    }

    inline Vec3 ToVec3(const ACPH_CAPIVec3 &value)
    {
        return Vec3(value.x, value.y, value.z);
    }

    inline RVec3 ToRVec3(const ACPH_CAPIVec3 &value)
    {
        return RVec3(Real(value.x), Real(value.y), Real(value.z));
    }

    inline Quat ToQuat(const ACPH_CAPIQuat &value)
    {
        return Quat(value.x, value.y, value.z, value.w);
    }

    inline ACPH_CAPIVec3 FromVec3(Vec3Arg value)
    {
        return MakeVec3(value.GetX(), value.GetY(), value.GetZ());
    }

    inline ACPH_CAPIVec3 FromRVec3(RVec3Arg value)
    {
        return MakeVec3(float(value.GetX()), float(value.GetY()), float(value.GetZ()));
    }

    inline ACPH_CAPIQuat FromQuat(QuatArg value)
    {
        return MakeQuat(value.GetX(), value.GetY(), value.GetZ(), value.GetW());
    }

    inline BodyID ToBodyID(uint32_t raw_id)
    {
        return BodyID(raw_id);
    }

    inline bool IsBodyKnown(const std::unordered_set<uint32_t> &bodies, uint32_t raw_id)
    {
        return bodies.find(raw_id) != bodies.end();
    }
}

struct ACPH_CAPIWorldHandle_t
{
    CapiBroadPhaseLayerInterface mBroadPhaseLayerInterface;
    CapiObjectLayerPairFilter mObjectLayerPairFilter;
    CapiObjectVsBroadPhaseLayerFilter mObjectVsBroadPhaseLayerFilter;
    TempAllocatorImplWithMallocFallback mTempAllocator;
    JobSystemSingleThreaded mJobSystem;
    PhysicsSystem mPhysicsSystem;
    std::unordered_set<uint32_t> mBodies;
    std::string mLastError;

    explicit ACPH_CAPIWorldHandle_t(const ACPH_CAPIWorldSettings &settings) :
        mTempAllocator(std::max(1u, settings.temp_allocator_size_mb) * 1024u * 1024u),
        mJobSystem(cMaxPhysicsJobs)
    {
        const uint max_bodies = std::max(1024u, settings.max_bodies);
        const uint max_body_pairs = std::max(1024u, settings.max_body_pairs);
        const uint max_contact_constraints = std::max(1024u, settings.max_contact_constraints);
        mPhysicsSystem.Init(max_bodies, 0, max_body_pairs, max_contact_constraints, mBroadPhaseLayerInterface, mObjectVsBroadPhaseLayerFilter, mObjectLayerPairFilter);
        mPhysicsSystem.SetGravity(ToVec3(settings.gravity));
    }

    ~ACPH_CAPIWorldHandle_t()
    {
        Reset();
    }

    void SetError(const char *message)
    {
        mLastError = message != nullptr? message : "Unknown AsterCore C API error.";
    }

    void ClearError()
    {
        mLastError.clear();
    }

    void Reset()
    {
        BodyInterface &body_interface = mPhysicsSystem.GetBodyInterface();
        for (uint32_t raw_id : mBodies)
        {
            BodyID body_id(raw_id);
            if (body_interface.IsAdded(body_id))
                body_interface.RemoveBody(body_id);
            body_interface.DestroyBody(body_id);
        }
        mBodies.clear();
        ClearError();
    }
};

static void ACPH_CAPIDefaultWorldSettings_Internal(ACPH_CAPIWorldSettings &settings)
{
    settings.max_bodies = 8192;
    settings.max_body_pairs = 8192;
    settings.max_contact_constraints = 16384;
    settings.temp_allocator_size_mb = 10;
    settings.worker_threads = 0;
    settings.gravity = MakeVec3(0.0f, -9.81f, 0.0f);
}

int ACPH_CAPIInitialize(void)
{
    std::lock_guard<std::mutex> lock(gRuntimeMutex);
    if (gRuntimeRefCount++ == 0)
    {
        RegisterDefaultAllocator();
        Factory::sInstance = new Factory();
        RegisterTypes();
    }
    return 1;
}

void ACPH_CAPIShutdown(void)
{
    std::lock_guard<std::mutex> lock(gRuntimeMutex);
    if (gRuntimeRefCount == 0)
        return;

    if (--gRuntimeRefCount == 0)
    {
        UnregisterTypes();
        delete Factory::sInstance;
        Factory::sInstance = nullptr;
    }
}

const char *ACPH_CAPIGetVersionString(void)
{
    return "AsterCore C API 3.1.0";
}

void ACPH_CAPIDefaultWorldSettings(ACPH_CAPIWorldSettings *out_settings)
{
    if (out_settings != nullptr)
        ACPH_CAPIDefaultWorldSettings_Internal(*out_settings);
}

ACPH_CAPIWorldHandle ACPH_CAPICreateWorld(const ACPH_CAPIWorldSettings *settings)
{
    ACPH_CAPIWorldSettings local_settings;
    ACPH_CAPIDefaultWorldSettings_Internal(local_settings);
    if (settings != nullptr)
        local_settings = *settings;

    if (!ACPH_CAPIInitialize())
        return nullptr;

    try
    {
        return new ACPH_CAPIWorldHandle_t(local_settings);
    }
    catch (...)
    {
        ACPH_CAPIShutdown();
        return nullptr;
    }
}

void ACPH_CAPIDestroyWorld(ACPH_CAPIWorldHandle world)
{
    delete world;
    ACPH_CAPIShutdown();
}

int ACPH_CAPIIsWorldValid(ACPH_CAPIWorldHandle world)
{
    return world != nullptr? 1 : 0;
}

const char *ACPH_CAPIGetLastError(ACPH_CAPIWorldHandle world)
{
    return world != nullptr? world->mLastError.c_str() : "AsterCore world handle is null.";
}

void ACPH_CAPIResetWorld(ACPH_CAPIWorldHandle world)
{
    if (world != nullptr)
        world->Reset();
}

void ACPH_CAPISetGravity(ACPH_CAPIWorldHandle world, ACPH_CAPIVec3 gravity)
{
    if (world != nullptr)
        world->mPhysicsSystem.SetGravity(ToVec3(gravity));
}

ACPH_CAPIVec3 ACPH_CAPIGetGravity(ACPH_CAPIWorldHandle world)
{
    return world != nullptr? FromVec3(world->mPhysicsSystem.GetGravity()) : MakeVec3();
}

void ACPH_CAPIOptimizeBroadPhase(ACPH_CAPIWorldHandle world)
{
    if (world != nullptr)
        world->mPhysicsSystem.OptimizeBroadPhase();
}

int ACPH_CAPIWorldStep(ACPH_CAPIWorldHandle world, float delta_time, uint32_t collision_steps)
{
    if (world == nullptr)
        return 0;

    world->ClearError();
    const uint32_t sub_steps = std::max(1u, collision_steps);
    const EPhysicsUpdateError error = world->mPhysicsSystem.Update(delta_time, int(sub_steps), &world->mTempAllocator, &world->mJobSystem);
    if (error != EPhysicsUpdateError::None)
    {
        world->SetError("AsterCore physics update reported an error state.");
        return 0;
    }
    return 1;
}

static uint32_t CreateBodyCommon(ACPH_CAPIWorldHandle world, const BodyCreationSettings &settings, EActivation activation)
{
    if (world == nullptr)
        return 0;

    world->ClearError();
    BodyInterface &body_interface = world->mPhysicsSystem.GetBodyInterface();
    Body *body = body_interface.CreateBody(settings);
    if (body == nullptr)
    {
        world->SetError("Failed to create physics body.");
        return 0;
    }

    const BodyID id = body->GetID();
    body_interface.AddBody(id, activation);

    const uint32_t raw_id = id.GetIndexAndSequenceNumber();
    world->mBodies.insert(raw_id);
    return raw_id;
}

uint32_t ACPH_CAPICreateBoxBody(ACPH_CAPIWorldHandle world, ACPH_CAPIVec3 half_extents, ACPH_CAPIVec3 position, int is_dynamic, float mass)
{
    if (world == nullptr)
        return 0;

    const bool dynamic = is_dynamic != 0;
    BodyCreationSettings settings(new BoxShapeSettings(ToVec3(half_extents)), ToRVec3(position), Quat::sIdentity(), dynamic? EMotionType::Dynamic : EMotionType::Static, dynamic? CapiLayers::kDynamic : CapiLayers::kStatic);
    settings.mMotionQuality = EMotionQuality::Discrete;
    settings.mLinearDamping = 0.0f;
    settings.mAngularDamping = 0.0f;
    settings.mCollisionGroup.SetGroupID(0);
    return CreateBodyCommon(world, settings, dynamic? EActivation::Activate : EActivation::DontActivate);
}

uint32_t ACPH_CAPICreateSphereBody(ACPH_CAPIWorldHandle world, float radius, ACPH_CAPIVec3 position, int is_dynamic, float mass)
{
    if (world == nullptr)
        return 0;

    const bool dynamic = is_dynamic != 0;
    BodyCreationSettings settings(new SphereShapeSettings(std::max(0.001f, radius)), ToRVec3(position), Quat::sIdentity(), dynamic? EMotionType::Dynamic : EMotionType::Static, dynamic? CapiLayers::kDynamic : CapiLayers::kStatic);
    settings.mMotionQuality = EMotionQuality::Discrete;
    settings.mLinearDamping = 0.0f;
    settings.mAngularDamping = 0.0f;
    settings.mCollisionGroup.SetGroupID(0);
    return CreateBodyCommon(world, settings, dynamic? EActivation::Activate : EActivation::DontActivate);
}

int ACPH_CAPIDestroyBody(ACPH_CAPIWorldHandle world, uint32_t body_id)
{
    if (world == nullptr || !IsBodyKnown(world->mBodies, body_id))
        return 0;

    BodyInterface &body_interface = world->mPhysicsSystem.GetBodyInterface();
    BodyID id = ToBodyID(body_id);
    if (body_interface.IsAdded(id))
        body_interface.RemoveBody(id);
    body_interface.DestroyBody(id);
    world->mBodies.erase(body_id);
    return 1;
}

int ACPH_CAPIHasBody(ACPH_CAPIWorldHandle world, uint32_t body_id)
{
    return world != nullptr && IsBodyKnown(world->mBodies, body_id)? 1 : 0;
}

int ACPH_CAPIIsBodyActive(ACPH_CAPIWorldHandle world, uint32_t body_id)
{
    if (world == nullptr || !IsBodyKnown(world->mBodies, body_id))
        return 0;

    return world->mPhysicsSystem.GetBodyInterface().IsActive(ToBodyID(body_id))? 1 : 0;
}

ACPH_CAPIVec3 ACPH_CAPIGetBodyPosition(ACPH_CAPIWorldHandle world, uint32_t body_id)
{
    if (world == nullptr || !IsBodyKnown(world->mBodies, body_id))
        return MakeVec3();

    return FromRVec3(world->mPhysicsSystem.GetBodyInterface().GetPosition(ToBodyID(body_id)));
}

void ACPH_CAPISetBodyPosition(ACPH_CAPIWorldHandle world, uint32_t body_id, ACPH_CAPIVec3 position)
{
    if (world == nullptr || !IsBodyKnown(world->mBodies, body_id))
        return;

    world->mPhysicsSystem.GetBodyInterface().SetPosition(ToBodyID(body_id), ToRVec3(position), EActivation::Activate);
}

ACPH_CAPIQuat ACPH_CAPIGetBodyRotation(ACPH_CAPIWorldHandle world, uint32_t body_id)
{
    if (world == nullptr || !IsBodyKnown(world->mBodies, body_id))
        return MakeQuat();

    return FromQuat(world->mPhysicsSystem.GetBodyInterface().GetRotation(ToBodyID(body_id)));
}

void ACPH_CAPISetBodyRotation(ACPH_CAPIWorldHandle world, uint32_t body_id, ACPH_CAPIQuat rotation)
{
    if (world == nullptr || !IsBodyKnown(world->mBodies, body_id))
        return;

    world->mPhysicsSystem.GetBodyInterface().SetRotation(ToBodyID(body_id), ToQuat(rotation), EActivation::Activate);
}

void ACPH_CAPISetBodyLinearVelocity(ACPH_CAPIWorldHandle world, uint32_t body_id, ACPH_CAPIVec3 velocity)
{
    if (world == nullptr || !IsBodyKnown(world->mBodies, body_id))
        return;

    world->mPhysicsSystem.GetBodyInterface().SetLinearVelocity(ToBodyID(body_id), ToVec3(velocity));
}

ACPH_CAPIVec3 ACPH_CAPIGetBodyLinearVelocity(ACPH_CAPIWorldHandle world, uint32_t body_id)
{
    if (world == nullptr || !IsBodyKnown(world->mBodies, body_id))
        return MakeVec3();

    return FromVec3(world->mPhysicsSystem.GetBodyInterface().GetLinearVelocity(ToBodyID(body_id)));
}

void ACPH_CAPISetBodyAngularVelocity(ACPH_CAPIWorldHandle world, uint32_t body_id, ACPH_CAPIVec3 velocity)
{
    if (world == nullptr || !IsBodyKnown(world->mBodies, body_id))
        return;

    world->mPhysicsSystem.GetBodyInterface().SetAngularVelocity(ToBodyID(body_id), ToVec3(velocity));
}

ACPH_CAPIVec3 ACPH_CAPIGetBodyAngularVelocity(ACPH_CAPIWorldHandle world, uint32_t body_id)
{
    if (world == nullptr || !IsBodyKnown(world->mBodies, body_id))
        return MakeVec3();

    return FromVec3(world->mPhysicsSystem.GetBodyInterface().GetAngularVelocity(ToBodyID(body_id)));
}

void ACPH_CAPIApplyBodyImpulse(ACPH_CAPIWorldHandle world, uint32_t body_id, ACPH_CAPIVec3 impulse)
{
    if (world == nullptr || !IsBodyKnown(world->mBodies, body_id))
        return;

    world->mPhysicsSystem.GetBodyInterface().AddImpulse(ToBodyID(body_id), ToVec3(impulse));
}

int ACPH_CAPIRaycast(ACPH_CAPIWorldHandle world, ACPH_CAPIVec3 origin, ACPH_CAPIVec3 direction, float max_distance, ACPH_CAPIRaycastHit *out_hit)
{
    if (out_hit != nullptr)
        *out_hit = {};

    if (world == nullptr)
        return 0;

    const float length_sq = direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
    if (length_sq <= 1.0e-8f || max_distance <= 0.0f)
        return 0;

    const float inv_length = 1.0f / std::sqrt(length_sq);
    const ACPH_CAPIVec3 normalized = MakeVec3(direction.x * inv_length, direction.y * inv_length, direction.z * inv_length);

    RayCastResult hit;
    const RRayCast ray(ToRVec3(origin), ToVec3(MakeVec3(normalized.x * max_distance, normalized.y * max_distance, normalized.z * max_distance)));
    if (!world->mPhysicsSystem.GetNarrowPhaseQuery().CastRay(ray, hit))
        return 0;

    if (out_hit != nullptr)
    {
        out_hit->hit = 1;
        out_hit->body_id = hit.mBodyID.GetIndexAndSequenceNumber();
        out_hit->fraction = hit.mFraction;
        out_hit->position = MakeVec3(
            origin.x + normalized.x * max_distance * hit.mFraction,
            origin.y + normalized.y * max_distance * hit.mFraction,
            origin.z + normalized.z * max_distance * hit.mFraction);
        out_hit->normal = MakeVec3();
    }
    return 1;
}
