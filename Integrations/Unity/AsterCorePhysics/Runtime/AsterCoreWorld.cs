using System;
using UnityEngine;

namespace AsterCore.Unity
{
    public sealed class AsterCoreWorld : IDisposable
    {
        internal IntPtr Handle { get; private set; }

        public bool IsValid => Handle != IntPtr.Zero && AsterCoreNative.ACPH_CAPIIsWorldValid(Handle) != 0;
        public string LastError => IsValid ? AsterCoreNative.ReadCString(AsterCoreNative.ACPH_CAPIGetLastError(Handle)) : string.Empty;
        public string RuntimeVersion => AsterCoreNative.ReadCString(AsterCoreNative.ACPH_CAPIGetVersionString());

        public AsterCoreWorld(AsterCoreWorldSettings? settings = null)
        {
            AsterCoreNative.ACPH_CAPIDefaultWorldSettings(out var localSettings);
            if (settings.HasValue)
            {
                localSettings = settings.Value;
            }

            Handle = AsterCoreNative.ACPH_CAPICreateWorld(ref localSettings);
            if (!IsValid)
            {
                throw new InvalidOperationException("Failed to create AsterCore world.");
            }
        }

        public Vector3 Gravity
        {
            get => IsValid ? AsterCoreNative.ACPH_CAPIGetGravity(Handle).ToUnity() : Physics.gravity;
            set
            {
                if (IsValid)
                {
                    AsterCoreNative.ACPH_CAPISetGravity(Handle, AsterCoreVec3.FromUnity(value));
                }
            }
        }

        public void Reset()
        {
            if (IsValid)
            {
                AsterCoreNative.ACPH_CAPIResetWorld(Handle);
            }
        }

        public void Step(float deltaTime, uint collisionSteps = 1)
        {
            if (!IsValid)
            {
                return;
            }

            if (AsterCoreNative.ACPH_CAPIWorldStep(Handle, deltaTime, collisionSteps < 1 ? 1u : collisionSteps) == 0)
            {
                Debug.LogWarning($"AsterCore step reported an error: {LastError}");
            }
        }

        public AsterCoreBody CreateBox(Vector3 halfExtents, Vector3 position, Quaternion rotation, bool isDynamic = true, float mass = 1.0f)
        {
            var id = AsterCoreNative.ACPH_CAPICreateBoxBody(Handle, AsterCoreVec3.FromUnity(halfExtents), AsterCoreVec3.FromUnity(position), isDynamic ? 1 : 0, mass);
            var body = new AsterCoreBody(this, id, isDynamic, AsterCoreBodyShape.Box);
            body.Rotation = rotation;
            return body;
        }

        public AsterCoreBody CreateSphere(float radius, Vector3 position, Quaternion rotation, bool isDynamic = true, float mass = 1.0f)
        {
            var id = AsterCoreNative.ACPH_CAPICreateSphereBody(Handle, radius, AsterCoreVec3.FromUnity(position), isDynamic ? 1 : 0, mass);
            var body = new AsterCoreBody(this, id, isDynamic, AsterCoreBodyShape.Sphere);
            body.Rotation = rotation;
            return body;
        }

        internal bool HasBody(uint bodyId) => IsValid && AsterCoreNative.ACPH_CAPIHasBody(Handle, bodyId) != 0;
        internal bool IsBodyActive(uint bodyId) => IsValid && AsterCoreNative.ACPH_CAPIIsBodyActive(Handle, bodyId) != 0;
        internal Vector3 GetBodyPosition(uint bodyId) => AsterCoreNative.ACPH_CAPIGetBodyPosition(Handle, bodyId).ToUnity();
        internal void SetBodyPosition(uint bodyId, Vector3 position) => AsterCoreNative.ACPH_CAPISetBodyPosition(Handle, bodyId, AsterCoreVec3.FromUnity(position));
        internal Quaternion GetBodyRotation(uint bodyId) => AsterCoreNative.ACPH_CAPIGetBodyRotation(Handle, bodyId).ToUnity();
        internal void SetBodyRotation(uint bodyId, Quaternion rotation) => AsterCoreNative.ACPH_CAPISetBodyRotation(Handle, bodyId, AsterCoreQuat.FromUnity(rotation));
        internal Vector3 GetBodyLinearVelocity(uint bodyId) => AsterCoreNative.ACPH_CAPIGetBodyLinearVelocity(Handle, bodyId).ToUnity();
        internal void SetBodyLinearVelocity(uint bodyId, Vector3 velocity) => AsterCoreNative.ACPH_CAPISetBodyLinearVelocity(Handle, bodyId, AsterCoreVec3.FromUnity(velocity));
        internal Vector3 GetBodyAngularVelocity(uint bodyId) => AsterCoreNative.ACPH_CAPIGetBodyAngularVelocity(Handle, bodyId).ToUnity();
        internal void SetBodyAngularVelocity(uint bodyId, Vector3 velocity) => AsterCoreNative.ACPH_CAPISetBodyAngularVelocity(Handle, bodyId, AsterCoreVec3.FromUnity(velocity));
        internal void ApplyBodyImpulse(uint bodyId, Vector3 impulse) => AsterCoreNative.ACPH_CAPIApplyBodyImpulse(Handle, bodyId, AsterCoreVec3.FromUnity(impulse));
        internal bool DestroyBody(uint bodyId) => IsValid && AsterCoreNative.ACPH_CAPIDestroyBody(Handle, bodyId) != 0;

        public bool Raycast(Vector3 origin, Vector3 direction, float maxDistance, out AsterCoreRaycastHit hit)
        {
            hit = default;
            return IsValid && AsterCoreNative.ACPH_CAPIRaycast(Handle, AsterCoreVec3.FromUnity(origin), AsterCoreVec3.FromUnity(direction), maxDistance, out hit) != 0;
        }

        public void Dispose()
        {
            if (Handle != IntPtr.Zero)
            {
                AsterCoreNative.ACPH_CAPIDestroyWorld(Handle);
                Handle = IntPtr.Zero;
            }
        }
    }
}

