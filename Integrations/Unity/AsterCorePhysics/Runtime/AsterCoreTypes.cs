using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace AsterCore.Unity
{
    [StructLayout(LayoutKind.Sequential)]
    public struct AsterCoreVec3
    {
        public float x;
        public float y;
        public float z;

        public AsterCoreVec3(float inX, float inY, float inZ)
        {
            x = inX;
            y = inY;
            z = inZ;
        }

        public static AsterCoreVec3 FromUnity(Vector3 value) => new AsterCoreVec3(value.x, value.y, value.z);
        public Vector3 ToUnity() => new Vector3(x, y, z);
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct AsterCoreQuat
    {
        public float x;
        public float y;
        public float z;
        public float w;

        public static AsterCoreQuat FromUnity(Quaternion value)
        {
            return new AsterCoreQuat { x = value.x, y = value.y, z = value.z, w = value.w };
        }

        public Quaternion ToUnity() => new Quaternion(x, y, z, w);
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct AsterCoreWorldSettings
    {
        public uint max_bodies;
        public uint max_body_pairs;
        public uint max_contact_constraints;
        public uint temp_allocator_size_mb;
        public uint worker_threads;
        public AsterCoreVec3 gravity;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct AsterCoreRaycastHit
    {
        public int hit;
        public uint body_id;
        public float fraction;
        public AsterCoreVec3 position;
        public AsterCoreVec3 normal;
    }
}
