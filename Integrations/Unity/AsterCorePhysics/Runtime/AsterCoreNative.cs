using System;
using System.Runtime.InteropServices;

namespace AsterCore.Unity
{
    internal static class AsterCoreNative
    {
        private const string LibraryName = "AsterCoreCAPI";

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int ACPH_CAPIInitialize();

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ACPH_CAPIShutdown();

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ACPH_CAPIGetVersionString();

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ACPH_CAPIDefaultWorldSettings(out AsterCoreWorldSettings settings);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ACPH_CAPICreateWorld(ref AsterCoreWorldSettings settings);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ACPH_CAPIDestroyWorld(IntPtr world);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int ACPH_CAPIIsWorldValid(IntPtr world);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ACPH_CAPIGetLastError(IntPtr world);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ACPH_CAPIResetWorld(IntPtr world);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ACPH_CAPISetGravity(IntPtr world, AsterCoreVec3 gravity);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern AsterCoreVec3 ACPH_CAPIGetGravity(IntPtr world);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int ACPH_CAPIWorldStep(IntPtr world, float deltaTime, uint collisionSteps);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern uint ACPH_CAPICreateBoxBody(IntPtr world, AsterCoreVec3 halfExtents, AsterCoreVec3 position, int isDynamic, float mass);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern uint ACPH_CAPICreateSphereBody(IntPtr world, float radius, AsterCoreVec3 position, int isDynamic, float mass);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int ACPH_CAPIDestroyBody(IntPtr world, uint bodyId);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int ACPH_CAPIHasBody(IntPtr world, uint bodyId);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int ACPH_CAPIIsBodyActive(IntPtr world, uint bodyId);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern AsterCoreVec3 ACPH_CAPIGetBodyPosition(IntPtr world, uint bodyId);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ACPH_CAPISetBodyPosition(IntPtr world, uint bodyId, AsterCoreVec3 position);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern AsterCoreQuat ACPH_CAPIGetBodyRotation(IntPtr world, uint bodyId);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ACPH_CAPISetBodyRotation(IntPtr world, uint bodyId, AsterCoreQuat rotation);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ACPH_CAPISetBodyLinearVelocity(IntPtr world, uint bodyId, AsterCoreVec3 velocity);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern AsterCoreVec3 ACPH_CAPIGetBodyLinearVelocity(IntPtr world, uint bodyId);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ACPH_CAPISetBodyAngularVelocity(IntPtr world, uint bodyId, AsterCoreVec3 velocity);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern AsterCoreVec3 ACPH_CAPIGetBodyAngularVelocity(IntPtr world, uint bodyId);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ACPH_CAPIApplyBodyImpulse(IntPtr world, uint bodyId, AsterCoreVec3 impulse);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int ACPH_CAPIRaycast(IntPtr world, AsterCoreVec3 origin, AsterCoreVec3 direction, float maxDistance, out AsterCoreRaycastHit hit);

        internal static string ReadCString(IntPtr ptr)
        {
            return ptr == IntPtr.Zero ? string.Empty : Marshal.PtrToStringAnsi(ptr) ?? string.Empty;
        }
    }
}
