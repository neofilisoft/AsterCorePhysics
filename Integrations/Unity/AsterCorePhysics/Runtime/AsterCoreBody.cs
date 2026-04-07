using UnityEngine;

namespace AsterCore.Unity
{
    public enum AsterCoreBodyShape
    {
        Box,
        Sphere
    }

    public sealed class AsterCoreBody
    {
        private readonly AsterCoreWorld _world;

        internal AsterCoreBody(AsterCoreWorld world, uint id, bool isDynamic, AsterCoreBodyShape shape)
        {
            _world = world;
            Id = id;
            IsDynamic = isDynamic;
            Shape = shape;
        }

        public uint Id { get; }
        public bool IsDynamic { get; }
        public AsterCoreBodyShape Shape { get; }
        public bool IsValid => _world != null && _world.HasBody(Id);
        public bool IsActive => IsValid && _world.IsBodyActive(Id);

        public Vector3 Position
        {
            get => _world.GetBodyPosition(Id);
            set => _world.SetBodyPosition(Id, value);
        }

        public Quaternion Rotation
        {
            get => _world.GetBodyRotation(Id);
            set => _world.SetBodyRotation(Id, value);
        }

        public Vector3 LinearVelocity
        {
            get => _world.GetBodyLinearVelocity(Id);
            set => _world.SetBodyLinearVelocity(Id, value);
        }

        public Vector3 AngularVelocity
        {
            get => _world.GetBodyAngularVelocity(Id);
            set => _world.SetBodyAngularVelocity(Id, value);
        }

        public void ApplyImpulse(Vector3 impulse)
        {
            _world.ApplyBodyImpulse(Id, impulse);
        }

        public void Destroy()
        {
            _world.DestroyBody(Id);
        }
    }
}
