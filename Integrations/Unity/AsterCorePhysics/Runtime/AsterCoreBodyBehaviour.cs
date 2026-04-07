using UnityEngine;

namespace AsterCore.Unity
{
    [DisallowMultipleComponent]
    public class AsterCoreBodyBehaviour : MonoBehaviour
    {
        [SerializeField] private AsterCoreWorldBehaviour worldBehaviour;
        [SerializeField] private AsterCoreBodyShape shape = AsterCoreBodyShape.Box;
        [SerializeField] private bool dynamicBody = true;
        [SerializeField] private float mass = 1.0f;
        [SerializeField] private Vector3 halfExtents = Vector3.one * 0.5f;
        [SerializeField] private float radius = 0.5f;
        [SerializeField] private bool createOnStart = true;

        public AsterCoreBody Body { get; private set; }

        private void Awake()
        {
            if (worldBehaviour == null)
            {
                worldBehaviour = FindObjectOfType<AsterCoreWorldBehaviour>();
            }

            if (TryGetComponent<Rigidbody>(out _))
            {
                Debug.LogWarning($"{name} has both Rigidbody and AsterCoreBodyBehaviour. Remove Rigidbody to avoid conflicting simulation.");
            }
        }

        private void Start()
        {
            if (createOnStart)
            {
                CreateBody();
            }
        }

        private void FixedUpdate()
        {
            if (Body == null || !Body.IsValid)
            {
                return;
            }

            if (dynamicBody)
            {
                transform.SetPositionAndRotation(Body.Position, Body.Rotation);
            }
            else
            {
                Body.Position = transform.position;
                Body.Rotation = transform.rotation;
            }
        }

        public void CreateBody()
        {
            if (worldBehaviour == null || worldBehaviour.World == null || Body != null)
            {
                return;
            }

            Body = shape == AsterCoreBodyShape.Sphere
                ? worldBehaviour.World.CreateSphere(radius, transform.position, transform.rotation, dynamicBody, mass)
                : worldBehaviour.World.CreateBox(halfExtents, transform.position, transform.rotation, dynamicBody, mass);
        }

        public void DestroyBody()
        {
            Body?.Destroy();
            Body = null;
        }

        public void ApplyImpulse(Vector3 impulse)
        {
            Body?.ApplyImpulse(impulse);
        }
    }
}
