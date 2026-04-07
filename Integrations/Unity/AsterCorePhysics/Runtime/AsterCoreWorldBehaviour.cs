using System;
using UnityEngine;

namespace AsterCore.Unity
{
    [DisallowMultipleComponent]
    public class AsterCoreWorldBehaviour : MonoBehaviour
    {
        [SerializeField] private Vector3 gravity = new Vector3(0.0f, -9.81f, 0.0f);
        [SerializeField] private uint collisionSteps = 1;
        [SerializeField] private bool autoStep = true;

        public AsterCoreWorld World { get; private set; }

        private void Awake()
        {
            if (World != null)
            {
                return;
            }

            World = new AsterCoreWorld();
            World.Gravity = gravity;
        }

        private void FixedUpdate()
        {
            if (autoStep && World != null)
            {
                World.Step(Time.fixedDeltaTime, collisionSteps);
            }
        }

        private void OnDestroy()
        {
            World?.Dispose();
            World = null;
        }

        public void ResetWorld()
        {
            World?.Reset();
        }
    }
}
