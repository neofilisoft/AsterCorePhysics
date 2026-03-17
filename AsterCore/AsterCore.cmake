# Requires C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Root
set(ASTER_PHYSICS_ROOT ${PHYSICS_REPO_ROOT}/AsterCore)

# Source files
set(ASTER_PHYSICS_SRC_FILES
	${ASTER_PHYSICS_ROOT}/AABBTree/AABBTreeBuilder.cpp
	${ASTER_PHYSICS_ROOT}/AABBTree/AABBTreeBuilder.h
	${ASTER_PHYSICS_ROOT}/AABBTree/AABBTreeToBuffer.h
	${ASTER_PHYSICS_ROOT}/AABBTree/NodeCodec/NodeCodecQuadTreeHalfFloat.h
	${ASTER_PHYSICS_ROOT}/AABBTree/TriangleCodec/TriangleCodecIndexed8BitPackSOA4Flags.h
	${ASTER_PHYSICS_ROOT}/ConfigurationString.h
	${ASTER_PHYSICS_ROOT}/Core/ARMNeon.h
	${ASTER_PHYSICS_ROOT}/Core/Array.h
	${ASTER_PHYSICS_ROOT}/Core/Atomics.h
	${ASTER_PHYSICS_ROOT}/Core/BinaryHeap.h
	${ASTER_PHYSICS_ROOT}/Core/ByteBuffer.h
	${ASTER_PHYSICS_ROOT}/Core/Color.cpp
	${ASTER_PHYSICS_ROOT}/Core/Color.h
	${ASTER_PHYSICS_ROOT}/Core/Core.h
	${ASTER_PHYSICS_ROOT}/Core/Factory.cpp
	${ASTER_PHYSICS_ROOT}/Core/Factory.h
	${ASTER_PHYSICS_ROOT}/Core/FixedSizeFreeList.h
	${ASTER_PHYSICS_ROOT}/Core/FixedSizeFreeList.inl
	${ASTER_PHYSICS_ROOT}/Core/FPControlWord.h
	${ASTER_PHYSICS_ROOT}/Core/FPException.h
	${ASTER_PHYSICS_ROOT}/Core/FPFlushDenormals.h
	${ASTER_PHYSICS_ROOT}/Core/HashCombine.h
	${ASTER_PHYSICS_ROOT}/Core/HashTable.h
	${ASTER_PHYSICS_ROOT}/Core/InsertionSort.h
	${ASTER_PHYSICS_ROOT}/Core/IssueReporting.cpp
	${ASTER_PHYSICS_ROOT}/Core/IssueReporting.h
	${ASTER_PHYSICS_ROOT}/Core/JobSystem.h
	${ASTER_PHYSICS_ROOT}/Core/JobSystem.inl
	${ASTER_PHYSICS_ROOT}/Core/JobSystemSingleThreaded.cpp
	${ASTER_PHYSICS_ROOT}/Core/JobSystemSingleThreaded.h
	${ASTER_PHYSICS_ROOT}/Core/JobSystemThreadPool.cpp
	${ASTER_PHYSICS_ROOT}/Core/JobSystemThreadPool.h
	${ASTER_PHYSICS_ROOT}/Core/JobSystemWithBarrier.cpp
	${ASTER_PHYSICS_ROOT}/Core/JobSystemWithBarrier.h
	${ASTER_PHYSICS_ROOT}/Core/LinearCurve.cpp
	${ASTER_PHYSICS_ROOT}/Core/LinearCurve.h
	${ASTER_PHYSICS_ROOT}/Core/LockFreeHashMap.h
	${ASTER_PHYSICS_ROOT}/Core/LockFreeHashMap.inl
	${ASTER_PHYSICS_ROOT}/Core/Memory.cpp
	${ASTER_PHYSICS_ROOT}/Core/Memory.h
	${ASTER_PHYSICS_ROOT}/Core/Mutex.h
	${ASTER_PHYSICS_ROOT}/Core/MutexArray.h
	${ASTER_PHYSICS_ROOT}/Core/NonCopyable.h
	${ASTER_PHYSICS_ROOT}/Core/Profiler.cpp
	${ASTER_PHYSICS_ROOT}/Core/Profiler.h
	${ASTER_PHYSICS_ROOT}/Core/Profiler.inl
	${ASTER_PHYSICS_ROOT}/Core/QuickSort.h
	${ASTER_PHYSICS_ROOT}/Core/Reference.h
	${ASTER_PHYSICS_ROOT}/Core/Result.h
	${ASTER_PHYSICS_ROOT}/Core/RTTI.cpp
	${ASTER_PHYSICS_ROOT}/Core/RTTI.h
	${ASTER_PHYSICS_ROOT}/Core/ScopeExit.h
	${ASTER_PHYSICS_ROOT}/Core/Semaphore.cpp
	${ASTER_PHYSICS_ROOT}/Core/Semaphore.h
	${ASTER_PHYSICS_ROOT}/Core/StaticArray.h
	${ASTER_PHYSICS_ROOT}/Core/STLAlignedAllocator.h
	${ASTER_PHYSICS_ROOT}/Core/STLAllocator.h
	${ASTER_PHYSICS_ROOT}/Core/STLLocalAllocator.h
	${ASTER_PHYSICS_ROOT}/Core/STLTempAllocator.h
	${ASTER_PHYSICS_ROOT}/Core/StreamIn.h
	${ASTER_PHYSICS_ROOT}/Core/StreamOut.h
	${ASTER_PHYSICS_ROOT}/Core/StreamUtils.h
	${ASTER_PHYSICS_ROOT}/Core/StreamWrapper.h
	${ASTER_PHYSICS_ROOT}/Core/StridedPtr.h
	${ASTER_PHYSICS_ROOT}/Core/StringTools.cpp
	${ASTER_PHYSICS_ROOT}/Core/StringTools.h
	${ASTER_PHYSICS_ROOT}/Core/TempAllocator.h
	${ASTER_PHYSICS_ROOT}/Core/TickCounter.cpp
	${ASTER_PHYSICS_ROOT}/Core/TickCounter.h
	${ASTER_PHYSICS_ROOT}/Core/UnorderedMap.h
	${ASTER_PHYSICS_ROOT}/Core/UnorderedSet.h
	${ASTER_PHYSICS_ROOT}/Geometry/AABox.h
	${ASTER_PHYSICS_ROOT}/Geometry/AABox4.h
	${ASTER_PHYSICS_ROOT}/Geometry/ClipPoly.h
	${ASTER_PHYSICS_ROOT}/Geometry/ClosestPoint.h
	${ASTER_PHYSICS_ROOT}/Geometry/ConvexHullBuilder.cpp
	${ASTER_PHYSICS_ROOT}/Geometry/ConvexHullBuilder.h
	${ASTER_PHYSICS_ROOT}/Geometry/ConvexHullBuilder2D.cpp
	${ASTER_PHYSICS_ROOT}/Geometry/ConvexHullBuilder2D.h
	${ASTER_PHYSICS_ROOT}/Geometry/ConvexSupport.h
	${ASTER_PHYSICS_ROOT}/Geometry/Ellipse.h
	${ASTER_PHYSICS_ROOT}/Geometry/EPAConvexHullBuilder.h
	${ASTER_PHYSICS_ROOT}/Geometry/EPAPenetrationDepth.h
	${ASTER_PHYSICS_ROOT}/Geometry/GJKClosestPoint.h
	${ASTER_PHYSICS_ROOT}/Geometry/IndexedTriangle.h
	${ASTER_PHYSICS_ROOT}/Geometry/Indexify.cpp
	${ASTER_PHYSICS_ROOT}/Geometry/Indexify.h
	${ASTER_PHYSICS_ROOT}/Geometry/MortonCode.h
	${ASTER_PHYSICS_ROOT}/Geometry/OrientedBox.cpp
	${ASTER_PHYSICS_ROOT}/Geometry/OrientedBox.h
	${ASTER_PHYSICS_ROOT}/Geometry/Plane.h
	${ASTER_PHYSICS_ROOT}/Geometry/RayAABox.h
	${ASTER_PHYSICS_ROOT}/Geometry/RayCapsule.h
	${ASTER_PHYSICS_ROOT}/Geometry/RayCylinder.h
	${ASTER_PHYSICS_ROOT}/Geometry/RaySphere.h
	${ASTER_PHYSICS_ROOT}/Geometry/RayTriangle.h
	${ASTER_PHYSICS_ROOT}/Geometry/Sphere.h
	${ASTER_PHYSICS_ROOT}/Geometry/Triangle.h
	${ASTER_PHYSICS_ROOT}/AsterCore.cmake
	${ASTER_PHYSICS_ROOT}/AsterCore.h
	${ASTER_PHYSICS_ROOT}/Math/BVec16.h
	${ASTER_PHYSICS_ROOT}/Math/BVec16.inl
	${ASTER_PHYSICS_ROOT}/Math/DMat44.h
	${ASTER_PHYSICS_ROOT}/Math/DMat44.inl
	${ASTER_PHYSICS_ROOT}/Math/Double3.h
	${ASTER_PHYSICS_ROOT}/Math/DVec3.h
	${ASTER_PHYSICS_ROOT}/Math/DVec3.inl
	${ASTER_PHYSICS_ROOT}/Math/DynMatrix.h
	${ASTER_PHYSICS_ROOT}/Math/EigenValueSymmetric.h
	${ASTER_PHYSICS_ROOT}/Math/FindRoot.h
	${ASTER_PHYSICS_ROOT}/Math/Float2.h
	${ASTER_PHYSICS_ROOT}/Math/Float3.h
	${ASTER_PHYSICS_ROOT}/Math/Float4.h
	${ASTER_PHYSICS_ROOT}/Math/GaussianElimination.h
	${ASTER_PHYSICS_ROOT}/Math/HalfFloat.h
	${ASTER_PHYSICS_ROOT}/Math/Mat44.h
	${ASTER_PHYSICS_ROOT}/Math/Mat44.inl
	${ASTER_PHYSICS_ROOT}/Math/Math.h
	${ASTER_PHYSICS_ROOT}/Math/MathTypes.h
	${ASTER_PHYSICS_ROOT}/Math/Matrix.h
	${ASTER_PHYSICS_ROOT}/Math/Quat.h
	${ASTER_PHYSICS_ROOT}/Math/Quat.inl
	${ASTER_PHYSICS_ROOT}/Math/Real.h
	${ASTER_PHYSICS_ROOT}/Math/Swizzle.h
	${ASTER_PHYSICS_ROOT}/Math/Trigonometry.h
	${ASTER_PHYSICS_ROOT}/Math/UVec4.h
	${ASTER_PHYSICS_ROOT}/Math/UVec4.inl
	${ASTER_PHYSICS_ROOT}/Math/Vec3.cpp
	${ASTER_PHYSICS_ROOT}/Math/Vec3.h
	${ASTER_PHYSICS_ROOT}/Math/Vec3.inl
	${ASTER_PHYSICS_ROOT}/Math/Vec4.h
	${ASTER_PHYSICS_ROOT}/Math/Vec4.inl
	${ASTER_PHYSICS_ROOT}/Math/Vector.h
	${ASTER_PHYSICS_ROOT}/ObjectStream/ObjectStream.h
	${ASTER_PHYSICS_ROOT}/ObjectStream/SerializableAttribute.h
	${ASTER_PHYSICS_ROOT}/ObjectStream/SerializableAttributeEnum.h
	${ASTER_PHYSICS_ROOT}/ObjectStream/SerializableAttributeTyped.h
	${ASTER_PHYSICS_ROOT}/ObjectStream/SerializableObject.cpp
	${ASTER_PHYSICS_ROOT}/ObjectStream/SerializableObject.h
	${ASTER_PHYSICS_ROOT}/ObjectStream/TypeDeclarations.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/AllowedDOFs.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/Body.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Body/Body.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/Body.inl
	${ASTER_PHYSICS_ROOT}/Physics/Body/BodyAccess.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/BodyActivationListener.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/BodyCreationSettings.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Body/BodyCreationSettings.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/BodyFilter.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/BodyID.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/BodyInterface.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Body/BodyInterface.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/BodyLock.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/BodyLockInterface.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/BodyLockMulti.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/BodyManager.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Body/BodyManager.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/BodyPair.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/BodyType.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/MassProperties.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Body/MassProperties.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/MotionProperties.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Body/MotionProperties.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/MotionProperties.inl
	${ASTER_PHYSICS_ROOT}/Physics/Body/MotionQuality.h
	${ASTER_PHYSICS_ROOT}/Physics/Body/MotionType.h
	${ASTER_PHYSICS_ROOT}/Physics/Character/Character.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Character/Character.h
	${ASTER_PHYSICS_ROOT}/Physics/Character/CharacterBase.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Character/CharacterBase.h
	${ASTER_PHYSICS_ROOT}/Physics/Character/CharacterID.h
	${ASTER_PHYSICS_ROOT}/Physics/Character/CharacterVirtual.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Character/CharacterVirtual.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/AABoxCast.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/ActiveEdgeMode.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/ActiveEdges.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/BackFaceMode.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/BroadPhase/BroadPhase.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/BroadPhase/BroadPhase.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/BroadPhase/BroadPhaseBruteForce.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/BroadPhase/BroadPhaseBruteForce.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/BroadPhase/BroadPhaseLayer.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceMask.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/BroadPhase/BroadPhaseQuadTree.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/BroadPhase/BroadPhaseQuadTree.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/BroadPhase/BroadPhaseQuery.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterMask.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/BroadPhase/QuadTree.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/BroadPhase/QuadTree.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CastConvexVsTriangles.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CastConvexVsTriangles.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CastResult.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CastSphereVsTriangles.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CastSphereVsTriangles.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CollectFacesMode.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CollideConvexVsTriangles.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CollideConvexVsTriangles.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CollidePointResult.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CollideShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CollideShapeVsShapePerLeaf.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CollideSoftBodyVertexIterator.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CollideSoftBodyVerticesVsTriangles.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CollideSphereVsTriangles.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CollideSphereVsTriangles.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CollisionCollector.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CollisionCollectorImpl.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CollisionDispatch.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CollisionDispatch.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CollisionGroup.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/CollisionGroup.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/ContactListener.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/EstimateCollisionResponse.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/EstimateCollisionResponse.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/GroupFilter.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/GroupFilter.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/GroupFilterTable.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/GroupFilterTable.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/InternalEdgeRemovingCollector.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/ManifoldBetweenTwoFaces.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/ManifoldBetweenTwoFaces.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/NarrowPhaseQuery.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/NarrowPhaseQuery.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/NarrowPhaseStats.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/NarrowPhaseStats.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/ObjectLayer.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/ObjectLayerPairFilterMask.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/ObjectLayerPairFilterTable.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/PhysicsMaterial.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/PhysicsMaterial.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/PhysicsMaterialSimple.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/PhysicsMaterialSimple.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/RayCast.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/BoxShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/BoxShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/CapsuleShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/CapsuleShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/CompoundShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/CompoundShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/CompoundShapeVisitors.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/ConvexHullShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/ConvexHullShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/ConvexShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/ConvexShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/CylinderShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/CylinderShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/DecoratedShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/DecoratedShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/EmptyShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/EmptyShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/GetTrianglesContext.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/HeightFieldShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/HeightFieldShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/MeshShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/MeshShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/MutableCompoundShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/MutableCompoundShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/OffsetCenterOfMassShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/OffsetCenterOfMassShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/PlaneShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/PlaneShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/PolyhedronSubmergedVolumeCalculator.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/RotatedTranslatedShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/RotatedTranslatedShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/ScaledShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/ScaledShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/ScaleHelpers.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/Shape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/Shape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/SphereShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/SphereShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/StaticCompoundShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/StaticCompoundShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/SubShapeID.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/SubShapeIDPair.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/TaperedCapsuleShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/TaperedCapsuleShape.gliffy
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/TaperedCapsuleShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/TaperedCylinderShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/TaperedCylinderShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/TriangleShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/Shape/TriangleShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/ShapeCast.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/ShapeFilter.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/SimShapeFilter.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/SimShapeFilterWrapper.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/SortReverseAndStore.h
	${ASTER_PHYSICS_ROOT}/Physics/Collision/TransformedShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Collision/TransformedShape.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/CalculateSolverSteps.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ConeConstraint.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ConeConstraint.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/Constraint.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/Constraint.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ConstraintManager.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ConstraintManager.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ConstraintPart/AngleConstraintPart.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ConstraintPart/AxisConstraintPart.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ConstraintPart/DualAxisConstraintPart.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ConstraintPart/GearConstraintPart.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ConstraintPart/HingeRotationConstraintPart.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ConstraintPart/IndependentAxisConstraintPart.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ConstraintPart/PointConstraintPart.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ConstraintPart/RackAndPinionConstraintPart.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ConstraintPart/RotationEulerConstraintPart.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ConstraintPart/RotationQuatConstraintPart.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ConstraintPart/SpringPart.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ConstraintPart/SwingTwistConstraintPart.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ContactConstraintManager.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/ContactConstraintManager.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/DistanceConstraint.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/DistanceConstraint.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/FixedConstraint.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/FixedConstraint.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/GearConstraint.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/GearConstraint.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/HingeConstraint.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/HingeConstraint.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/JiggleConstraint.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/JiggleConstraint.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/MotorSettings.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/MotorSettings.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/PathConstraint.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/PathConstraint.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/PathConstraintPath.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/PathConstraintPath.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/PathConstraintPathHermite.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/PathConstraintPathHermite.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/PointConstraint.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/PointConstraint.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/PulleyConstraint.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/PulleyConstraint.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/RackAndPinionConstraint.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/RackAndPinionConstraint.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/SixDOFConstraint.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/SixDOFConstraint.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/SliderConstraint.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/SliderConstraint.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/SpringSettings.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/SpringSettings.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/SwingTwistConstraint.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/SwingTwistConstraint.h
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/TwoBodyConstraint.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Constraints/TwoBodyConstraint.h
	${ASTER_PHYSICS_ROOT}/Physics/DeterminismLog.cpp
	${ASTER_PHYSICS_ROOT}/Physics/DeterminismLog.h
	${ASTER_PHYSICS_ROOT}/Physics/EActivation.h
	${ASTER_PHYSICS_ROOT}/Physics/EPhysicsUpdateError.h
	${ASTER_PHYSICS_ROOT}/Physics/IslandBuilder.cpp
	${ASTER_PHYSICS_ROOT}/Physics/IslandBuilder.h
	${ASTER_PHYSICS_ROOT}/Physics/LargeIslandSplitter.cpp
	${ASTER_PHYSICS_ROOT}/Physics/LargeIslandSplitter.h
	${ASTER_PHYSICS_ROOT}/Physics/PhysicsLock.h
	${ASTER_PHYSICS_ROOT}/Physics/PhysicsScene.cpp
	${ASTER_PHYSICS_ROOT}/Physics/PhysicsScene.h
	${ASTER_PHYSICS_ROOT}/Physics/PhysicsSettings.h
	${ASTER_PHYSICS_ROOT}/Physics/PhysicsStepListener.h
	${ASTER_PHYSICS_ROOT}/Physics/PhysicsSystem.cpp
	${ASTER_PHYSICS_ROOT}/Physics/PhysicsSystem.h
	${ASTER_PHYSICS_ROOT}/Physics/PhysicsUpdateContext.cpp
	${ASTER_PHYSICS_ROOT}/Physics/PhysicsUpdateContext.h
	${ASTER_PHYSICS_ROOT}/Physics/Ragdoll/Ragdoll.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Ragdoll/Ragdoll.h
	${ASTER_PHYSICS_ROOT}/Physics/SoftBody/SoftBodyContactListener.h
	${ASTER_PHYSICS_ROOT}/Physics/SoftBody/SoftBodyCreationSettings.cpp
	${ASTER_PHYSICS_ROOT}/Physics/SoftBody/SoftBodyCreationSettings.h
	${ASTER_PHYSICS_ROOT}/Physics/SoftBody/SoftBodyManifold.h
	${ASTER_PHYSICS_ROOT}/Physics/SoftBody/SoftBodyMotionProperties.cpp
	${ASTER_PHYSICS_ROOT}/Physics/SoftBody/SoftBodyMotionProperties.h
	${ASTER_PHYSICS_ROOT}/Physics/SoftBody/SoftBodyShape.cpp
	${ASTER_PHYSICS_ROOT}/Physics/SoftBody/SoftBodyShape.h
	${ASTER_PHYSICS_ROOT}/Physics/SoftBody/SoftBodySharedSettings.cpp
	${ASTER_PHYSICS_ROOT}/Physics/SoftBody/SoftBodySharedSettings.h
	${ASTER_PHYSICS_ROOT}/Physics/SoftBody/SoftBodyUpdateContext.h
	${ASTER_PHYSICS_ROOT}/Physics/SoftBody/SoftBodyVertex.h
	${ASTER_PHYSICS_ROOT}/Physics/Fluid/ParticleFluidSharedBuffer.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Fluid/ParticleFluidSharedBuffer.h
	${ASTER_PHYSICS_ROOT}/Physics/Fluid/ParticleFluidSystem.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Fluid/ParticleFluidSystem.h
	${ASTER_PHYSICS_ROOT}/Physics/StateRecorder.h
	${ASTER_PHYSICS_ROOT}/Physics/StateRecorderImpl.cpp
	${ASTER_PHYSICS_ROOT}/Physics/StateRecorderImpl.h
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/MotorcycleController.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/MotorcycleController.h
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/TrackedVehicleController.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/TrackedVehicleController.h
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/VehicleAntiRollBar.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/VehicleAntiRollBar.h
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/VehicleCollisionTester.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/VehicleCollisionTester.h
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/VehicleConstraint.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/VehicleConstraint.h
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/VehicleController.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/VehicleController.h
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/VehicleDifferential.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/VehicleDifferential.h
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/VehicleEngine.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/VehicleEngine.h
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/VehicleTrack.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/VehicleTrack.h
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/VehicleTransmission.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/VehicleTransmission.h
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/Wheel.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/Wheel.h
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/WheeledVehicleController.cpp
	${ASTER_PHYSICS_ROOT}/Physics/Vehicle/WheeledVehicleController.h
	${ASTER_PHYSICS_ROOT}/RegisterTypes.cpp
	${ASTER_PHYSICS_ROOT}/RegisterTypes.h
	${ASTER_PHYSICS_ROOT}/Renderer/DebugRenderer.cpp
	${ASTER_PHYSICS_ROOT}/Renderer/DebugRenderer.h
	${ASTER_PHYSICS_ROOT}/Renderer/DebugRendererPlayback.cpp
	${ASTER_PHYSICS_ROOT}/Renderer/DebugRendererPlayback.h
	${ASTER_PHYSICS_ROOT}/Renderer/DebugRendererRecorder.cpp
	${ASTER_PHYSICS_ROOT}/Renderer/DebugRendererRecorder.h
	${ASTER_PHYSICS_ROOT}/Renderer/DebugRendererSimple.cpp
	${ASTER_PHYSICS_ROOT}/Renderer/DebugRendererSimple.h
	${ASTER_PHYSICS_ROOT}/Skeleton/SkeletalAnimation.cpp
	${ASTER_PHYSICS_ROOT}/Skeleton/SkeletalAnimation.h
	${ASTER_PHYSICS_ROOT}/Skeleton/Skeleton.cpp
	${ASTER_PHYSICS_ROOT}/Skeleton/Skeleton.h
	${ASTER_PHYSICS_ROOT}/Skeleton/SkeletonMapper.cpp
	${ASTER_PHYSICS_ROOT}/Skeleton/SkeletonMapper.h
	${ASTER_PHYSICS_ROOT}/Skeleton/SkeletonPose.cpp
	${ASTER_PHYSICS_ROOT}/Skeleton/SkeletonPose.h
	${ASTER_PHYSICS_ROOT}/TriangleSplitter/TriangleSplitter.cpp
	${ASTER_PHYSICS_ROOT}/TriangleSplitter/TriangleSplitter.h
	${ASTER_PHYSICS_ROOT}/TriangleSplitter/TriangleSplitterBinning.cpp
	${ASTER_PHYSICS_ROOT}/TriangleSplitter/TriangleSplitterBinning.h
	${ASTER_PHYSICS_ROOT}/TriangleSplitter/TriangleSplitterMean.cpp
	${ASTER_PHYSICS_ROOT}/TriangleSplitter/TriangleSplitterMean.h
)

if (ENABLE_OBJECT_STREAM)
	set(ASTER_PHYSICS_SRC_FILES
		${ASTER_PHYSICS_SRC_FILES}
		${ASTER_PHYSICS_ROOT}/ObjectStream/GetPrimitiveTypeOfType.h
		${ASTER_PHYSICS_ROOT}/ObjectStream/ObjectStream.cpp
		${ASTER_PHYSICS_ROOT}/ObjectStream/ObjectStreamBinaryIn.cpp
		${ASTER_PHYSICS_ROOT}/ObjectStream/ObjectStreamBinaryIn.h
		${ASTER_PHYSICS_ROOT}/ObjectStream/ObjectStreamBinaryOut.cpp
		${ASTER_PHYSICS_ROOT}/ObjectStream/ObjectStreamBinaryOut.h
		${ASTER_PHYSICS_ROOT}/ObjectStream/ObjectStreamIn.cpp
		${ASTER_PHYSICS_ROOT}/ObjectStream/ObjectStreamIn.h
		${ASTER_PHYSICS_ROOT}/ObjectStream/ObjectStreamOut.cpp
		${ASTER_PHYSICS_ROOT}/ObjectStream/ObjectStreamOut.h
		${ASTER_PHYSICS_ROOT}/ObjectStream/ObjectStreamTextIn.cpp
		${ASTER_PHYSICS_ROOT}/ObjectStream/ObjectStreamTextIn.h
		${ASTER_PHYSICS_ROOT}/ObjectStream/ObjectStreamTextOut.cpp
		${ASTER_PHYSICS_ROOT}/ObjectStream/ObjectStreamTextOut.h
		${ASTER_PHYSICS_ROOT}/ObjectStream/ObjectStreamTypes.h
		${ASTER_PHYSICS_ROOT}/ObjectStream/TypeDeclarations.cpp
	)
endif()

if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Windows")
	# Add natvis file
	set(ASTER_PHYSICS_SRC_FILES ${ASTER_PHYSICS_SRC_FILES} ${ASTER_PHYSICS_ROOT}/AsterCore.natvis)
endif()

# Group source files
source_group(TREE ${ASTER_PHYSICS_ROOT} FILES ${ASTER_PHYSICS_SRC_FILES})

# Create AsterCore lib
add_library(AsterCore ${ASTER_PHYSICS_SRC_FILES})
add_library(AsterCore::AsterCore ALIAS AsterCore)

if (BUILD_SHARED_LIBS)
	# Set default visibility to hidden
	set(CMAKE_CXX_VISIBILITY_PRESET hidden)

	if (GENERATE_DEBUG_SYMBOLS)
		if (MSVC)
			# MSVC specific option to enable PDB generation
			set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DEBUG:FASTLINK")
		else()
			# Clang/GCC option to enable debug symbol generation
			set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} -g")
		endif()
	endif()

	# Set linker flags for other build types to be the same as release
	set(CMAKE_SHARED_LINKER_FLAGS_RELEASEASAN "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}")
	set(CMAKE_SHARED_LINKER_FLAGS_RELEASEUBSAN "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}")
	set(CMAKE_SHARED_LINKER_FLAGS_RELEASETSAN "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}")
	set(CMAKE_SHARED_LINKER_FLAGS_RELEASECOVERAGE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}")
	set(CMAKE_SHARED_LINKER_FLAGS_DISTRIBUTION "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}")

	# Public define to instruct user code to import AsterCore symbols (rather than use static linking)
	target_compile_definitions(AsterCore PUBLIC ACPH_SHARED_LIBRARY)

	# Private define to instruct the library to export symbols for shared linking
	target_compile_definitions(AsterCore PRIVATE ACPH_BUILD_SHARED_LIBRARY)
endif()

# Use repository as include directory when building, install directory when installing
target_include_directories(AsterCore PUBLIC
	$<BUILD_INTERFACE:${PHYSICS_REPO_ROOT}>
	$<INSTALL_INTERFACE:include/>)

# Code coverage doesn't work when using precompiled headers
if (CMAKE_GENERATOR STREQUAL "Ninja Multi-Config" AND MSVC)
	# The Ninja Multi-Config generator errors out when selectively disabling precompiled headers for certain configurations.
	# See: https://github.com/jrouwe/JoltPhysics/issues/1211
	target_precompile_headers(AsterCore PRIVATE "${ASTER_PHYSICS_ROOT}/AsterCore.h")
else()
	target_precompile_headers(AsterCore PRIVATE "$<$<NOT:$<CONFIG:ReleaseCoverage>>:${ASTER_PHYSICS_ROOT}/AsterCore.h>")
endif()

# Set the debug/non-debug build flags
target_compile_definitions(AsterCore PUBLIC "$<$<CONFIG:Debug>:_DEBUG>")
target_compile_definitions(AsterCore PUBLIC "$<$<CONFIG:Release,Distribution,ReleaseASAN,ReleaseUBSAN,ReleaseTSAN,ReleaseCoverage>:NDEBUG>")

# ASAN and TSAN should use the default allocators
target_compile_definitions(AsterCore PUBLIC "$<$<CONFIG:ReleaseASAN,ReleaseTSAN>:ACPH_DISABLE_TEMP_ALLOCATOR;ACPH_DISABLE_CUSTOM_ALLOCATOR>")

# Setting floating point exceptions
if (FLOATING_POINT_EXCEPTIONS_ENABLED AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	target_compile_definitions(AsterCore PUBLIC "$<$<CONFIG:Debug,Release>:ACPH_FLOATING_POINT_EXCEPTIONS_ENABLED>")
endif()

# Setting the disable custom allocator flag
if (DISABLE_CUSTOM_ALLOCATOR)
	target_compile_definitions(AsterCore PUBLIC ACPH_DISABLE_CUSTOM_ALLOCATOR)
endif()

# Setting enable asserts flag
if (USE_ASSERTS)
	target_compile_definitions(AsterCore PUBLIC ACPH_ENABLE_ASSERTS)
endif()

# Setting double precision flag
if (DOUBLE_PRECISION)
	target_compile_definitions(AsterCore PUBLIC ACPH_DOUBLE_PRECISION)
endif()

# Setting to attempt cross platform determinism
if (CROSS_PLATFORM_DETERMINISTIC)
	target_compile_definitions(AsterCore PUBLIC ACPH_CROSS_PLATFORM_DETERMINISTIC)
endif()

# Setting to determine number of bits in ObjectLayer
if (OBJECT_LAYER_BITS)
	target_compile_definitions(AsterCore PUBLIC ACPH_OBJECT_LAYER_BITS=${OBJECT_LAYER_BITS})
endif()

if (USE_STD_VECTOR)
	target_compile_definitions(AsterCore PUBLIC ACPH_USE_STD_VECTOR)
endif()

# Setting to periodically trace broadphase stats to help determine if the broadphase layer configuration is optimal
if (TRACK_BROADPHASE_STATS)
	target_compile_definitions(AsterCore PUBLIC ACPH_TRACK_BROADPHASE_STATS)
endif()

# Setting to periodically trace narrowphase stats to help determine which collision queries could be optimized
if (TRACK_NARROWPHASE_STATS)
	target_compile_definitions(AsterCore PUBLIC ACPH_TRACK_NARROWPHASE_STATS)
endif()

# Enable the debug renderer
if (DEBUG_RENDERER_IN_DISTRIBUTION)
	target_compile_definitions(AsterCore PUBLIC "ACPH_DEBUG_RENDERER")
elseif (DEBUG_RENDERER_IN_DEBUG_AND_RELEASE)
	target_compile_definitions(AsterCore PUBLIC "$<$<CONFIG:Debug,Release,ReleaseASAN,ReleaseUBSAN,ReleaseTSAN>:ACPH_DEBUG_RENDERER>")
endif()

# Enable the profiler
if (ACPH_USE_EXTERNAL_PROFILE)
	set(JOLT_PROFILE_DEFINE ACPH_EXTERNAL_PROFILE)
else()
	set(JOLT_PROFILE_DEFINE ACPH_PROFILE_ENABLED)
endif()
if (PROFILER_IN_DISTRIBUTION)
	target_compile_definitions(AsterCore PUBLIC "${JOLT_PROFILE_DEFINE}")
elseif (PROFILER_IN_DEBUG_AND_RELEASE)
	target_compile_definitions(AsterCore PUBLIC "$<$<CONFIG:Debug,Release,ReleaseASAN,ReleaseUBSAN,ReleaseTSAN>:${JOLT_PROFILE_DEFINE}>")
endif()

# Compile the ObjectStream class and RTTI attribute information
if (ENABLE_OBJECT_STREAM)
	target_compile_definitions(AsterCore PUBLIC ACPH_OBJECT_STREAM)
endif()

# Emit the instruction set definitions to ensure that child projects use the same settings even if they override the used instruction sets (a mismatch causes link errors)
function(EMIT_X86_INSTRUCTION_SET_DEFINITIONS)
	if (USE_AVX512)
		target_compile_definitions(AsterCore PUBLIC ACPH_USE_AVX512)
	endif()
	if (USE_AVX2)
		target_compile_definitions(AsterCore PUBLIC ACPH_USE_AVX2)
	endif()
	if (USE_AVX)
		target_compile_definitions(AsterCore PUBLIC ACPH_USE_AVX)
	endif()
	if (USE_SSE4_1)
		target_compile_definitions(AsterCore PUBLIC ACPH_USE_SSE4_1)
	endif()
	if (USE_SSE4_2)
		target_compile_definitions(AsterCore PUBLIC ACPH_USE_SSE4_2)
	endif()
	if (USE_LZCNT)
		target_compile_definitions(AsterCore PUBLIC ACPH_USE_LZCNT)
	endif()
	if (USE_TZCNT)
		target_compile_definitions(AsterCore PUBLIC ACPH_USE_TZCNT)
	endif()
	if (USE_F16C)
		target_compile_definitions(AsterCore PUBLIC ACPH_USE_F16C)
	endif()
	if (USE_FMADD AND NOT CROSS_PLATFORM_DETERMINISTIC)
		target_compile_definitions(AsterCore PUBLIC ACPH_USE_FMADD)
	endif()
endfunction()

# Add the compiler commandline flags to select the right instruction sets
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	if ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "x86" OR "${CMAKE_VS_PLATFORM_NAME}" STREQUAL "x64")
		if (USE_AVX512)
			target_compile_options(AsterCore PUBLIC /arch:AVX512)
		elseif (USE_AVX2)
			target_compile_options(AsterCore PUBLIC /arch:AVX2)
		elseif (USE_AVX)
			target_compile_options(AsterCore PUBLIC /arch:AVX)
		endif()
		EMIT_X86_INSTRUCTION_SET_DEFINITIONS()
	endif()
else()
	if (XCODE)
		# XCode builds for multiple architectures, we can't set global flags
	elseif (CROSS_COMPILE_ARM OR CMAKE_OSX_ARCHITECTURES MATCHES "arm64" OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
		# ARM64 uses no special commandline flags
	elseif (EMSCRIPTEN)
		if (USE_WASM_SIMD)
			# AsterCore currently doesn't implement the WASM specific SIMD intrinsics so uses the SSE 4.2 intrinsics
			# See: https://emscripten.org/docs/porting/simd.html#webassembly-simd-intrinsics
			# Note that this does not require the browser to actually support SSE 4.2 it merely means that it can translate those instructions to WASM SIMD instructions
			target_compile_options(AsterCore PUBLIC -msimd128 -msse4.2)
		endif()
		if (ACPH_USE_WASM64)
			target_compile_options(AsterCore PUBLIC -sMEMORY64)
			target_link_options(AsterCore PUBLIC -sMEMORY64)
		endif()
	elseif ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64" OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "AMD64" OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86" OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "i386")
		# x86 and x86_64
		# On 32-bit builds we need to default to using SSE instructions, the x87 FPU instructions have higher intermediate precision
		# which will cause problems in the collision detection code (the effect is similar to leaving FMA on, search for
		# ACPH_PRECISE_MATH_ON for the locations where this is a problem).

		if (USE_AVX512)
			target_compile_options(AsterCore PUBLIC -mavx512f -mavx512vl -mavx512dq -mavx2 -mbmi -mpopcnt -mlzcnt -mf16c)
		elseif (USE_AVX2)
			target_compile_options(AsterCore PUBLIC -mavx2 -mbmi -mpopcnt -mlzcnt -mf16c)
		elseif (USE_AVX)
			target_compile_options(AsterCore PUBLIC -mavx -mpopcnt)
		elseif (USE_SSE4_2)
			target_compile_options(AsterCore PUBLIC -msse4.2 -mpopcnt)
		elseif (USE_SSE4_1)
			target_compile_options(AsterCore PUBLIC -msse4.1)
		else()
			target_compile_options(AsterCore PUBLIC -msse2)
		endif()
		if (USE_LZCNT)
			target_compile_options(AsterCore PUBLIC -mlzcnt)
		endif()
		if (USE_TZCNT)
			target_compile_options(AsterCore PUBLIC -mbmi)
		endif()
		if (USE_F16C)
			target_compile_options(AsterCore PUBLIC -mf16c)
		endif()
		if (USE_FMADD AND NOT CROSS_PLATFORM_DETERMINISTIC)
			target_compile_options(AsterCore PUBLIC -mfma)
		endif()

		if (NOT MSVC)
			target_compile_options(AsterCore PUBLIC -mfpmath=sse)
		endif()

		EMIT_X86_INSTRUCTION_SET_DEFINITIONS()
	endif()
endif()

# On Unix flavors we need the pthread library
if (NOT ("${CMAKE_SYSTEM_NAME}" STREQUAL "Windows") AND NOT EMSCRIPTEN)
	target_compile_options(AsterCore PUBLIC -pthread)
	target_link_options(AsterCore PUBLIC -pthread)
endif()

if (EMSCRIPTEN)
	# We need more than the default 64KB stack and 16MB memory
	# In your application, specify at least -sSTACK_SIZE=1048576 -sINITIAL_MEMORY=134217728
	# Also disable warning: running limited binaryen optimizations because DWARF info requested (or indirectly required)
	target_link_options(AsterCore PUBLIC -Wno-limited-postlink-optimizations)
endif()
