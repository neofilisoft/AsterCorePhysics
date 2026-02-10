#include <vector>
#include <memory>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <iostream>

#if defined(_WIN32)
#include <malloc.h>
#else
#include <stdlib.h>
#endif

class AsterAllocator {
public:
    virtual void* allocate(size_t size, size_t alignment) = 0;
    virtual void deallocate(void* ptr) = 0;
    virtual ~AsterAllocator() = default;
};

class DefaultAllocator : public AsterAllocator {
public:
    void* allocate(size_t size, size_t alignment) override {
#if defined(_WIN32)
        return _aligned_malloc(size, alignment);
#else
        return aligned_alloc(alignment, size);
#endif
    }
    
    void deallocate(void* ptr) override {
#if defined(_WIN32)
        _aligned_free(ptr);
#else
        free(ptr);
#endif
    }
};

struct Vector3 {
    float x, y, z;

    Vector3(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}
    
    float magnitude() const {
        return std::sqrt(x*x + y*y + z*z);
    }
    
    Vector3 normalized() const {
        float mag = magnitude();
        return mag > 0 ? Vector3(x/mag, y/mag, z/mag) : Vector3();
    }
    
    Vector3 operator+(const Vector3& rhs) const {
        return Vector3(x+rhs.x, y+rhs.y, z+rhs.z);
    }
    
    Vector3 operator-(const Vector3& rhs) const {
        return Vector3(x-rhs.x, y-rhs.y, z-rhs.z);
    }
    
    Vector3 operator*(float scalar) const {
        return Vector3(x*scalar, y*scalar, z*scalar);
    }
    
    Vector3& operator+=(const Vector3& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    float dot(const Vector3& rhs) const {
        return x*rhs.x + y*rhs.y + z*rhs.z;
    }
};

enum CollisionShapeType {
    SHAPE_SPHERE,
    SHAPE_BOX
};

class CollisionShape {
public:
    virtual ~CollisionShape() = default;
    virtual CollisionShapeType getType() const = 0;
    virtual Vector3 getCenter() const = 0;
};

class SphereShape : public CollisionShape {
public:
    float radius;
    Vector3 center;
    
    SphereShape(float r) : radius(r) {}
    CollisionShapeType getType() const override { return SHAPE_SPHERE; }
    Vector3 getCenter() const override { return center; }
};

class BoxShape : public CollisionShape {
public:
    Vector3 halfExtents;
    Vector3 center;
    
    BoxShape(float w, float h, float d) : halfExtents(w/2, h/2, d/2) {}
    CollisionShapeType getType() const override { return SHAPE_BOX; }
    Vector3 getCenter() const override { return center; }
};

struct RigidBodyState {
    Vector3 position;
    Vector3 linearVelocity;
};

class RigidBody {
public:
    RigidBodyState currentState;
    RigidBodyState previousState;
    float mass = 1.0f;
    float restitution = 0.5f;
    float friction = 0.2f;
    std::unique_ptr<CollisionShape> collisionShape;
    void* userData = nullptr;
};

struct ContactPoint {
    Vector3 position;
    Vector3 normal;      // A to B (normal point from A)
    float penetration;
};

struct CollisionPair {
    RigidBody* bodyA;
    RigidBody* bodyB;
    std::vector<ContactPoint> contacts;
};

class Broadphase {
public:
    virtual void update(std::vector<RigidBody*>& bodies) = 0;
    virtual void findPotentialPairs(std::vector<CollisionPair>& pairs) = 0;
    virtual ~Broadphase() = default;
};

class SAPBroadphase : public Broadphase {
    std::vector<RigidBody*> bodies;
    
public:
    void update(std::vector<RigidBody*>& newBodies) override {
        bodies = newBodies;
    }
    
    void findPotentialPairs(std::vector<CollisionPair>& pairs) override {
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                pairs.push_back({bodies[i], bodies[j]});
            }
        }
    }
};

class Narrowphase {
public:
    bool computeContacts(CollisionPair& pair) {
        if (!pair.bodyA->collisionShape || !pair.bodyB->collisionShape) 
            return false;

        CollisionShapeType typeA = pair.bodyA->collisionShape->getType();
        CollisionShapeType typeB = pair.bodyB->collisionShape->getType();

        // Sphere vs Sphere
        if (typeA == SHAPE_SPHERE && typeB == SHAPE_SPHERE) {
            SphereShape* sphereA = static_cast<SphereShape*>(pair.bodyA->collisionShape.get());
            SphereShape* sphereB = static_cast<SphereShape*>(pair.bodyB->collisionShape.get());

            sphereA->center = pair.bodyA->currentState.position;
            sphereB->center = pair.bodyB->currentState.position;

            Vector3 delta = sphereB->center - sphereA->center;
            float distance = delta.magnitude();
            float minDistance = sphereA->radius + sphereB->radius;

            if (distance < minDistance && distance > 0.0001f) {  // ป้องกัน divide by zero
                ContactPoint contact;
                contact.normal = delta.normalized();
                contact.penetration = minDistance - distance;
                contact.position = sphereA->center + contact.normal * sphereA->radius;
                pair.contacts.push_back(contact);
                return true;
            }
            return false;
        }

        // Sphere vs Box (top plane)
        RigidBody* sphereBody = nullptr;
        RigidBody* boxBody = nullptr;
        if (typeA == SHAPE_SPHERE && typeB == SHAPE_BOX) {
            sphereBody = pair.bodyA;
            boxBody = pair.bodyB;
        } else if (typeA == SHAPE_BOX && typeB == SHAPE_SPHERE) {
            sphereBody = pair.bodyB;
            boxBody = pair.bodyA;
        }

        if (sphereBody && boxBody) {
            SphereShape* sphere = static_cast<SphereShape*>(sphereBody->collisionShape.get());
            BoxShape* box = static_cast<BoxShape*>(boxBody->collisionShape.get());

            sphere->center = sphereBody->currentState.position;
            box->center = boxBody->currentState.position;

            float groundTopY = box->center.y + box->halfExtents.y;
            float sphereBottomY = sphere->center.y - sphere->radius;

            if (sphereBottomY < groundTopY) {
                float dx = std::abs(sphere->center.x - box->center.x);
                float dz = std::abs(sphere->center.z - box->center.z);
                if (dx < box->halfExtents.x + sphere->radius &&
                    dz < box->halfExtents.z + sphere->radius) {

                    ContactPoint cp;
                    cp.normal = Vector3(0, 1, 0);
                    cp.penetration = groundTopY - sphereBottomY;
                    cp.position = Vector3(sphere->center.x, groundTopY, sphere->center.z);
                    pair.contacts.push_back(cp);
                    return true;
                }
            }
        }

        return false;
    }
};

class Constraint {
public:
    virtual void preSolve(float dt) = 0;
    virtual void solve() = 0;
    virtual ~Constraint() = default;
};

class ContactConstraint : public Constraint {
    RigidBody* bodyA;
    RigidBody* bodyB;
    ContactPoint contact;

public:
    ContactConstraint(RigidBody* a, RigidBody* b, const ContactPoint& cp)
        : bodyA(a), bodyB(b), contact(cp) {}

    void preSolve(float dt) override {}

    void solve() override {
        Vector3 rv = bodyB->currentState.linearVelocity - bodyA->currentState.linearVelocity;
        float velAlongNormal = rv.dot(contact.normal);

        if (velAlongNormal > 0) return;

        float e = std::min(bodyA->restitution, bodyB->restitution);
        float mu = std::min(bodyA->friction, bodyB->friction);  // friction coeff

        float invMassA = (bodyA->mass > 0.0f) ? 1.0f / bodyA->mass : 0.0f;
        float invMassB = (bodyB->mass > 0.0f) ? 1.0f / bodyB->mass : 0.0f;
        float invMassSum = invMassA + invMassB;

        if (invMassSum < 1e-6f) return;

        // Normal impulse
        float jn = -(1.0f + e) * velAlongNormal / invMassSum;
        jn = std::max(jn, 0.0f);

        Vector3 normalImpulse = contact.normal * jn;

        bodyA->currentState.linearVelocity -= normalImpulse * invMassA;
        bodyB->currentState.linearVelocity += normalImpulse * invMassB;

        // Friction (tangential) impulse
        Vector3 tangent = rv - contact.normal * velAlongNormal;
        float tangentMag = tangent.magnitude();
        if (tangentMag > 1e-6f) {
            tangent = tangent.normalized();

            float jt_max = mu * jn;  // Coulomb limit
            float jt = -tangent.dot(rv) / invMassSum;
            jt = std::max(-jt_max, std::min(jt_max, jt));

            Vector3 frictionImpulse = tangent * jt;

            bodyA->currentState.linearVelocity -= frictionImpulse * invMassA;
            bodyB->currentState.linearVelocity += frictionImpulse * invMassB;
        }

        // Position correction
        const float percent = 0.2f;
        const float slop = 0.01f;
        float correctionAmount = std::max(contact.penetration - slop, 0.0f) * percent / invMassSum;
        Vector3 correction = contact.normal * correctionAmount;

        bodyA->currentState.position -= correction * invMassA;
        bodyB->currentState.position += correction * invMassB;
    }
};

class PhysicsWorld {
    DefaultAllocator defaultAlloc;
    
public:
    PhysicsWorld(AsterAllocator* alloc = nullptr) 
        : allocator(alloc ? alloc : &defaultAlloc) {}
    
    ~PhysicsWorld() {
        for (auto* body : bodies) {
            body->~RigidBody();
            allocator->deallocate(body);
        }
        for (auto* constraint : constraints) {
            constraint->~Constraint();
            allocator->deallocate(constraint);
        }
    }
    
    void stepSimulation(float dt) {      
        integrate(dt);
        detectCollisions();
        solveConstraints(dt);
    }
    
    RigidBody* createRigidBody() {
        void* memory = allocator->allocate(sizeof(RigidBody), alignof(RigidBody));
        RigidBody* body = new (memory) RigidBody();
        bodies.push_back(body);
        return body;
    }

    void addGlobalForce(const Vector3& force) {
        globalForce = force;
    }

private:
    void integrate(float dt) {
        for (RigidBody* body : bodies) {
            if (body->mass <= 0) continue;
            
            body->previousState = body->currentState;
            
            Vector3 acceleration = globalForce * (1.0f / body->mass);
            body->currentState.linearVelocity += acceleration * dt;
            body->currentState.position += body->currentState.linearVelocity * dt;
        }
    }
    
    void detectCollisions() {
        for (auto* c : constraints) {
            c->~Constraint();
            allocator->deallocate(c);
        }
        constraints.clear();
        
        broadphase.update(bodies);
        std::vector<CollisionPair> pairs;
        broadphase.findPotentialPairs(pairs);
        
        for (CollisionPair& pair : pairs) {
            pair.contacts.clear();
            if (narrowphase.computeContacts(pair)) {
                for (const auto& cp : pair.contacts) {
                    void* mem = allocator->allocate(sizeof(ContactConstraint), alignof(ContactConstraint));
                    constraints.push_back(new (mem) ContactConstraint(pair.bodyA, pair.bodyB, cp));
                }
            }
        }
    }
    
    void solveConstraints(float dt) {
        for (Constraint* constraint : constraints) {
            constraint->preSolve(dt);
        }
        
        for (int i = 0; i < 10; ++i) {
            for (Constraint* constraint : constraints) {
                constraint->solve();
            }
        }
    }

    std::vector<RigidBody*> bodies;
    std::vector<Constraint*> constraints;
    SAPBroadphase broadphase;
    Narrowphase narrowphase;
    AsterAllocator* allocator;
    Vector3 globalForce = Vector3(0, -9.8f, 0);
};

int main() {
    DefaultAllocator allocator;
    PhysicsWorld world(&allocator);
    
    RigidBody* ground = world.createRigidBody();
    ground->mass = 0.0f;
    ground->restitution = 0.9f;   // ← ปรับตรงนี้: 1.0f = กระเด้งแรงสุด, 0.0f = ไม่กระเด้ง
    ground->friction = 0.6f;      // ← friction ของพื้น (สูง = ลื่นน้อย)
    ground->collisionShape = std::make_unique<BoxShape>(10.0f, 1.0f, 10.0f);
    ground->currentState.position = Vector3(0, -2, 0);
    
    RigidBody* sphere = world.createRigidBody();
    sphere->mass = 1.0f;
    sphere->restitution = 0.8f;
    sphere->friction = 0.3f;      // ball friction
    sphere->collisionShape = std::make_unique<SphereShape>(1.0f);
    sphere->currentState.position = Vector3(0, 5, 0);
    
    // RigidBody* sphere2 = world.createRigidBody();
    // sphere2->mass = 1.0f;
    // sphere2->restitution = 0.7f;
    // sphere2->friction = 0.4f;
    // sphere2->collisionShape = std::make_unique<SphereShape>(0.8f);
    // sphere2->currentState.position = Vector3(2, 3, 0);
    std::cout << "Starting\n";
    
    for (int i = 0; i < 600; ++i) {  // เพิ่ม frame เพื่อเห็น friction ชัด
        world.stepSimulation(0.016f);
        
        if (i % 20 == 0) {
            Vector3 pos = sphere->currentState.position;
            Vector3 vel = sphere->currentState.linearVelocity;
            std::cout << "Frame " << i << ": Pos (" 
                      << pos.x << ", " << pos.y << ", " << pos.z << ") Vel (" 
                      << vel.x << ", " << vel.y << ", " << vel.z << ")\n";
        }
    }

    std::cout << "Simulation completed!\n";
    return 0;
}