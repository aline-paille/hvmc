#ifndef HVMC_PHYSICS_H
#define HVMC_PHYSICS_H

#include <vector>
#include "hvmc_math.h"
#include "hvmc_collisions.h"
#include "hvmc_gjk.h"

enum RigidBodyType
{
    RIGID_BODY_SPHERE,
    RIGID_BODY_BOX,
    RIGID_BODY_POLY,
    NB_RIGID_BODY_TYPES,
};

struct Collider
{
    Collider() {}
    RigidBodyType type;
    polygon poly;
    union
    {
        f32 radius; // Sphere
        vec2 dims; // Box
    };
};

struct RigidBody
{
    RigidBody() {}
    ~RigidBody() {}
    
    void Update( f32 dt );
    void IntegrateForces(f32 dt);
    void IntegrateVelocities(f32 dt);
    void ApplyForce( vec2 const& force );
    void ApplyForceAng( f32 m );
    void ApplyForce( vec2 const& f ,vec2 const& r);
    void ApplyImpulse( vec2 const& impulse, vec2 const& contactVector );
    
    void SetKinematic();
    void SetGravityMode( int mode );
    
    vec2 getMinBox()const;
    vec2 getMaxBox()const;

    f32 I = 0.f;    // inertia
    f32 iI = 0.f;   // inverse inertia
    f32 m = 0.f;    // mass
    f32 im = 0.f;   // inverse mass
    f32 e = 0.f;    // coefficient de restitution
    
    int gravityMode = 1;

    vec2 forces;
    vec2 position;
    vec2 velocity;

    f32 torque = 0.f;
    f32 rotation = 0.f; 
    f32 angularVelocity = 0.f;

    Collider collider;
};

struct PhysicsSystem
{
    bool Init();
    void Cleanup();
    
    void Update( f32 dt );
    
    RigidBody* AddSphere( vec2 const& pos, f32 radius );
    RigidBody* AddBox( vec2 const& pos, vec2 const& dims );
    RigidBody* AddPolygon( vec2 const& pos, vec2 const& dims );
    RigidBody* AddWall( vec2 const& pos, vec2 const& dims );

    std::vector<RigidBody*> rigidBodies;
    std::vector<CollisionInfo> collisions;
    vec2 gravity;
};

#endif

