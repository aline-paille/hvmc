#include "hvmc_collisions.h"
#include "hvmc_physics.h"

bool CollideBoxes(const RigidBody &a, const RigidBody &b, CollisionInfo &info){
    return false;
}

bool CollideCircles(const RigidBody &a, const RigidBody &b, CollisionInfo &info){
    f32 raduisSum = pow(a.collider.radius  + b.collider.radius, 2);
    f32 distCenters = pow(a.position.x - b.position.x, 2) + pow(a.position.y + b.position.y,2);
    if(raduisSum > distCenters)
        return true;
    return false;
}


bool CollideBoxCircle(const RigidBody &a /*Box*/, const RigidBody &b, CollisionInfo &info){
    vec2 extent;
    extent.x = a.position
    return false;
}

bool CollideCircleBox(const RigidBody &a, const RigidBody &b/*Box*/, CollisionInfo &info){
    vec2 extent;
    extent.x =
    return false;
}


bool Collide(const RigidBody &a,const RigidBody &b, CollisionInfo &info)
{
    // mise à jour de info
    if(a.collider.type == 0 && b.collider.type == 0 )
         return CollideCircles(a,b,info);
    if(a.collider.type == 1 && b.collider.type == 1 )
         return CollideBoxes(a,b,info);
    if (a.collider.type == 1 && b.collider.type == 0)
        return CollideBoxCircle(a,b,info);

    if (a.collider.type == 0 && b.collider.type == 1)
         return CollideCircleBox(a,b,info);
    return false;
}
