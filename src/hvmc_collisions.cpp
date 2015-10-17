#include "hvmc_collisions.h"
#include "hvmc_physics.h"
#include "math.h"

template<typename N>
N clamp(N v, N lo, N li)
{
    if (v < lo) return lo;
    if (v > li) return li;
    return v;
}

bool CollideBoxes(const RigidBody *a, const RigidBody *b, CollisionInfo &info){
    return false;
}

bool CollideCircles(const RigidBody *a, const RigidBody *b, CollisionInfo &info){
    f32 raduisSum = pow(a->collider.radius  + b->collider.radius, 2);
    f32 distCenters = pow(a->position.x - b->position.x, 2) + pow(a->position.y + b->position.y,2);
    if(raduisSum > distCenters)
        return true;
    return false;
}


bool CollideBoxCircle(const RigidBody *a /*Box*/, const RigidBody *b, CollisionInfo &info){

    // point le plus proche appartenant à la boite

    vec2 min = a->getMinBox();
    vec2 max = a->getMaxBox();
    vec2 extent = vec2{max.x - min.x, max.y - min.y};

    vec2 ab = b->position-a->position;

    vec2 p = vec2{clamp(ab.x, -extent.x, extent.x),clamp(ab.y, -extent.y, extent.y)};

    // || p-b^2 || < r^2
    vec2 pb = p-b->position;
    f32 dist = pow(pb.x,2) + pow(pb.y,2);
    f32 r = b->collider.radius;
    return  dist < pow(r,2);
}

bool CollideCircleBox(const RigidBody *a, const RigidBody *b/*Box*/, CollisionInfo &info){
    //vec2 extent;
    //extent.x =
    return CollideBoxCircle(b,a,info);
}


bool Collide(const RigidBody *a, const RigidBody *b, CollisionInfo &info)
{
    // mise à jour de info
    if(a->collider.type == RIGID_BODY_SPHERE && b->collider.type == RIGID_BODY_SPHERE )
         return CollideCircles(a,b,info);
    if(a->collider.type == RIGID_BODY_BOX && b->collider.type == RIGID_BODY_BOX )
         return CollideBoxes(a,b,info);
    if (a->collider.type == RIGID_BODY_BOX && b->collider.type == RIGID_BODY_SPHERE)
        return CollideBoxCircle(a,b,info);
    if (a->collider.type == RIGID_BODY_SPHERE && b->collider.type == RIGID_BODY_BOX)
         return CollideCircleBox(a,b,info);
    return false;
}
