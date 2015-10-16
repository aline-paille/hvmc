#include "hvmc_collisions.h"

bool collideBoxes(const RigidBody &a, const RigidBody &b, CollisionInfo &info){
    return false;
}

bool collideCircles(const RigidBody &a, const RigidBody &b, CollisionInfo &info){

    return false;
}

bool collideCircleBox(const RigidBody &a, const RigidBody &b, CollisionInfo &info){

    return false;
}


bool Collide(const RigidBody &a,const RigidBody &b, CollisionInfo &info)
{
    // mise à jour de info
    if(a.collider.type == 0 && b.collider.type == 0 )
         return collideCircles(a,b,info);
    if(a.collider.type == 1 && b.collider.type == 1 )
         return collideBoxes(a,b,info);
    if(a.collider.type == 1 && b.collider.type == 0 || a.collider.type == 0 && b.collider.type == 1)
         return collideCircleBoxe(a,b,info);
    return false;
}
