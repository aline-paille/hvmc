#ifndef HVMC_COLLISIONS_H
#define HVMC_COLLISIONS_H

#include "hvmc_math.h"

struct RigidBody;

struct CollisionInfo
{
    vec2 normContact; // normale au contact
    f32 distIterpen; // distance d'interpenetration
    RigidBody* rb1;
    RigidBody* rb2;
    vec2 ptcontact;

    void Solve();
};

void initCollide();

bool Collide(RigidBody *a, RigidBody *b, CollisionInfo &info);


#endif

