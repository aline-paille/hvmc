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
};

void initCollide();

bool Collide(const RigidBody &a, const RigidBody &b, CollisionInfo &info);
// faire un tableau 2D de fonctions !

#endif

