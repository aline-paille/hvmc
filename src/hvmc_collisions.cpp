#include "hvmc_collisions.h"
#include "hvmc_physics.h"
#include "math.h"
#include "stdio.h"
#include "hvmc_gjk.h"
#include <iostream>

#define EPSILON 0.001

template<typename N>
N clamp(N v, N lo, N li)
{
    if (v < lo) return lo;
    if (v > li) return li;
    return v;
}

/** Foction qui teste s'il y a collision entre deux une boxes
 * @brief CollideBoxes
 * @param a : box
 * @param b : box
 * @param info : structure contennant l'info d'une Collision
 * @return return true s'il a collision entre a et b
 */
bool CollideBoxes(RigidBody *a, RigidBody *b, CollisionInfo &info){
    vec2 mina = a->getMinBox();
    vec2 maxa = a->getMaxBox();
    vec2 minb = b->getMinBox();
    vec2 maxb = b->getMaxBox();

    if(minb.x > maxa.x || minb.y > maxa.y || mina.x > maxb.x || mina.y > maxb.y)
      return false;

    vec2 ab = b->position - a->position;
    f32 overlapx, overlapy;

    // on détermine les emplacements de chaque box
    if(ab.x > 0){   //b à droite de a
        overlapx = maxa.x - minb.x;
    }
    else{           //b à gauche de a
        overlapx = maxb.x - mina.x;
    }
    if(ab.y > 0){   //b au dessus de a
        overlapy = maxa.y - minb.y;
    }
    else{           //b au dessous de a
        overlapy = maxb.y - mina.y;
    }

    // calculs pour remplir la collisionInfo
    if(overlapx < overlapy){
        info.distIterpen = overlapx;
        info.normContact.y = 0;
        if(ab.y > 0)
        {
            info.normContact.x = 1;
        }
        else{
            info.normContact.x = -1;
        }
    }
    else {
        info.distIterpen = overlapy;
        info.normContact.x = 0;
        if(ab.x > 0){
            info.normContact.y = 1;
        }
        else {
            info.normContact.y = -1;
        }
    }

    info.ptcontact = maxa;//{overlapx, overlapy};
    info.rb1 = a;
    info.rb2 = b;
    return true;
}

/** Foction qui teste s'il y a collision entre deux cercles
 * @brief CollideCircles
 * @param a : cercle
 * @param b : cercle
 * @param info : structure contennant l'info d'une Collision
 * @return true s'il a collision entre a et b
 */
bool CollideCircles(RigidBody *a, RigidBody *b, CollisionInfo &info){
    f32 radiusSum = pow(a->collider.radius  + b->collider.radius, 2);
    f32 distCenters = pow(b->position.x - a->position.x, 2) + pow(b->position.y - a->position.y,2);

    info.normContact.x = (b->position.x - a->position.x) / sqrt(distCenters);
    info.normContact.y = (b->position.y - a->position.y) / sqrt(distCenters);

    info.distIterpen = b->collider.radius + a->collider.radius - sqrt(distCenters);

    info.ptcontact.x = a->position.x + a->collider.radius * info.normContact.x;
    info.ptcontact.y = a->position.y + a->collider.radius * info.normContact.y;

    info.rb1 = a;
    info.rb2 = b;

    return(radiusSum > distCenters);
}

/** Foction qui teste s'il y a collision entre une box a et un cercle b
 * @brief CollideBoxCircle
 * @param a : box
 * @param b : cercle
 * @param info : structure contennant l'info d'une Collision
 * @return true s'il a collision entre a et b
 */
bool CollideBoxCircle(RigidBody *a, RigidBody *b, CollisionInfo &info){
    // point le plus proche appartenant à la boite

    vec2 min = a->getMinBox();
    vec2 max = a->getMaxBox();
    vec2 extent = vec2{(max.x - min.x) / 2, (max.y - min.y) / 2};

    vec2 ab = b->position-a->position;

    vec2 p = vec2{a->position.x + clamp(ab.x, -extent.x, extent.x), a->position.y + clamp(ab.y, -extent.y, extent.y)};

    // || (p-b) ||^2 < r^2
    vec2 pb = p-b->position;
    f32 dist = LengthSquared(pb);
    f32 r = b->collider.radius;
    if (dist < r*r)
    {
        info.rb1 = a;
        info.rb2 = b;
        info.ptcontact = p;
        info.distIterpen = r - dist;

        if (abs(p.y-max.y) < EPSILON) info.normContact = {0,1};
        else if (abs(p.y-min.y) < EPSILON) info.normContact = {0,-1};
        else if (abs(p.x-max.x) < EPSILON) info.normContact = {1,0};
        else if (abs(p.x-min.x) < EPSILON) info.normContact = {-1,0};

        return true;
    }
    return false;
}

/** Foction qui teste s'il y a collision entre deux polygones
 * @brief CollidePolys
 * @param a : polygone
 * @param b : polygone
 * @param info : structure contennant l'info d'une Collision
 * @return return true s'il a collision entre a et b
 */
bool CollidePolys(RigidBody *a, RigidBody *b, CollisionInfo &info){
    vec2 dist = gjk(a->collider.poly, b->collider.poly);
    if(dist.x == 0.f && dist.y == 0.f){
      return true;
    }
    return false;
}


/** Fonction qui teste s'il y a collision entre un cercle a et une box b
 * @brief CollideCircleBox
 * @param a : cerlce
 * @param b : box
 * @param info : structure contennant l'info d'une Collision
 * @return true s'il a collision entre a et b et false sinon
 */
bool CollideCircleBox(RigidBody *a, RigidBody *b, CollisionInfo &info){
    return CollideBoxCircle(b,a,info);
}

bool (*t[NB_RIGID_BODY_TYPES][NB_RIGID_BODY_TYPES])(RigidBody *a, RigidBody *b, CollisionInfo &info);

void initCollide(){
    t[RIGID_BODY_BOX][RIGID_BODY_BOX] =  CollideBoxes;
    t[RIGID_BODY_SPHERE][RIGID_BODY_SPHERE] = CollideCircles;
    t[RIGID_BODY_BOX][RIGID_BODY_SPHERE] = CollideBoxCircle;
    t[RIGID_BODY_SPHERE][RIGID_BODY_BOX] = CollideCircleBox;
    t[RIGID_BODY_POLY][RIGID_BODY_POLY] = CollidePolys;
    }

/** Fonction générique pour tester s'il y a collision entre deux rigidbdy
 * @brief Collide
 * @param a
 * @param b
 * @param info
 * @return
 */
bool Collide(RigidBody *a, RigidBody *b, CollisionInfo &info)
{
    if(a->collider.type == RIGID_BODY_POLY && b->collider.type != RIGID_BODY_POLY)
        return t[RIGID_BODY_BOX][b->collider.type](a,b,info);

    if(a->collider.type != RIGID_BODY_POLY && b->collider.type == RIGID_BODY_POLY)
        return t[a->collider.type][RIGID_BODY_BOX](a,b,info);

    return t[a->collider.type][b->collider.type](a,b,info);
}

/** Fonction de résolution des collisions */
void CollisionInfo::Solve()
{
    f32 e = std::min(rb1->e, rb2->e);   // e: coeff de restitution
    vec2 r1 = -rb1->position + ptcontact;
    vec2 r2 = -rb2->position + ptcontact;
    vec2 vrel = (rb2->velocity + Cross(rb2->angularVelocity,r2))  - (rb1->velocity + Cross(rb1->angularVelocity,r1));

    if (Dot(vrel, normContact) < 0)
    {
        auto J = (-(1+e) * Dot(vrel, normContact)) /(rb1->im + rb2->im + rb1->iI * Cross(r1, normContact) + rb2->iI * Cross(r2,normContact));
        vec2 j1 = -J * normContact;
        vec2 j2 = J * normContact;

        rb1->ApplyImpulse(j1*2, ptcontact);
        rb2->ApplyImpulse(j2*2, ptcontact);
    }
}


void CollisionInfo::CorrectPositions(){
  const f32 seuil = 0.01f;
  const f32 p = 0.5f;
  vec2 correction = ( std::max( distIterpen - seuil, 0.0f ) / ( rb1->im + rb2->im ) ) * p * normContact;
  rb1->position = rb1->position - rb1->im * correction;
  rb2->position += rb2->im * correction;
}
