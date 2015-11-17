#include "hvmc_collisions.h"
#include "hvmc_physics.h"
#include "math.h"
#include "stdio.h"

#define EPSILON 0.001

template<typename N>
N clamp(N v, N lo, N li)
{
    if (v < lo) return lo;
    if (v > li) return li;
    return v;
}

bool CollideBoxes(RigidBody *a, RigidBody *b, CollisionInfo &info){
    vec2 mina = a->getMinBox();
    vec2 maxa = a->getMaxBox();
    vec2 minb = b->getMinBox();
    vec2 maxb = b->getMaxBox();
    if(minb.x > maxa.x || minb.y > maxa.y || mina.x > maxb.x || mina.y > maxb.y)
      return false;
    
    vec2 ab = b->position-a->position;
    f32 overlapx;
    f32 overlapy;
    if(ab.x > 0){
      //b à droite de a
      overlapx=maxa.x-minb.x;
    }else{
      overlapx=maxb.x-mina.x;
    }
    if(ab.y > 0){
      overlapy=maxa.y-minb.y;
    }else{
      overlapy=maxb.y-mina.y;
    }
    
    if(overlapx<overlapy){
      info.distIterpen=overlapx;
      info.normContact.y=0;
      if(ab.y > 0){
    info.normContact.x=1;
      }else{
    info.normContact.x=-1;
      }
    }else{
      info.distIterpen=overlapy;
      info.normContact.x=0;
      if(ab.x > 0){
    info.normContact.y=1;
      }else{
    info.normContact.y=-1;
      }
    }

    info.rb1 = a;
    info.rb2 = b;
    return true;
}

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

   // fprintf(stderr, "raduisSum: %f \tdistCenter: %f\n",radiusSum, distCenters);
    return(radiusSum > distCenters);
}


bool CollideBoxCircle(RigidBody *a /*Box*/, RigidBody *b, CollisionInfo &info){

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
        // TODO revoir ça avec un - et eplison
        //float epsilon = 0.001;
        if (abs(p.y-max.y) < EPSILON) info.normContact = {0,1};
        else if (abs(p.y-min.y) < EPSILON) info.normContact = {0,-1};
        else if (abs(p.x-max.x) < EPSILON) info.normContact = {1,0};
        else if (abs(p.x-min.x) < EPSILON) info.normContact = {-1,0};
        //else info.normContact = {0,1};

        return true;
    }
    return false;
}

bool CollideCircleBox(RigidBody *a, RigidBody *b/*Box*/, CollisionInfo &info){
    return CollideBoxCircle(b,a,info);
}


bool (*t[NB_RIGID_BODY_TYPES][NB_RIGID_BODY_TYPES])(RigidBody *a, RigidBody *b, CollisionInfo &info);

void initCollide(){
    t[RIGID_BODY_BOX][RIGID_BODY_BOX] =  CollideBoxes;
    t[RIGID_BODY_SPHERE][RIGID_BODY_SPHERE] = CollideCircles;
    t[RIGID_BODY_BOX][RIGID_BODY_SPHERE] = CollideBoxCircle;
    t[RIGID_BODY_SPHERE][RIGID_BODY_BOX] = CollideCircleBox;
}

bool Collide(RigidBody *a, RigidBody *b, CollisionInfo &info)
{
    // mise à jour de info
    return t[a->collider.type][b->collider.type](a,b,info);
}



void CollisionInfo::Solve()
{
    if (rb1->collider.type == RIGID_BODY_BOX && rb2->collider.type == RIGID_BODY_BOX)
        fprintf(stderr, "%f %f\n", normContact.x, normContact.y);
    f32 e = std::min(rb1->e, rb2->e);

    // vrel = (vb + wb*rb) - (va+wa*ra);
    vec2 r1 = rb1->position - ptcontact;
    vec2 r2 = rb2->position - ptcontact;

    vec2 vrel = (rb2->velocity + Cross(rb2->angularVelocity,r2))  - (rb1->velocity + Cross(rb1->angularVelocity,r1));
    //vec2 vrel = rb2->velocity - rb1->velocity;

    if (Dot(vrel, normContact) < 0)
    {
        //auto J = (-(1+e) * Dot(vrel, normContact)) / (rb1->im + rb2->im);
        auto J = (-(1+e) * Dot(vrel, normContact)) /(rb1->im + rb2->im + rb1->iI * Cross(r1, normContact) + rb2->iI * Cross(r2,normContact));
        //fprintf(stderr, "J: %f\n", J);

        vec2 j1 =  -J * normContact;
        vec2 j2 = J * normContact;

        rb1->ApplyImpulse(j1, ptcontact);
        rb2->ApplyImpulse(j2, ptcontact);

    }

}
