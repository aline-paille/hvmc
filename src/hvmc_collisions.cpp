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

bool CollideBoxes(RigidBody *a, RigidBody *b, CollisionInfo &info){
    vec2 mina = a->getMinBox();
    vec2 maxa = a->getMaxBox();
    vec2 minb = b->getMinBox();
    vec2 maxb = b->getMaxBox();
    return (((maxa.x >= minb.x && maxa.x <= maxb.x) || (mina.x >= minb.x && mina.x <= maxb.x)) &&
	((maxa.y >= minb.y && maxa.y <= maxb.y) || (mina.y >= minb.y && mina.y <= maxb.y)));
}

bool CollideCircles(RigidBody *a, RigidBody *b, CollisionInfo &info){
    f32 raduisSum = pow(a->collider.radius  + b->collider.radius, 2);
    f32 distCenters = pow(b->position.x - a->position.x, 2) + pow(b->position.y - a->position.y,2);

    info.normContact.x = (b->position.x - a->position.x) / sqrt(distCenters);
    info.normContact.y = (b->position.y - a->position.y) / sqrt(distCenters);

    info.distIterpen = b->collider.radius + a->collider.radius - sqrt(distCenters);

    info.ptcontact.x = a->position.x + a->collider.radius * info.normContact.x;
    info.ptcontact.y = a->position.y + a->collider.radius * info.normContact.y;

    return(raduisSum > distCenters);
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
    f32 dist = pow(pb.x,2) + pow(pb.y,2);
    f32 r = b->collider.radius;
    if (dist < pow(r,2))
    {
        info.rb1 = a;
        info.rb2 = b;
        info.ptcontact = p;
        info.distIterpen = r - dist;
        return true;
    }
    return false;
}

bool CollideCircleBox(RigidBody *a, RigidBody *b/*Box*/, CollisionInfo &info){
    /*vec2 min = b.getMinBox();
    vec2 max = b.getMaxBox();
    vec2 extent = vec2{max.x - min.x, max.y - min.y};

    vec2 ab = a.position-b.position;

    vec2 p = vec2{clamp(ab.x, -extent.x, extent.x),clamp(ab.y, -extent.y, extent.y)};

    // || p-b^2 || < r^2
    vec2 pb = p-a.position;
    f32 dist = pow(pb.x,2) + pow(pb.y,2);
    f32 r = a.collider.radius;
    return  dist < pow(r,2);*/
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
