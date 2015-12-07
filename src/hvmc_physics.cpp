#include "hvmc_physics.h"
#include <iostream>

void RigidBody::Update( f32 dt )
{
  vec2 a=im * forces;
  velocity+=dt*a;
  position+=dt*velocity;
  
  std::cout << "jrthuu\n";
}

void RigidBody::ApplyForce( vec2 const& f )
{
  forces+=f;
}

void RigidBody::ApplyForceAng( f32 m )
{
  torque+=m;
}

void RigidBody::ApplyForce( vec2 const& f ,vec2 const& r)
{
  forces+=f;
  torque+=Cross(r,f);
}

void RigidBody::IntegrateForces(f32 dt){
    if (m == 0) return;

    vec2 a = im * forces;
    velocity += dt * a;

    f32 theta = im * torque;
    angularVelocity += theta * dt;
}

void RigidBody::IntegrateVelocities(f32 dt){
    if (m == 0) return;
    
    position += dt * velocity;
    if(collider.type == RIGID_BODY_POLY){
      for (unsigned int i=0; i< collider.poly.pts.size(); i++){
	collider.poly.pts[i]+=dt*velocity;
      }
    }
    rotation += dt * angularVelocity;
}
void RigidBody::ApplyImpulse( vec2 const& impulse, vec2 const& contactVector )
{
  if (m==0) return;
  velocity += impulse * im;

  if(this->collider.type != RIGID_BODY_BOX)
    angularVelocity = Cross(contactVector,impulse ) * iI;
}

void RigidBody::SetKinematic()
{
    I = iI = m = im = 0.f;
}

vec2 RigidBody::getMinBox()const
{
    Collider c = this->collider;
    if (c.type == RIGID_BODY_BOX)
    {
        vec2 min;
        min.x = position.x - (c.dims.x/2);
        min.y = position.y - (c.dims.y/2);
        return min;
    }
    return vec2{-1,-1};
}

vec2 RigidBody::getMaxBox() const
{
    Collider c = this->collider;
    if (c.type == RIGID_BODY_BOX)
    {
        vec2 max;
        max.x = position.x + (c.dims.x/2);
        max.y = position.y + (c.dims.y/2);
        return max;
    }
    return vec2{-1,-1};
}

bool PhysicsSystem::Init()
{
    gravity = vec2{ 0.f, -9.81f };

    return true;
}

void PhysicsSystem::Cleanup()
{
    rigidBodies.clear();
}

RigidBody* PhysicsSystem::AddSphere( vec2 const& pos, f32 radius )
{
    RigidBody* body = new RigidBody;
    
    body->forces = { 0.f, 0.f };
    body->im = 1.f; // 1 kg
    body->iI = 1.f;
    body->I = 1.f;
    body->m = 1.f;
    body->e = 0.1; // effet boules de billard
    body->position = pos;
    body->velocity = { 0.f, 0.f };

    body->collider.type = RIGID_BODY_SPHERE;
    body->collider.radius = radius;

    rigidBodies.push_back( body );
    return body;
}

RigidBody* PhysicsSystem::AddBox( vec2 const& pos, vec2 const& dims )
{
    RigidBody* body = new RigidBody; 
    
    body->forces = { 0.f, 0.f };
    body->im = 1.f; // 1 kg
    body->iI = 1.f;
    body->I = 1.f;
    body->m = 1.f;
    body->e = 0.1; // peu élastique
    body->position = pos;
    body->velocity = { 0.f, 0.f };
    
    body->collider.type = RIGID_BODY_BOX;
    body->collider.dims = dims;

    rigidBodies.push_back( body );
    return body;
}

RigidBody* PhysicsSystem::AddPolygon( vec2 const& pos, vec2 const& dims )
{
    RigidBody* body = new RigidBody; 
    
    /*body->forces = { 0.f, 0.f };
    body->im = 1.f; // 1 kg
    body->iI = 1.f;
    body->I = 1.f;
    body->m = 1.f;
    body->e = 0.1; // peu élastique
    vec2 pos2 = {12.5/5.f,11.f/5.f};
    body->position = pos2;
    body->velocity = { 0.f, 0.f };
    
    body->collider.type = RIGID_BODY_POLY;
    vec2 a = {1.f,0.f};
    body->collider.poly.pts.push_back(pos+a);
    a = {0.f,3.f};
    body->collider.poly.pts.push_back(pos+a);
    a = {2.5,5.f};
    body->collider.poly.pts.push_back(pos+a);
    a = {5.f,3.f};
    body->collider.poly.pts.push_back(pos+a);
    a = {4.f,0.f};
    body->collider.poly.pts.push_back(pos+a);
    cout << "okkkkkkkkkkk\n";
    rigidBodies.push_back( body );
    return body;*/
    body->forces = { 0.f, 0.f };
    body->im = 1.f; // 1 kg
    body->iI = 1.f;
    body->I = 1.f;
    body->m = 1.f;
    body->e = 0.1; // peu élastique
    body->position = pos;
    body->velocity = { 0.f, 0.f };

    body->collider.type = RIGID_BODY_POLY;//RIGID_BODY_BOX;
    body->collider.dims = dims;

    rigidBodies.push_back( body );
    return body;
}

RigidBody* PhysicsSystem::AddWall( vec2 const& pos, vec2 const& dims )
{
    RigidBody* body = new RigidBody;

    body->im = 0.f;
    body->velocity = { 0.f, 0.f };
    body->angularVelocity = 0.f;
    body->position = pos;

    body->collider.type = RIGID_BODY_BOX;
    body->collider.dims = dims;

    rigidBodies.push_back( body );
    return body;
}

void PhysicsSystem::Update( f32 dt )
{    
    // Add gravity
    for (auto & rb: rigidBodies){
        if (rb->m != 0) {
            rb->ApplyForce(rb->m * gravity);
        }
    }

    // Generate contact info
    /*u32 count = rigidBodies.size();
    for ( u32 i = 0; i < count - 1; ++i )
    {
        for ( u32 j = i + 1; j < count; ++i )
        {
            RigidBody* a = rigidBodies[i];
            RigidBody* b = rigidBodies[j];*/
    for (auto &a: rigidBodies)
    {
        for (auto &b: rigidBodies)
        {
            if (a!=b )
            {
                if (a->m != 0 || b->m !=0)
                {
                    if(a->collider.type == RIGID_BODY_POLY || b->collider.type == RIGID_BODY_POLY)
                        cout << "avant collide\n";
                    CollisionInfo info;
                    if (Collide(a, b, info))
                    {
                        if(a->collider.type == RIGID_BODY_POLY || b->collider.type == RIGID_BODY_POLY)
                            cout << "collide\n";
                        if(a->collider.type == RIGID_BODY_POLY || b->collider.type == RIGID_BODY_POLY){
                            cout << "polygon";
                            a->SetKinematic();
                            b->SetKinematic();
                        }else{
                            //std::cout << "Collision détectée: " ;
                            (a->collider.type == RIGID_BODY_SPHERE) ? (std::cout<< "cercle "):(std::cout<< "box ");
                            (b->collider.type == RIGID_BODY_SPHERE) ? (std::cout<< "cercle" << std::endl):(std::cout<< "box" << std::endl);

                            collisions.push_back(info);
                        }
                    }
                }
            }
        }
    }


    // Integrate forces
    for (auto &rb: rigidBodies)
        if (rb->m != 0)
            rb->IntegrateForces(dt);

    // Solve collision
    for (auto &col: collisions)
    {
        col.Solve();
    }

    //Integrate velocities
    for (auto &rb: rigidBodies)
        if (rb->m != 0)
            rb->IntegrateVelocities(dt);

    // Position correction

    // clear forces
    for(auto &rb: rigidBodies)
    {
        rb->forces = {0.0, 0.0};
        rb->torque = 0.0;
    }

    collisions.clear();

}

