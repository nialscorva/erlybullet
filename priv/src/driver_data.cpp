#include <erl_driver.h>
#include <ei.h>
#include <stdio.h>
#include <string.h>
#include<map>
#include<vector>
#include <iostream>
#include <btBulletDynamicsCommon.h>

#include "erlybulletMotionState.hpp"
#include "erlybulletcommands.hpp"

#include "driver_data.hpp"

void tick_callback(btDynamicsWorld *world, btScalar timeStep);

std::ostream& operator<<(std::ostream& out, const btVector3& vec)
{
	return out << "(" << vec.x() << "," << vec.y() << "," << vec.z() << ")";
}

#define EXTRACT(Buffer,Size,Var) assert(Size >= sizeof(Var)); memcpy(&Var,Buffer,sizeof(Var)); buffer += sizeof(Var); size += sizeof(Var)


btCollisionShape* decode_shape(unsigned char* &buffer,int &size)
{
	unsigned char shape;
	EXTRACT(buffer,size,shape);
	switch(shape)
	{
	case EB_SPHERE_SHAPE:
		double r;
		EXTRACT(buffer,size,r);
		return new btSphereShape(r);
	default:
		return NULL;
	}
}

const btVector3 decode_vector(unsigned char* &buffer,int &size)
{
	double x,y,z;
	EXTRACT(buffer,size,x);
	EXTRACT(buffer,size,y);
	EXTRACT(buffer,size,z);
	return btVector3(x,y,z);
}


driver_data::driver_data(ErlDrvPort p) :
	port(p),
	broadphase(new btDbvtBroadphase),
	collision_configuration(new btDefaultCollisionConfiguration),
	solver(new btSequentialImpulseConstraintSolver),
	last_run(0)
{
	dispatcher = new btCollisionDispatcher(collision_configuration);
	world = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collision_configuration);
	world->setInternalTickCallback(tick_callback,this);
}


driver_data::~driver_data()
{
	body_map_type::iterator it=bodies.begin();
	for(;it != bodies.end();++it)
	{
		world->removeRigidBody(it->second);
		delete it->second->getCollisionShape();
		delete it->second->getMotionState();
		delete it->second;
	}

	delete world;
	delete dispatcher;
	delete solver;
	delete collision_configuration;
	delete broadphase;
}


// broken, saving reference to a temporary like a noob
template<class INSERT_ITERATOR>
void write_vector(const erlybullet::motion_state::vec& v, INSERT_ITERATOR& t)
{
	*t = ERL_DRV_FLOAT; *t = (ErlDrvTermData)&v.x;
	*t = ERL_DRV_FLOAT; *t = (ErlDrvTermData)&v.y;
	*t = ERL_DRV_FLOAT; *t = (ErlDrvTermData)&v.z;
	*t = ERL_DRV_TUPLE; *t = 3;
}


void driver_data::step_simulation(unsigned char* buffer, int size)
{
	timeval start;
	gettimeofday(&start,NULL);
	uint64_t now=uint64_t(start.tv_sec)*uint64_t(1e6) + uint64_t(start.tv_usec);

	float elapsed=float(now-last_run)/1000.0f;

	// if there is no last run data, step the simulation by 1/60th of a second to get things rolling
	if(last_run==0)
		elapsed = 1.0/30.0;

	world->stepSimulation(elapsed,5);

	body_map_type::iterator it=bodies.begin();
	for(;it != bodies.end();++it)
	{
		erlybullet::motion_state* ms=dynamic_cast<erlybullet::motion_state*>(it->second->getMotionState());
		if(ms->isDirty())
		{

			btTransform wt;
			ms->getWorldTransform(wt);
			erlybullet::motion_state::vec pos(wt.getOrigin());

			std::vector<ErlDrvTermData> terms;
			std::back_insert_iterator<std::vector<ErlDrvTermData> > t = std::back_inserter(terms);

			// term {erlybullet, EntityId, [{location,{x,y,z}},{velocity,{vx,vy,vz}}]}
			*t=ERL_DRV_ATOM;  *t = driver_mk_atom("erlybullet");
		  *t = ERL_DRV_UINT; *t = it->first;
			int list_elements=1;

			*t = ERL_DRV_ATOM; *t = driver_mk_atom("location");
			write_vector(pos,t);
			*t = ERL_DRV_TUPLE; *t = 2;
			list_elements++;

			erlybullet::motion_state::vec velocity(it->second->getLinearVelocity());
			*t = ERL_DRV_ATOM; *t = driver_mk_atom("velocity");
			write_vector(velocity,t);
			*t = ERL_DRV_TUPLE; *t = 2;
			list_elements++;


			// {collision, [{id,{x,y,z}} ... ] }
			int num_collisions=1;
			*t = ERL_DRV_ATOM; *t = driver_mk_atom("collisions");
			erlybullet::motion_state::collision_map::iterator collisionI=ms->begin_collision();
			for(;collisionI != ms->end_collision();++collisionI,++num_collisions)
			{
				*t = ERL_DRV_UINT; *t = collisionI->first;
				write_vector(collisionI->second,t);
				*t = ERL_DRV_TUPLE; *t = 2;
			}
			*t = ERL_DRV_NIL;  // terminator on the list
			*t = ERL_DRV_LIST; *t = num_collisions; // close the list of collisions
			*t = ERL_DRV_TUPLE; *t=2; // close the "collision" pair
			++list_elements;

			// close off the term
			*t = ERL_DRV_NIL;
			*t = ERL_DRV_LIST; *t = list_elements;
			*t = ERL_DRV_TUPLE; *t = 3;

			driver_output_term(port,&terms[0],terms.size());
			ms->reset();
		}
	}
	last_run = now;
}
void driver_data::add_entity(unsigned char* buffer, int size)
{
	// <<ShapeBin/binary,Id:64/native-integer,Mass/float,LocBin/binary,VelocityBin/binary>>
	btCollisionShape* shape=decode_shape(buffer,size);

	uint64_t id;
	EXTRACT(buffer,size,id);

	double mass;
	EXTRACT(buffer,size,mass);

	btVector3 location=decode_vector(buffer,size);
	btVector3 velocity=decode_vector(buffer,size);

	double restitution;
	EXTRACT(buffer,size,restitution);


	btVector3 localInertia(0,0,0);
	shape->calculateLocalInertia(mass,localInertia);

	erlybullet::motion_state* ms = new erlybullet::motion_state(location,id);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,ms,shape,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	body->setLinearVelocity(velocity);
	body->setRestitution(restitution);

	//add the body to the dynamics world
	world->addRigidBody(body);

	body_map_type::iterator it=bodies.find(id);
	if(it != bodies.end())
	{
		world->removeRigidBody(it->second);
		delete it->second->getCollisionShape();
		delete it->second->getMotionState();
		delete it->second;
	}
	bodies.insert(std::make_pair(id,body));
}

void driver_data::remove_entity(unsigned char* buffer, int size)
{
	uint64_t id;
	EXTRACT(buffer,size,id);

	body_map_type::iterator it=bodies.find(id);
	if(it != bodies.end())
	{
		world->removeRigidBody(it->second);
		delete it->second->getCollisionShape();
		delete it->second->getMotionState();
		delete it->second;
		bodies.erase(it);
	}
}

void driver_data::tick(double timeStep)
{
	int numManifolds = world->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold = world->getDispatcher()->getManifoldByIndexInternal(i);
		btRigidBody* obA = static_cast<btRigidBody*>(contactManifold->getBody0());
		btRigidBody* obB = static_cast<btRigidBody*>(contactManifold->getBody1());

		int numContacts = contactManifold->getNumContacts();
		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			if (pt.getDistance()<=0.f)
			{
				const btVector3& ptA = pt.getPositionWorldOnA();
				const btVector3& ptB = pt.getPositionWorldOnB();
				const btVector3& normalOnB = pt.m_normalWorldOnB;

				erlybullet::motion_state* msA=dynamic_cast<erlybullet::motion_state*>(obA->getMotionState());
				erlybullet::motion_state* msB=dynamic_cast<erlybullet::motion_state*>(obB->getMotionState());
				msA->addCollision(msB->get_id(),ptA);
				msB->addCollision(msA->get_id(),ptB);

				std::cout << "Found collision at " << ptA << " and " << ptB << " at distance " << pt.getDistance()
								  << " on " << msA->get_id() << " and " << msB->get_id() << std::endl;
			}
		}
	}

}

void tick_callback(btDynamicsWorld *world, btScalar timeStep)
{
  driver_data *w = static_cast<driver_data*>(world->getWorldUserInfo());
  w->tick(timeStep);
}

