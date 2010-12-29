// Copyright 2009 Jason Wagner
// Released under the zlib license.  See LICENSE.txt
#include <erl_driver.h>
#include <ei.h>
#include <stdio.h>
#include <string.h>
#include<map>
#include<vector>
#include <iostream>
#include <btBulletDynamicsCommon.h>
#include <sys/time.h>
#include "motion_state.hpp"
#include "erlybulletcommands.hpp"

#include "driver_data.hpp"


namespace erlybullet {

static void tick_callback(btDynamicsWorld *world, btScalar timeStep);

// convenience for debugging
std::ostream& operator<<(std::ostream& out, const btVector3& vec)
{
	return out << "(" << vec.x() << "," << vec.y() << "," << vec.z() << ")";
}

/// Does a memcpy of the variable out of the buffer, saving a lot of boilerplate code.
#define EXTRACT(Buffer,Size,Var) assert(Size >= sizeof(Var)); memcpy(&Var,Buffer,sizeof(Var)); buffer += sizeof(Var); size += sizeof(Var)

//-------------------------------------------------------------------------------------------------
/// Hides the big switch statement that would uglify the code below.  Simply switches on the shape type, extracts
/// the shape parameters, and returns the collision shape.
inline static btCollisionShape* decode_shape(unsigned char* &buffer,int &size)
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

//-------------------------------------------------------------------------------------------------
/// Extracts a vector from the buffer.  This is done fairly frequently.
inline static const btVector3 decode_vector(unsigned char* &buffer,int &size)
{
	double x,y,z;
	EXTRACT(buffer,size,x);
	EXTRACT(buffer,size,y);
	EXTRACT(buffer,size,z);
	return btVector3(x,y,z);
}


//-------------------------------------------------------------------------------------------------
/// Basic constructor.  Sets up the simulation and saves off the basic necessities.
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

//-------------------------------------------------------------------------------------------------
/// Cleanup
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


//-------------------------------------------------------------------------------------------------
/// Turns a motion_state::vec into a 3-tuple.  This is safe because we know that the vec will live
/// until after the driver_output() call finishes.
template<class INSERT_ITERATOR>
void write_vector(const motion_state::vec& v, INSERT_ITERATOR& t)
{
	*t = ERL_DRV_FLOAT; *t = (ErlDrvTermData)&v.x;
	*t = ERL_DRV_FLOAT; *t = (ErlDrvTermData)&v.y;
	*t = ERL_DRV_FLOAT; *t = (ErlDrvTermData)&v.z;
	*t = ERL_DRV_TUPLE; *t = 3;
}

//-------------------------------------------------------------------------------------------------
/// Advances the simulation by the amount of time since it's last run.  The clock is maintained
/// internally.  It may be worth refactoring to optionally pass in a timestep.
/// TODO make world->stepSimulation() use the asynchronous threadpool
void driver_data::step_simulation(unsigned char* buffer, int size)
{
	timeval start;
	gettimeofday(&start,NULL);
	uint64_t now=uint64_t(start.tv_sec)*uint64_t(1e6) + uint64_t(start.tv_usec);

	double elapsed=double(now-last_run)/1000.0;

	// if there is no last run data, step the simulation by 1/60th of a second to get things rolling
	if(last_run==0)
		elapsed = 1.0/30.0;

	if(size > 0)
	{
		EXTRACT(buffer,size,elapsed);
	}
	world->stepSimulation(elapsed,5);

	// loop over the bodies in the simulation and send them all to the erlang side
	body_map_type::iterator it=bodies.begin();
	for(;it != bodies.end();++it)
	{
		motion_state* ms=dynamic_cast<motion_state*>(it->second->getMotionState());
		if(ms->isDirty())
		{
			btTransform wt;
			ms->getWorldTransform(wt);
			motion_state::vec pos(wt.getOrigin());
			motion_state::vec velocity(it->second->getLinearVelocity());

			std::vector<ErlDrvTermData> terms;
			std::back_insert_iterator<std::vector<ErlDrvTermData> > t = std::back_inserter(terms);

			// term {erlybullet, EntityId, [{location,{x,y,z}},{velocity,{vx,vy,vz}},{collisions,[...]}]}
			*t = ERL_DRV_ATOM;  *t = driver_mk_atom((char*)"erlybullet");
		  *t = ERL_DRV_UINT;  *t = it->first;
			int list_elements=1;

			*t = ERL_DRV_ATOM;  *t = driver_mk_atom((char*)"location");
			write_vector(pos,t);
			*t = ERL_DRV_TUPLE; *t = 2;
			list_elements++;

			*t = ERL_DRV_ATOM;  *t = driver_mk_atom((char*)"velocity");
			write_vector(velocity,t);
			*t = ERL_DRV_TUPLE; *t = 2;
			list_elements++;


			// {collision, [{id,{x,y,z}} ... ] }
			int num_collisions=1;
			*t = ERL_DRV_ATOM; *t = driver_mk_atom((char*)"collisions");
			motion_state::collision_map::iterator collisionI=ms->begin_collision();

			// the motion_state::vec class largely exists to support this loop.  If we used
			// temporary doubles, they would go out of scope before the driver_output_term()
			// call.  converting them to doubles first and storing them that way lets them
			// live until the ms->reset() call, avoiding allocation or corruption.  It's
			// vaguely clever, so I documented it.
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

			// close off the top level term
			*t = ERL_DRV_NIL;
			*t = ERL_DRV_LIST;  *t = list_elements;
			*t = ERL_DRV_TUPLE; *t = 3;

			driver_output_term(port,&terms[0],terms.size());
			ms->reset();
		}
	}
	last_run = now;
}

//-------------------------------------------------------------------------------------------------
/// Adds a rigid body to the simulation.  Currently only handles spheres, but easily extendable.
/// The ID must be unique, in case of duplicate the old ones will be deleted without notification.
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

	motion_state* ms = new motion_state(location,id);
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

//-------------------------------------------------------------------------------------------------
/// Removes an entity by it's ID.  There is no notification that the entity was removed from the
/// calling process.
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

//-------------------------------------------------------------------------------------------------
/// Callback function for the internal simulation tick.  Looks for collisions and saves them for
/// later delivery.  Only the last collision is recorded.  So if A and B bounce off each other
/// 5 times in one step, the last one is sent up to Erlang.
/// MUST NOT interact with the Erlang port in any way, since the simulation
/// step call is going to be made async at some point.
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

				motion_state* msA=dynamic_cast<motion_state*>(obA->getMotionState());
				motion_state* msB=dynamic_cast<motion_state*>(obB->getMotionState());
				msA->addCollision(msB->get_id(),ptA);
				msB->addCollision(msA->get_id(),ptB);
			}
		}
	}

}

void driver_data::apply_impulse(unsigned char* buffer, int size)
{
	uint64_t id;
	EXTRACT(buffer,size,id);
	btVector3 impulse=decode_vector(buffer,size);
	btRigidBody* rb=bodies[id];

	if(rb)
	{
		rb->applyCentralImpulse(impulse);
	}
}



//-------------------------------------------------------------------------------------------------
/// Trampoline function for the tick callback.
static void tick_callback(btDynamicsWorld *world, btScalar timeStep)
{
  driver_data *w = static_cast<driver_data*>(world->getWorldUserInfo());
  w->tick(timeStep);
}

} //namespace erlybullet
