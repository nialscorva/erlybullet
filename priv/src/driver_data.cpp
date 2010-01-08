#include <erl_driver.h>
#include <ei.h>
#include <stdio.h>
#include <string.h>
#include<map>
#include <iostream>
#include <btBulletDynamicsCommon.h>

#include "erlybulletMotionState.hpp"
#include "erlybulletcommands.hpp"

#include "driver_data.hpp"


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
			double x=1,y=2,z=3;

			btTransform wt;
			ms->getWorldTransform(wt);
			btVector3 pos=wt.getOrigin();
			x=pos.x();
			y=pos.y();
			z=pos.z();

			// term {erlybullet, EntityId, [{location,{x,y,z}}]}
			ErlDrvTermData terms[] = {
					ERL_DRV_ATOM, driver_mk_atom("erlybullet"),
					ERL_DRV_UINT, it->first,
					ERL_DRV_ATOM, driver_mk_atom("location"),
					ERL_DRV_FLOAT, (ErlDrvTermData) &x,
					ERL_DRV_FLOAT, (ErlDrvTermData)&y,
					ERL_DRV_FLOAT, (ErlDrvTermData)&z,
					ERL_DRV_TUPLE,3,
					ERL_DRV_TUPLE,2,
					ERL_DRV_TUPLE, 3
			};
			driver_output_term(port,terms,sizeof(terms)/sizeof(terms[0]));
			ms->setClean();
		}
	}
	last_run = now;
}
void driver_data::add_shape(unsigned char* buffer, int size)
{
	// <<ShapeBin/binary,Id:64/native-integer,Mass/float,LocBin/binary,VelocityBin/binary>>
	btCollisionShape* shape=decode_shape(buffer,size);

	uint64_t id;
	EXTRACT(buffer,size,id);

	double mass;
	EXTRACT(buffer,size,mass);

	btVector3 location=decode_vector(buffer,size);
	btVector3 velocity=decode_vector(buffer,size);
	btVector3 localInertia(0,0,0);
	shape->calculateLocalInertia(mass,localInertia);

	erlybullet::motion_state* ms = new erlybullet::motion_state(location);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,ms,shape,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	body->setLinearVelocity(velocity);

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


void tick_callback(const btDynamicsWorld *world, btScalar timeStep)
{

}

