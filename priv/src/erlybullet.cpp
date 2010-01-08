#include <erl_driver.h>
#include <ei.h>
#include <stdio.h>
#include <string.h>
#include<map>
#include <iostream>
#include <btBulletDynamicsCommon.h>

#include "erlybulletMotionState.hpp"
#include "erlybulletcommands.hpp"


std::ostream& operator<<(std::ostream& out, const btVector3& vec)
{
	return out << "(" << vec.x() << "," << vec.y() << "," << vec.z() << ")";
}

/* Driver interface declarations */
static ErlDrvData start(ErlDrvPort port, char *command);
static void stop(ErlDrvData drv_data);
static void outputv(ErlDrvData handle, ErlIOVec *ev);

static ErlDrvEntry erlybullet_driver_entry = {
    NULL,                        /* init */
    start,
    stop,
    NULL,                        /* output */
    NULL,                        /* ready_input */
    NULL,                        /* ready_output */
    "erlybullet_drv",            /* the name of the driver */
    NULL,                        /* finish */
    NULL,                        /* handle */
    NULL,                        /* control */
    NULL,                        /* timeout */
    outputv,                        /* outputv */
    NULL,                        /* ready_async */
    NULL,                        /* flush */
    NULL,                        /* call */
    NULL,                        /* event */
    ERL_DRV_EXTENDED_MARKER,          /* ERL_DRV_EXTENDED_MARKER */
    ERL_DRV_EXTENDED_MAJOR_VERSION,   /* ERL_DRV_EXTENDED_MAJOR_VERSION */
    ERL_DRV_EXTENDED_MAJOR_VERSION,   /* ERL_DRV_EXTENDED_MINOR_VERSION */
    ERL_DRV_FLAG_USE_PORT_LOCKING     /* ERL_DRV_FLAGs */

};

void tick_callback(const btDynamicsWorld *world, btScalar timeStep);

struct driver_data {
	ErlDrvPort port;
private:
  btDbvtBroadphase* broadphase;
  btDefaultCollisionConfiguration* collision_configuration;
  btSequentialImpulseConstraintSolver* solver;   // The actual physics solver
  btCollisionDispatcher* dispatcher;
  std::map<uint64_t,erlybullet::motion_state*> motion_states;
public:
  btDiscreteDynamicsWorld* dynamics_world;   // The world.
  uint64_t last_run;

  void add_entity(uint64_t id, erlybullet::motion_state* state)
  {
  	std::map<uint64_t,erlybullet::motion_state*>::iterator it=motion_states.find(id);
  	if(it != motion_states.end())
  		delete it->second;
  	motion_states.insert(std::make_pair(id,state));
  }

  driver_data(ErlDrvPort p) :
  	port(p),
  	broadphase(new btDbvtBroadphase),
  	collision_configuration(new btDefaultCollisionConfiguration),
  	solver(new btSequentialImpulseConstraintSolver),
  	last_run(0)
  {
  	dispatcher = new btCollisionDispatcher(collision_configuration);
  	dynamics_world = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collision_configuration);
  }

  void process_motion_states()
  {
  	std::map<uint64_t,erlybullet::motion_state*>::iterator it=motion_states.begin();
  	std::cout << "Checking for dirties\n";
  	for(;it != motion_states.end();++it)
  	{
  		if(it->second->isDirty())
  		{
  			double x=1,y=2,z=3;

  			btTransform wt;
  			it->second->getWorldTransform(wt);
  			btVector3 pos=wt.getOrigin();
  			x=pos.x();
  			y=pos.y();
  			z=pos.z();

  			std::cout << "Sending location data on " << it->first << " with pos " << pos << std::endl;
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
  			it->second->setClean();
  		}
  	}
  }

	~driver_data()
	{
		delete dynamics_world;
		delete dispatcher;
		delete solver;
		delete collision_configuration;
		delete broadphase;
	}

};

//================================================================================
// Driver control functions
//================================================================================
extern "C" {
DRIVER_INIT(erlybullet_drv)
{
	return &erlybullet_driver_entry;
}
}
//--------------------------------------------------------------------------------
// Starts the driver
ErlDrvData start(ErlDrvPort port, char *command)
{
	driver_data* data;
	data = new driver_data(port);

	return reinterpret_cast<ErlDrvData>(data);
}


//--------------------------------------------------------------------------------
// Stops the driver
void stop(ErlDrvData drv_data)
{
	driver_data *data = reinterpret_cast<driver_data*>(drv_data);
	delete data;
}

void step_simulation(driver_data& data, unsigned char* buffer, int size)
{
	timeval start;
	gettimeofday(&start,NULL);
	uint64_t now=uint64_t(start.tv_sec)*uint64_t(1e6) + uint64_t(start.tv_usec);

	float elapsed=float(now-data.last_run)/1000.0f;

	// if there is no last run data, step the simulation by 1/60th of a second to get things rolling
	if(data.last_run==0)
		elapsed = 1.0/30.0;
	std::cout << "Stepping simulation by " << elapsed << std::endl;
	data.dynamics_world->stepSimulation(elapsed,5);

	data.process_motion_states();

	data.last_run = now;
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



void add_shape(driver_data& data, unsigned char* buffer, int size)
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

	erlybullet::motion_state* myMotionState = new erlybullet::motion_state(location);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	body->setLinearVelocity(velocity);
	//add the body to the dynamics world
	data.dynamics_world->addRigidBody(body);
	data.add_entity(id,myMotionState);
}



//--------------------------------------------------------------------------------
// dispatches the data from erlang to the implementation functions
static void outputv(ErlDrvData drv_data, ErlIOVec *ev)
{
	driver_data *data = reinterpret_cast<driver_data*>(drv_data);
  ErlDrvBinary* buffer = ev->binv[1];
  int command = buffer->orig_bytes[0]; // First byte is the command
  switch(command) {
		case EB_STEP_SIMULATION:  step_simulation(*data,(unsigned char*)&(buffer->orig_bytes[1]),buffer->orig_size-1); break;
		case EB_ADD_SHAPE:        add_shape(*data,(unsigned char*)&(buffer->orig_bytes[1]),buffer->orig_size-1); break;
  }
}


void tick_callback(const btDynamicsWorld *world, btScalar timeStep)
{

}

