#include <erl_driver.h>
#include <ei.h>
#include <stdio.h>
#include <string.h>
#include<vector>
#include <iostream>
#include <btBulletDynamicsCommon.h>

#include "erlybulletMotionState.hpp"
#include "erlybulletcommands.hpp"

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

struct driver_data {
	ErlDrvPort port;

  btDbvtBroadphase* broadphase;
  btDefaultCollisionConfiguration* collision_configuration;
  btSequentialImpulseConstraintSolver* solver;   // The actual physics solver
  btCollisionDispatcher* dispatcher;
  btDiscreteDynamicsWorld* dynamics_world;   // The world.
  std::vector<erlybullet::motion_state*> motion_states;

  driver_data(ErlDrvPort p) :
  	port(p),
  	broadphase(new btDbvtBroadphase),
  	collision_configuration(new btDefaultCollisionConfiguration),
  	solver(new btSequentialImpulseConstraintSolver)
  {
  	dispatcher = new btCollisionDispatcher(collision_configuration);
  	dynamics_world = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collision_configuration);
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

void step_simulation(driver_data& data, const char* buffer, int size)
{
	std::cout << "Received simulation step" << std::endl;
}

void add_sphere(driver_data& data, const char* buffer, int size)
{
	std::cout << "Received add sphere" << std::endl;
}



//--------------------------------------------------------------------------------
// dispatches the data from erlang to the implementation functions
static void outputv(ErlDrvData drv_data, ErlIOVec *ev)
{
	driver_data *data = reinterpret_cast<driver_data*>(drv_data);
  ErlDrvBinary* buffer = ev->binv[1];
  int command = buffer->orig_bytes[0]; // First byte is the command
  switch(command) {
		case EB_STEP_SIMULATION:  step_simulation(*data,&(buffer->orig_bytes[1]),buffer->orig_size-1); break;
		case EB_ADD_SPHERE:  add_sphere(*data,&(buffer->orig_bytes[1]),buffer->orig_size-1); break;
  }
}


