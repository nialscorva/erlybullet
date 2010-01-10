#include <erl_driver.h>
#include <ei.h>
#include <stdio.h>
#include <string.h>

#include "erlybulletcommands.hpp"
#include "driver_data.hpp"

using erlybullet::driver_data;

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

//--------------------------------------------------------------------------------
// dispatches the data from erlang to the implementation functions
static void outputv(ErlDrvData drv_data, ErlIOVec *ev)
{
	driver_data *data = reinterpret_cast<driver_data*>(drv_data);
  ErlDrvBinary* buffer = ev->binv[1];
  int command = buffer->orig_bytes[0]; // First byte is the command
  switch(command) {
		case EB_STEP_SIMULATION:   data->step_simulation((unsigned char*)&(buffer->orig_bytes[1]),buffer->orig_size-1); break;
		case EB_ADD_ENTITY:        data->add_entity(     (unsigned char*)&(buffer->orig_bytes[1]),buffer->orig_size-1); break;
		case EB_REMOVE_ENTITY:     data->remove_entity(  (unsigned char*)&(buffer->orig_bytes[1]),buffer->orig_size-1); break;
  }
}

