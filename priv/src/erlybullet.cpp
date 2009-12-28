#include <erl_driver.h>
#include <ei.h>
#include <stdio.h>
#include <string.h>


/* Driver interface declarations */
static ErlDrvData start(ErlDrvPort port, char *command);
static void stop(ErlDrvData drv_data);

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
    NULL,                        /* output */
    NULL,                        /* ready_async */
    NULL,                        /* flush */
    NULL,                        /* call */
    NULL                         /* event */
};

struct driver_data {
	char foo;
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
	data = reinterpret_cast<driver_data*>(driver_alloc(sizeof(driver_data)));
	return reinterpret_cast<ErlDrvData>(data);
}

//--------------------------------------------------------------------------------
// Stops the driver
void stop(ErlDrvData drv_data)
{
	driver_data *data = reinterpret_cast<driver_data*>(drv_data);

}
