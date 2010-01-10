#ifndef DRIVER_DATA_HPP_INCLUDED__
#define DRIVER_DATA_HPP_INCLUDED__
#include <erl_driver.h>
#include<map>

class btDbvtBroadphase;
class btDefaultCollisionConfiguration;
class btSequentialImpulseConstraintSolver;
class btCollisionDispatcher;
class btDiscreteDynamicsWorld;
class btRigidBody;

namespace erlybullet { class motion_state; }


struct driver_data {
public:
	void add_entity(unsigned char* buffer, int size);
	void remove_entity(unsigned char* buffer, int size);
	void step_simulation(unsigned char* buffer, int size);
	void tick(double timestep);

  driver_data(ErlDrvPort p);
	~driver_data();
private:
	typedef std::map<uint64_t,btRigidBody*> body_map_type;
	ErlDrvPort                           port;
  btDbvtBroadphase*                    broadphase;
  btDefaultCollisionConfiguration*     collision_configuration;
  btSequentialImpulseConstraintSolver* solver;   // The actual physics solver
  btCollisionDispatcher*               dispatcher;
  body_map_type                        bodies;
  btDiscreteDynamicsWorld*             world;   // The world.
  uint64_t                             last_run;

};
#endif //DRIVER_DATA_HPP_INCLUDED__
