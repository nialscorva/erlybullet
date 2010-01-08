#ifndef ERLYBULLETMOTIONSTATE_H_INCLUDED__
#define ERLYBULLETMOTIONSTATE_H_INCLUDED__

#include<LinearMath/btMotionState.h>
#include<LinearMath/btTransform.h>


namespace erlybullet {
class motion_state : public btMotionState {
public:
  motion_state(const btVector3 &initial_pos) :dirty(false) {
  	mPos1.setIdentity();
  	mPos1.setOrigin(initial_pos);
  }

  virtual ~motion_state() {
  }

  virtual void getWorldTransform(btTransform &worldTrans) const {
      worldTrans = mPos1;
  }

  virtual void setWorldTransform(const btTransform &worldTrans) {
  	dirty=true;
  	mPos1=worldTrans;
  }

  virtual void setClean() { dirty=false; }
  virtual bool isDirty() { return dirty; }

protected:
  btTransform mPos1;
  bool dirty;
  uint64_t id;
};

} // namespace erlybullet

#endif //ERLYBULLETMOTIONSTATE_H_INCLUDED__
