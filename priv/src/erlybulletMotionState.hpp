#ifndef ERLYBULLETMOTIONSTATE_H_INCLUDED__
#define ERLYBULLETMOTIONSTATE_H_INCLUDED__

#include<LinearMath/btMotionState.h>
#include<LinearMath/btTransform.h>


namespace erlybullet {
class motion_state : btMotionState {
  motion_state(const btTransform &initialpos) {
      mPos1 = initialpos;
  }

  virtual ~motion_state() {
  }

  virtual void getWorldTransform(btTransform &worldTrans) const {
      worldTrans = mPos1;
  }

  virtual void setWorldTransform(const btTransform &worldTrans) {
//      if(NULL == mVisibleobj) return; // silently return before we set a node
//      btQuaternion rot = worldTrans.getRotation();
//      mVisibleobj->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
//      btVector3 pos = worldTrans.getOrigin();
//      mVisibleobj->setPosition(pos.x(), pos.y(), pos.z());
  }

protected:
  btTransform mPos1;
};

} // namespace erlybullet

#endif //ERLYBULLETMOTIONSTATE_H_INCLUDED__
