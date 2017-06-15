#ifndef RADIX_ENTITIES_PLAYER_HPP
#define RADIX_ENTITIES_PLAYER_HPP

#include <array>

#include <bullet/BulletCollision/CollisionDispatch/btGhostObject.h>
#include <bullet/BulletCollision/CollisionShapes/btConvexShape.h>

#include <radix/Entity.hpp>
#include <radix/physics/KinematicCharacterController.hpp>

namespace radix {
namespace entities {

class Player : public Entity {
public:
  std::shared_ptr<btConvexShape> shape;
  btPairCachingGhostObject *obj;
  KinematicCharacterController *controller;

  Vector3f velocity, headAngle;
  bool flying, noclip, frozen;
  float speed;
  float stepCounter;

  Player(const CreationParams&);
  ~Player();

  void tick(TDelta);

  Quaternion getBaseHeadOrientation() const;
  Quaternion getHeadOrientation() const;
  inline void setHeadOrientation(Quaternion &quaternion) {
    headAngle = quaternion.toAero();
  }
};

/* TODO class ContactPlayerCallback : public btCollisionWorld::ContactResultCallback {
public:
  ContactPlayerCallback(BaseGame &game) : btCollisionWorld::ContactResultCallback(), game(game) { };

  BaseGame &game;

  virtual btScalar addSingleResult(btManifoldPoint& cp,	const btCollisionObjectWrapper* colObj0Wrap,
                                   int partId0, int index0,const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1) {
    Entity* playerEntity = (Entity*) colObj0Wrap->getCollisionObject()->getUserPointer();
    Entity* triggerEntity = (Entity*) colObj1Wrap->getCollisionObject()->getUserPointer();

    if (triggerEntity && playerEntity) {
      if (triggerEntity->hasComponent<Trigger>()) {
        Trigger &trigger = triggerEntity->getComponent<Trigger>();
        trigger.onUpdate(game);

        if (playerEntity->hasComponent<Player>()) {
          Player &player = playerEntity->getComponent<Player>();
          player.trigger = &trigger;
        }
      }
    }
    return 0;
  };
};*/

} /* namespace entities */
} /* namespace radix */

#endif /* RADIX_ENTITIES_PLAYER_HPP */
