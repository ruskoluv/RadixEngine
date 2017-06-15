#include <radix/entities/Player.hpp>

namespace radix {
namespace entities {

static const Vector3f PLAYER_SIZE(0.5, 1, 0.5);

static constexpr float RUNNING_SPEED = 0.1f;
static constexpr float JUMP_SPEED = 0.15f;
static constexpr float HURT_VELOCITY = 0.18f;

static const std::array<const std::string, 2> PLAYER_PANTING_SOUND = {{
  "/audio/sfx/character/fem_panting_1.ogg",
  "/audio/sfx/character/fem_panting_2.ogg"
}};

static const std::array<const std::string, 2> PLAYER_JUMP_SOUND = {{
  "/audio/sfx/character/fem_jump_1.ogg",
  "/audio/sfx/character/fem_jump_2.ogg"
}};

static const std::array<const std::string, 2> PLAYER_FALL_SOUND = {{
  "/audio/sfx/character/fem_fall_1.ogg",
  "/audio/sfx/character/fem_fall_2.ogg"
}};

static const std::array<const std::string, 6> PLAYER_FOOT_SOUND = {{
  "/audio/sfx/character/fem_foot_1.ogg",
  "/audio/sfx/character/fem_foot_2.ogg",
  "/audio/sfx/character/fem_foot_3.ogg",
  "/audio/sfx/character/fem_foot_4.ogg",
  "/audio/sfx/character/fem_foot_5.ogg",
  "/audio/sfx/character/fem_foot_6.ogg"
}};

Player::Player(const CreationParams &cp) :
  Entity(cp),
  flying(false),
  noclip(false),
  frozen(false),
  stepCounter(0) {

  entity.getComponent<Transform>().setScale(PLAYER_SIZE);

  obj = new btPairCachingGhostObject;
  Transform &tform = entity.getComponent<Transform>();
  obj->setWorldTransform(btTransform(tform.getOrientation(), tform.getPosition()));
  shape = std::make_shared<btCapsuleShape>(.4, 1);
  obj->setCollisionShape(shape.get());
  obj->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
  obj->setUserPointer(&entity);
  controller = new KinematicCharacterController(obj, shape.get(), 0.35);
}

Player::~Player() {
  delete controller;
  delete obj;
}

void tick(TDelta dtime) {
  (void) dtime;
  Player &player = game.getWorld()->getPlayer().getComponent<Player>();
  if (player.frozen) {
    return;
  }
  InputSource &input = game.getWorld()->input;
  bool movingFwd     = input.isKeyDown(SDL_SCANCODE_W) or input.isKeyDown(SDL_SCANCODE_UP),
    movingBack    = input.isKeyDown(SDL_SCANCODE_S) or input.isKeyDown(SDL_SCANCODE_DOWN),
    strafingLeft  = input.isKeyDown(SDL_SCANCODE_A) or input.isKeyDown(SDL_SCANCODE_LEFT),
    strafingRight = input.isKeyDown(SDL_SCANCODE_D) or input.isKeyDown(SDL_SCANCODE_RIGHT),
    jumping       = input.isKeyDown(SDL_SCANCODE_SPACE) or
                    input.isKeyDown(SDL_SCANCODE_BACKSPACE);
  float rot = player.headAngle.heading;
  Vector3f movement;
  KinematicCharacterController &controller = *game.getWorld()->getPlayer().getComponent<Player>().controller;
  Transform &plrTform = game.getWorld()->getPlayer().getComponent<Transform>();
  plrTform.privSetPosition(game.getWorld()->getPlayer().getComponent<Player>().obj->getWorldTransform().getOrigin());

  if (jumping and controller.canJump()) {
    std::uniform_int_distribution<> dis(0, PLAYER_JUMP_SOUND.size()-1);
    game.getWorld()->getPlayer().getComponent<SoundSource>().playSound(
      Environment::getDataDir() + PLAYER_JUMP_SOUND[dis(Util::Rand)]);
    controller.jump();
  }

  if (movingFwd || movingBack || strafingLeft || strafingRight) {
    if (player.trigger) {
      player.trigger->actionOnMove(game);
    }
  }

  if (movingFwd) {
    movement.x += -sin(rot);
    movement.z += -cos(rot);
  }
  if (movingBack) {
    movement.x += sin(rot);
    movement.z += cos(rot);
  }
  if (strafingLeft) {
    movement.x += -cos(rot);
    movement.z += sin(rot);
  }
  if (strafingRight) {
    movement.x += cos(rot);
    movement.z += -sin(rot);
  }

  movement *= RUNNING_SPEED;
  controller.setWalkDirection(movement);

  if (controller.onGround()) {
    player.stepCounter += std::sqrt(movement.x*movement.x + movement.z*movement.z);

    if (player.stepCounter >= 2.5f) {
      std::uniform_int_distribution<> distribution(0, PLAYER_FOOT_SOUND.size()-1);
      game.getWorld()->getPlayer().getComponent<SoundSource>().playSound(
        Environment::getDataDir() + PLAYER_FOOT_SOUND[distribution(Util::Rand)]);
      player.stepCounter -= 2.5f;
    }
  }
}

Quaternion Player::getBaseHeadOrientation() const {
  return Quaternion().fromAero(headAngle);
}

Quaternion Player::getHeadOrientation() const {
  return Quaternion().fromAero(headAngle);
}

} /* namespace entities */
} /* namespace radix */
