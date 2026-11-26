
#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include "Collision.h"

class RigidBody {
 public:
  RigidBody(float x, float y, float width, float height, float velocity_x,
            float velocity_y, float mass, bool is_static = false);

  void Update(float delta_time);

  AABB GetBounds() const;

  float GetX() const;
  float GetY() const;

  float GetWidth() const;
  float GetHeight() const;

  void SetPosition(float x, float y);
  void Stop();

  float GetVelocityX() const;
  float GetVelocityY() const;

  void SetVelocity(float velocity_x, float velocity_y);

 private:
  float x_;
  float y_;

  float width_;
  float height_;

  float velocity_x_;
  float velocity_y_;

  float mass_;
  bool is_static_;
};

#endif
