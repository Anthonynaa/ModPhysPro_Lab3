
#include "RigidBody.h"

RigidBody::RigidBody(float x, float y, float width, float height,
                     float velocity_x, float velocity_y, float mass,
                     bool is_static)
    : x_(x),
      y_(y),
      width_(width),
      height_(height),
      velocity_x_(velocity_x),
      velocity_y_(velocity_y),
      mass_(mass),
      is_static_(is_static) {}

void RigidBody::Update(float delta_time) {
  if (is_static_) {
    return;
  }

  x_ += velocity_x_ * delta_time;
  y_ += velocity_y_ * delta_time;
}

AABB RigidBody::GetBounds() const { return {x_, y_, width_, height_}; }

float RigidBody::GetX() const { return x_; }

float RigidBody::GetY() const { return y_; }

float RigidBody::GetWidth() const { return width_; }

float RigidBody::GetHeight() const { return height_; }

void RigidBody::SetPosition(float x, float y) {
  x_ = x;
  y_ = y;
}

void RigidBody::Stop() {
  velocity_x_ = 0.0f;
  velocity_y_ = 0.0f;
}

float RigidBody::GetVelocityX() const { return velocity_x_; }

float RigidBody::GetVelocityY() const { return velocity_y_; }

void RigidBody::SetVelocity(float velocity_x, float velocity_y) {
  velocity_x_ = velocity_x;
  velocity_y_ = velocity_y;
}
