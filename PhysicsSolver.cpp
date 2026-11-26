#include "PhysicsSolver.h"

#include <algorithm>
#include <iostream>

#include "Collision.h"
#include "SimulationConfig.h"

void PhysicsSolver::UpdateCCD(RigidBody& moving_body, const AABB& obstacle,
                              float delta_time) {
  AABB bounds = moving_body.GetBounds();

  SweepResult result = SweepAABB(bounds, obstacle, moving_body.GetVelocityX(),
                                 moving_body.GetVelocityY(), delta_time);

  if (!result.has_collision) {
    moving_body.Update(delta_time);
    return;
  }

  float new_x = moving_body.GetX() + moving_body.GetVelocityX() * result.time;

  float new_y = moving_body.GetY() + moving_body.GetVelocityY() * result.time;

  moving_body.SetPosition(new_x, new_y);
  moving_body.Stop();
}

void PhysicsSolver::AddStaticBody(const RigidBody& body) {
  static_bodies_.push_back(body);
}

void PhysicsSolver::AddOrdinaryBody(const RigidBody& body) {
  ordinary_bodies_.push_back(body);
}

void PhysicsSolver::AddFastBody(const RigidBody& body) {
  fast_bodies_.push_back(body);
}

const std::vector<RigidBody>& PhysicsSolver::GetStaticBodies() const {
  return static_bodies_;
}

const std::vector<RigidBody>& PhysicsSolver::GetOrdinaryBodies() const {
  return ordinary_bodies_;
}

const std::vector<RigidBody>& PhysicsSolver::GetFastBodies() const {
  return fast_bodies_;
}

void PhysicsSolver::UpdateMultiRate(float frame_time,
                                    const AABB& ordinary_obstacle,
                                    const AABB& fast_obstacle) {
  const float ordinary_delta_time = 1.0f / SimulationConfig::kOrdinaryFrequency;

  const float fast_delta_time = 1.0f / SimulationConfig::kFastFrequency;

  ordinary_accumulator_ += frame_time;
  fast_accumulator_ += frame_time;

  while (ordinary_accumulator_ >= ordinary_delta_time) {
    for (std::size_t i = 0; i < ordinary_bodies_.size(); ++i) {
      RigidBody& body = ordinary_bodies_[i];

      body.Update(ordinary_delta_time);

      if (i == 0) {
        if (CheckAABB(body.GetBounds(), ordinary_obstacle)) {
          body.SetPosition(ordinary_obstacle.left - body.GetWidth(),
                           body.GetY());

          body.Stop();
        }
      }
    }

    for (std::size_t i = 1; i < ordinary_bodies_.size(); ++i) {
      for (const RigidBody& obstacle : static_bodies_) {
        ResolveStaticCollision(ordinary_bodies_[i], obstacle);
      }
    }

    for (std::size_t i = 1; i < ordinary_bodies_.size(); ++i) {
      for (std::size_t j = i + 1; j < ordinary_bodies_.size(); ++j) {
        ResolveBodyCollision(ordinary_bodies_[i], ordinary_bodies_[j]);
      }
    }

    ordinary_accumulator_ -= ordinary_delta_time;
  }

  while (fast_accumulator_ >= fast_delta_time) {
    for (std::size_t i = 0; i < fast_bodies_.size(); ++i) {
      RigidBody& body = fast_bodies_[i];

      body.Update(fast_delta_time);

      if (i == 0) {
        if (CheckAABB(body.GetBounds(), fast_obstacle)) {
          body.SetPosition(fast_obstacle.left - body.GetWidth(), body.GetY());

          body.Stop();
        }
      }
    }

    fast_accumulator_ -= fast_delta_time;
  }
}

void PhysicsSolver::ResolveStaticCollision(RigidBody& moving_body,
                                           const RigidBody& static_body) {
  AABB moving = moving_body.GetBounds();
  AABB obstacle = static_body.GetBounds();

  if (!CheckAABB(moving, obstacle)) {
    return;
  }

  float overlap_x =
      std::min(moving.left + moving.width, obstacle.left + obstacle.width) -
      std::max(moving.left, obstacle.left);

  float overlap_y =
      std::min(moving.top + moving.height, obstacle.top + obstacle.height) -
      std::max(moving.top, obstacle.top);

  float moving_center_x = moving.left + moving.width / 2.0f;
  float moving_center_y = moving.top + moving.height / 2.0f;

  float obstacle_center_x = obstacle.left + obstacle.width / 2.0f;
  float obstacle_center_y = obstacle.top + obstacle.height / 2.0f;

  float velocity_x = moving_body.GetVelocityX();
  float velocity_y = moving_body.GetVelocityY();

  const float restitution = SimulationConfig::kRestitution;

  if (overlap_x < overlap_y) {
    if (moving_center_x < obstacle_center_x) {
      moving_body.SetPosition(moving_body.GetX() - overlap_x,
                              moving_body.GetY());

      if (velocity_x > 0.0f) {
        velocity_x = -velocity_x * restitution;
      }
    } else {
      moving_body.SetPosition(moving_body.GetX() + overlap_x,
                              moving_body.GetY());

      if (velocity_x < 0.0f) {
        velocity_x = -velocity_x * restitution;
      }
    }
  } else {
    if (moving_center_y < obstacle_center_y) {
      moving_body.SetPosition(moving_body.GetX(),
                              moving_body.GetY() - overlap_y);

      if (velocity_y > 0.0f) {
        velocity_y = -velocity_y * restitution;
      }
    } else {
      moving_body.SetPosition(moving_body.GetX(),
                              moving_body.GetY() + overlap_y);

      if (velocity_y < 0.0f) {
        velocity_y = -velocity_y * restitution;
      }
    }
  }

  moving_body.SetVelocity(velocity_x, velocity_y);
}

void PhysicsSolver::ResolveBodyCollision(RigidBody& first, RigidBody& second) {
  AABB first_bounds = first.GetBounds();
  AABB second_bounds = second.GetBounds();

  if (!CheckAABB(first_bounds, second_bounds)) {
    return;
  }

  float overlap_x = std::min(first_bounds.left + first_bounds.width,
                             second_bounds.left + second_bounds.width) -
                    std::max(first_bounds.left, second_bounds.left);

  float overlap_y = std::min(first_bounds.top + first_bounds.height,
                             second_bounds.top + second_bounds.height) -
                    std::max(first_bounds.top, second_bounds.top);

  float first_center_x = first_bounds.left + first_bounds.width / 2.0f;

  float first_center_y = first_bounds.top + first_bounds.height / 2.0f;

  float second_center_x = second_bounds.left + second_bounds.width / 2.0f;

  float second_center_y = second_bounds.top + second_bounds.height / 2.0f;

  if (overlap_x < overlap_y) {
    float separation = overlap_x / 2.0f;

    if (first_center_x < second_center_x) {
      first.SetPosition(first.GetX() - separation, first.GetY());
      second.SetPosition(second.GetX() + separation, second.GetY());
    } else {
      first.SetPosition(first.GetX() + separation, first.GetY());
      second.SetPosition(second.GetX() - separation, second.GetY());
    }

    float first_velocity = first.GetVelocityX();
    float second_velocity = second.GetVelocityX();

    if ((first_center_x < second_center_x &&
         first_velocity > second_velocity) ||
        (first_center_x > second_center_x &&
         first_velocity < second_velocity)) {
      const float restitution = SimulationConfig::kRestitution;

      float new_first_velocity = (first_velocity * (1.0f - restitution) +
                                  second_velocity * (1.0f + restitution)) /
                                 2.0f;

      float new_second_velocity = (second_velocity * (1.0f - restitution) +
                                   first_velocity * (1.0f + restitution)) /
                                  2.0f;

      first.SetVelocity(new_first_velocity, first.GetVelocityY());
      second.SetVelocity(new_second_velocity, second.GetVelocityY());
    }
  } else {
    float separation = overlap_y / 2.0f;

    if (first_center_y < second_center_y) {
      first.SetPosition(first.GetX(), first.GetY() - separation);
      second.SetPosition(second.GetX(), second.GetY() + separation);
    } else {
      first.SetPosition(first.GetX(), first.GetY() + separation);
      second.SetPosition(second.GetX(), second.GetY() - separation);
    }

    float first_velocity = first.GetVelocityY();
    float second_velocity = second.GetVelocityY();

    if ((first_center_y < second_center_y &&
         first_velocity > second_velocity) ||
        (first_center_y > second_center_y &&
         first_velocity < second_velocity)) {
      const float restitution = SimulationConfig::kRestitution;

      float new_first_velocity = (first_velocity * (1.0f - restitution) +
                                  second_velocity * (1.0f + restitution)) /
                                 2.0f;

      float new_second_velocity = (second_velocity * (1.0f - restitution) +
                                   first_velocity * (1.0f + restitution)) /
                                  2.0f;

      first.SetVelocity(first.GetVelocityX(), new_first_velocity);
      second.SetVelocity(second.GetVelocityX(), new_second_velocity);
    }
  }
}
