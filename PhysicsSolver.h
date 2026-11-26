
#ifndef PHYSICS_SOLVER_H
#define PHYSICS_SOLVER_H

#include <vector>

#include "RigidBody.h"

class PhysicsSolver {
 public:
  void UpdateCCD(RigidBody& moving_body, const AABB& obstacle,
                 float delta_time);

  void AddStaticBody(const RigidBody& body);
  void AddOrdinaryBody(const RigidBody& body);
  void AddFastBody(const RigidBody& body);

  const std::vector<RigidBody>& GetStaticBodies() const;
  const std::vector<RigidBody>& GetOrdinaryBodies() const;
  const std::vector<RigidBody>& GetFastBodies() const;

  void UpdateMultiRate(float frame_time, const AABB& ordinary_obstacle,
                       const AABB& fast_obstacle);

 private:
  std::vector<RigidBody> static_bodies_;
  std::vector<RigidBody> ordinary_bodies_;
  std::vector<RigidBody> fast_bodies_;

  float ordinary_accumulator_ = 0.0f;
  float fast_accumulator_ = 0.0f;

  void ResolveStaticCollision(RigidBody& moving_body,
                              const RigidBody& static_body);

  void ResolveBodyCollision(RigidBody& first, RigidBody& second);
};

#endif
