#ifndef COLLISION_H
#define COLLISION_H

struct AABB {
  float left;
  float top;
  float width;
  float height;
};

struct SweepResult {
  bool has_collision;
  float time;
  float normal_x;
  float normal_y;
};

bool CheckAABB(const AABB& first, const AABB& second);

SweepResult SweepAABB(const AABB& first, const AABB& second, float velocity_x,
                      float velocity_y, float delta_time);

#endif
