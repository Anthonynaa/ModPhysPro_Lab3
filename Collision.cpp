#include "Collision.h"

#include <iostream>
#include <limits>

bool CheckAABB(const AABB& first, const AABB& second) {
  return first.left < second.left + second.width &&
         first.left + first.width > second.left &&
         first.top < second.top + second.height &&
         first.top + first.height > second.top;
}

SweepResult SweepAABB(const AABB& first, const AABB& second, float velocity_x,
                      float velocity_y, float delta_time) {
  float x_entry;
  float x_exit;
  float y_entry;
  float y_exit;

  if (velocity_x > 0.0f) {
    x_entry = second.left - (first.left + first.width);
    x_exit = (second.left + second.width) - first.left;
  } else if (velocity_x < 0.0f) {
    x_entry = (second.left + second.width) - first.left;
    x_exit = second.left - (first.left + first.width);
  } else {
    bool overlaps_y = first.top < second.top + second.height &&
                      first.top + first.height > second.top;

    if (!overlaps_y) {
      return {false, 0.0f, 0.0f, 0.0f};
    }

    x_entry = -std::numeric_limits<float>::infinity();
    x_exit = std::numeric_limits<float>::infinity();
  }

  if (velocity_y > 0.0f) {
    y_entry = second.top - (first.top + first.height);
    y_exit = (second.top + second.height) - first.top;
  } else if (velocity_y < 0.0f) {
    y_entry = (second.top + second.height) - first.top;
    y_exit = second.top - (first.top + first.height);
  } else {
    bool overlaps_y = first.top < second.top + second.height &&
                      first.top + first.height > second.top;

    if (!overlaps_y) {
      return {false, 0.0f, 0.0f, 0.0f};
    }

    y_entry = -std::numeric_limits<float>::infinity();
    y_exit = std::numeric_limits<float>::infinity();
  }

  if (velocity_x != 0.0f) {
    x_entry /= velocity_x;
    x_exit /= velocity_x;
  }

  if (velocity_y != 0.0f) {
    y_entry /= velocity_y;
    y_exit /= velocity_y;
  }

  float entry_time = (x_entry > y_entry) ? x_entry : y_entry;
  float exit_time = (x_exit < y_exit) ? x_exit : y_exit;

  if (entry_time > exit_time || entry_time < 0.0f || entry_time > delta_time ||
      (x_entry < 0.0f && y_entry < 0.0f)) {
    return {false, 0.0f, 0.0f, 0.0f};
  }

  float normal_x = 0.0f;
  float normal_y = 0.0f;

  if (x_entry > y_entry) {
    normal_x = (velocity_x > 0.0f) ? -1.0f : 1.0f;
  } else {
    normal_y = (velocity_y > 0.0f) ? -1.0f : 1.0f;
  }

  return {true, entry_time, normal_x, normal_y};
}
