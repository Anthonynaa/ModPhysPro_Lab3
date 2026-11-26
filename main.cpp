
#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>

#include "Collision.h"
#include "PhysicsSolver.h"
#include "SimulationConfig.h"

enum class SimulationMode { Discrete, CCD, MultiRate };

void CreateMultiRateScene(PhysicsSolver& solver) {
  solver.AddStaticBody(
      RigidBody(0.0f, 0.0f, 800.0f, 10.0f, 0.0f, 0.0f, 1.0f, true));

  solver.AddStaticBody(
      RigidBody(0.0f, 590.0f, 800.0f, 10.0f, 0.0f, 0.0f, 1.0f, true));

  solver.AddStaticBody(
      RigidBody(0.0f, 0.0f, 10.0f, 600.0f, 0.0f, 0.0f, 1.0f, true));

  solver.AddStaticBody(
      RigidBody(790.0f, 0.0f, 10.0f, 600.0f, 0.0f, 0.0f, 1.0f, true));

  solver.AddStaticBody(
      RigidBody(700.0f, 300.0f, 24.0f, 24.0f, 0.0f, 0.0f, 1.0f, true));

  solver.AddStaticBody(
      RigidBody(310.0f, 230.0f, 10.0f, 50.0f, 0.0f, 0.0f, 1.0f, true));

  solver.AddStaticBody(
      RigidBody(310.0f, 430.0f, 10.0f, 50.0f, 0.0f, 0.0f, 1.0f, true));

  solver.AddOrdinaryBody(
      RigidBody(80.0f, 250.0f, SimulationConfig::kProjectileSize,
                SimulationConfig::kProjectileSize,
                SimulationConfig::kProjectileSpeed, 0.0f, 1.0f));

  for (int i = 0; i < SimulationConfig::kOrdinaryBodyCount; ++i) {
    float x = 60.0f + (i % 10) * 20.0f;
    float y = 120.0f + (i % 10) * 20.0f;

    float width = 20.0f;
    float height = 20.0f;

    float velocity_x = 30.0f + (i % 10) * 20.0f;
    float velocity_y = 20.0f + (i % 10) * 20.0f;

    solver.AddOrdinaryBody(
        RigidBody(x, y, width, height, velocity_x, velocity_y, 1.0f));
  }

  solver.AddFastBody(RigidBody(80.0f, 450.0f, SimulationConfig::kProjectileSize,
                               SimulationConfig::kProjectileSize,
                               SimulationConfig::kProjectileSpeed, 0.0f, 1.0f));
}

int main() {
  const float fixed_delta_time = 1.0f / SimulationConfig::kOrdinaryFrequency;
  const float simulation_scale = SimulationConfig::kSimulationScale;

  const AABB wall = {310.0f, 170.0f, 10.0f, 160.0f};

  const AABB ordinary_wall = {310.0f, 230.0f, 10.0f, 50.0f};
  const AABB fast_wall = {310.0f, 430.0f, 10.0f, 50.0f};

  RigidBody projectile(80.0f, 250.0f, 8.0f, 8.0f, 1200.0f, 0.0f, 1.0f);

  PhysicsSolver solver;
  SimulationMode mode = SimulationMode::Discrete;

  bool finished = false;
  float accumulator = 0.0f;

  sf::RenderWindow window(sf::VideoMode({800, 600}), "ModPhysPro_Lab3");
  window.setFramerateLimit(60);

  sf::RectangleShape wall_shape({wall.width, wall.height});
  wall_shape.setPosition({wall.left, wall.top});
  wall_shape.setFillColor(sf::Color(120, 120, 120));

  sf::RectangleShape ordinary_wall_shape(
      {ordinary_wall.width, ordinary_wall.height});
  ordinary_wall_shape.setPosition({ordinary_wall.left, ordinary_wall.top});
  ordinary_wall_shape.setFillColor(sf::Color(120, 120, 120));

  sf::RectangleShape fast_wall_shape({fast_wall.width, fast_wall.height});
  fast_wall_shape.setPosition({fast_wall.left, fast_wall.top});
  fast_wall_shape.setFillColor(sf::Color(120, 120, 120));

  sf::RectangleShape projectile_shape(
      {projectile.GetWidth(), projectile.GetHeight()});

  sf::Clock clock;

  while (window.isOpen()) {
    while (const auto event = window.pollEvent()) {
      if (event->is<sf::Event::Closed>()) {
        window.close();
      }

      if (const auto* key = event->getIf<sf::Event::KeyPressed>()) {
        if (key->code == sf::Keyboard::Key::Escape) {
          window.close();
        }

        if (key->code == sf::Keyboard::Key::Num1) {
          mode = SimulationMode::Discrete;
          projectile =
              RigidBody(80.0f, 250.0f, 8.0f, 8.0f, 1200.0f, 0.0f, 1.0f);
          accumulator = 0.0f;
          finished = false;
        }

        if (key->code == sf::Keyboard::Key::Num2) {
          mode = SimulationMode::CCD;
          projectile =
              RigidBody(80.0f, 250.0f, 8.0f, 8.0f, 1200.0f, 0.0f, 1.0f);
          accumulator = 0.0f;
          finished = false;
        }

        if (key->code == sf::Keyboard::Key::Num3) {
          mode = SimulationMode::MultiRate;
          solver = PhysicsSolver();
          CreateMultiRateScene(solver);
          accumulator = 0.0f;
          finished = false;
        }

        if (key->code == sf::Keyboard::Key::R) {
          accumulator = 0.0f;
          finished = false;

          if (mode == SimulationMode::MultiRate) {
            solver = PhysicsSolver();
            CreateMultiRateScene(solver);
          } else {
            projectile =
                RigidBody(80.0f, 250.0f, 8.0f, 8.0f, 1200.0f, 0.0f, 1.0f);
          }
        }
      }
    }

    float frame_time = clock.restart().asSeconds();

    if (mode == SimulationMode::MultiRate) {
      solver.UpdateMultiRate(frame_time * simulation_scale, ordinary_wall,
                             fast_wall);
    }

    if (mode != SimulationMode::MultiRate) {
      accumulator += frame_time * simulation_scale;
    }

    while (accumulator >= fixed_delta_time && !finished &&
           mode != SimulationMode::MultiRate) {
      if (mode == SimulationMode::Discrete) {
        projectile.Update(fixed_delta_time);

        if (CheckAABB(projectile.GetBounds(), wall)) {
          projectile.Stop();
          finished = true;
        }

        if (projectile.GetX() > 780.0f) {
          projectile.Stop();
          finished = true;
        }
      } else if (mode == SimulationMode::CCD) {
        solver.UpdateCCD(projectile, wall, fixed_delta_time);

        if (projectile.GetVelocityX() == 0.0f) {
          finished = true;
        }
      }

      accumulator -= fixed_delta_time;
    }

    window.clear(sf::Color(30, 30, 35));

    if (mode == SimulationMode::MultiRate) {
      for (const RigidBody& body : solver.GetStaticBodies()) {
        sf::RectangleShape shape({body.GetWidth(), body.GetHeight()});
        shape.setPosition({body.GetX(), body.GetY()});
        shape.setFillColor(sf::Color(120, 120, 120));
        window.draw(shape);
      }

      for (const RigidBody& body : solver.GetOrdinaryBodies()) {
        sf::RectangleShape shape({body.GetWidth(), body.GetHeight()});
        shape.setPosition({body.GetX(), body.GetY()});
        shape.setFillColor(sf::Color(60, 140, 255));
        window.draw(shape);
      }

      for (const RigidBody& body : solver.GetFastBodies()) {
        sf::RectangleShape shape({body.GetWidth(), body.GetHeight()});
        shape.setPosition({body.GetX(), body.GetY()});
        shape.setFillColor(sf::Color(60, 220, 120));
        window.draw(shape);
      }

    } else {
      projectile_shape.setPosition({projectile.GetX(), projectile.GetY()});

      if (mode == SimulationMode::Discrete) {
        projectile_shape.setFillColor(sf::Color(60, 140, 255));
      } else {
        projectile_shape.setFillColor(sf::Color(60, 220, 120));
      }

      window.draw(wall_shape);
      window.draw(projectile_shape);
    }

    window.display();
  }

  return 0;
}
