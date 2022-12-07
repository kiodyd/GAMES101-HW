#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D offset = end - start;
        float step_x = offset.x / num_nodes;
        float step_y = offset.y / num_nodes;
        Mass* last_mass = nullptr;
        Vector2D current_pos = start;
        for (int i = 0; i < num_nodes; ++i)
        {
            Mass* mass = new Mass(current_pos, node_mass, false);
            if (last_mass != nullptr)
            {
                Spring* spring = new Spring(last_mass, mass, k);
                springs.push_back(spring);
            }
            last_mass = mass;
            masses.push_back(mass);
            current_pos.x += step_x;
            current_pos.y += step_y;
        }
//        Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D bToa = s->m2->position - s->m1->position;
            Vector2D force = -s->k * bToa.unit() * (bToa.norm() - s->rest_length);
            s->m1->forces += -force;
            s->m2->forces += force;
        }
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                // TODO (Part 2): Add global damping
                //另一种方法需要计算两个端点的速度
                m->forces -= 0.005 * m->velocity;

                Vector2D a = m->forces / m->mass;

                // 显式欧拉
//                m->position += m->velocity * delta_t;
//                m->velocity += a * delta_t;

                // 半隐式欧拉
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;

            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    // 好像不太对，后面研究下，目前直接用了pdf给出的公式
    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D bToa = s->m2->position - s->m1->position;
            Vector2D force = -s->k * bToa.unit() * (bToa.norm() - s->rest_length);
            s->m1->forces += -force;
            s->m2->forces += force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;

                m->forces += gravity * m->mass;

                float damping_factor = 0.0001;

                Vector2D a = m->forces / m->mass;

                m->position = m->position + (1 - damping_factor) * (m->position - m->last_position)
                        + a * delta_t * delta_t;
                m->last_position = temp_position;
                // TODO (Part 3.1): Set the new position of the rope mass
                
                // TODO (Part 4): Add global Verlet damping
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
