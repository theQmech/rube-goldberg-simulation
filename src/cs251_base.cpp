/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "cs251_base.hpp"
#include <cstdio>

using namespace std;
using namespace cs251;

base_sim_t::base_sim_t() {
  b2Vec2 gravity;
  gravity.Set(0.0f, -10.0f);
  m_world = new b2World(gravity);

  m_text_line = 30;
  m_mouse_joint = NULL;
  m_point_count = 0;

  m_world->SetDebugDraw(&m_debug_draw);

  m_step_count = 0;

  b2BodyDef body_def;
  m_ground_body = m_world->CreateBody(&body_def);

  memset(&m_max_profile, 0, sizeof(b2Profile));
  memset(&m_total_profile, 0, sizeof(b2Profile));
}

base_sim_t::~base_sim_t() {
  // By deleting the world, we delete the bomb, mouse joint, etc.
  delete m_world;
  m_world = NULL;
}

void base_sim_t::pre_solve(b2Contact *contact, const b2Manifold *oldManifold) {
  const b2Manifold *manifold = contact->GetManifold();

  if (manifold->pointCount == 0) {
    return;
  }

  b2Fixture *fixtureA = contact->GetFixtureA();
  b2Fixture *fixtureB = contact->GetFixtureB();

  b2PointState state1[b2_maxManifoldPoints], state2[b2_maxManifoldPoints];
  b2GetPointStates(state1, state2, oldManifold, manifold);

  b2WorldManifold world_manifold;
  contact->GetWorldManifold(&world_manifold);

  for (int32 i = 0;
       i < manifold->pointCount && m_point_count < k_max_contact_points; ++i) {
    contact_point_t *cp = m_points + m_point_count;
    cp->fixtureA = fixtureA;
    cp->fixtureB = fixtureB;
    cp->position = world_manifold.points[i];
    cp->normal = world_manifold.normal;
    cp->state = state2[i];
    ++m_point_count;
  }
}

class query_call_back : public b2QueryCallback {
public:
  query_call_back(const b2Vec2 &point) {
    m_point = point;
    m_fixture = NULL;
  }

  bool ReportFixture(b2Fixture *fixture) {
    b2Body *body = fixture->GetBody();
    if (body->GetType() == b2_dynamicBody) {
      bool inside = fixture->TestPoint(m_point);
      if (inside) {
        m_fixture = fixture;

        // We are done, terminate the query.
        return false;
      }
    }

    // Continue the query.
    return true;
  }

  b2Vec2 m_point;
  b2Fixture *m_fixture;
};

void base_sim_t::mouse_down(const b2Vec2 &p) {
  m_mouse_world = p;

  if (m_mouse_joint != NULL) {
    return;
  }

  // Make a small box.
  b2AABB aabb;
  b2Vec2 d;
  d.Set(0.001f, 0.001f);
  aabb.lowerBound = p - d;
  aabb.upperBound = p + d;

  // Query the world for overlapping shapes.
  query_call_back callback(p);
  m_world->QueryAABB(&callback, aabb);

  if (callback.m_fixture) {
    b2Body *body = callback.m_fixture->GetBody();
    b2MouseJointDef md;
    md.bodyA = m_ground_body;
    md.bodyB = body;
    md.target = p;
    md.maxForce = 1000.0f * body->GetMass();
    m_mouse_joint = (b2MouseJoint *)m_world->CreateJoint(&md);
    body->SetAwake(true);
  }
}

void base_sim_t::mouse_up(const b2Vec2 &p) {
  if (m_mouse_joint) {
    m_world->DestroyJoint(m_mouse_joint);
    m_mouse_joint = NULL;
  }
}

void base_sim_t::mouse_move(const b2Vec2 &p) {
  m_mouse_world = p;
  if (m_mouse_joint) {
    m_mouse_joint->SetTarget(p);
  }
}

void base_sim_t::draw_title(int x, int y, const char *string) {
  m_debug_draw.DrawString(x, y, string);
}

void base_sim_t::step(settings_t *settings) {
  float32 time_step = settings->hz > 0.0f ? 1.0f / settings->hz : float32(0.0f);

  if (settings->pause) {
    if (settings->single_step) {
      settings->single_step = 0;
    } else {
      time_step = 0.0f;
    }

    m_debug_draw.DrawString(5, m_text_line, "****PAUSED****");
    m_text_line += 15;
  }

  uint32 flags = 0;
  flags += settings->draw_shapes * b2Draw::e_shapeBit;
  flags += settings->draw_joints * b2Draw::e_jointBit;
  flags += settings->draw_AABBs * b2Draw::e_aabbBit;
  flags += settings->draw_pairs * b2Draw::e_pairBit;
  flags += settings->draw_COMs * b2Draw::e_centerOfMassBit;
  m_debug_draw.SetFlags(flags);

  m_world->SetWarmStarting(settings->enable_warm_starting > 0);
  m_world->SetContinuousPhysics(settings->enable_continuous > 0);
  m_world->SetSubStepping(settings->enable_sub_stepping > 0);

  m_point_count = 0;

  m_world->Step(time_step, settings->velocity_iterations,
                settings->position_iterations);

  m_world->DrawDebugData();

  if (time_step > 0.0f) {
    ++m_step_count;
  }

  if (settings->draw_stats) {
    int32 body_count = m_world->GetBodyCount();
    int32 contact_count = m_world->GetContactCount();
    int32 joint_count = m_world->GetJointCount();
    m_debug_draw.DrawString(5, m_text_line, "bodies/contacts/joints = %d/%d/%d",
                            body_count, contact_count, joint_count);
    m_text_line += 15;

    int32 proxy_count = m_world->GetProxyCount();
    int32 height = m_world->GetTreeHeight();
    int32 balance = m_world->GetTreeBalance();
    float32 quality = m_world->GetTreeQuality();
    m_debug_draw.DrawString(5, m_text_line,
                            "proxies/height/balance/quality = %d/%d/%d/%g",
                            proxy_count, height, balance, quality);
    m_text_line += 15;
  }

  if (m_mouse_joint) {
    b2Vec2 p1 = m_mouse_joint->GetAnchorB();
    b2Vec2 p2 = m_mouse_joint->GetTarget();

    glPointSize(4.0f);
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_POINTS);
    glVertex2f(p1.x, p1.y);
    glVertex2f(p2.x, p2.y);
    glEnd();
    glPointSize(1.0f);

    glColor3f(0.8f, 0.8f, 0.8f);
    glBegin(GL_LINES);
    glVertex2f(p1.x, p1.y);
    glVertex2f(p2.x, p2.y);
    glEnd();
  }

  // Track maximum profile times
  {
    const b2Profile &p = m_world->GetProfile();
    m_max_profile.step = b2Max(m_max_profile.step, p.step);
    m_max_profile.collide = b2Max(m_max_profile.collide, p.collide);
    m_max_profile.solve = b2Max(m_max_profile.solve, p.solve);
    m_max_profile.solveInit = b2Max(m_max_profile.solveInit, p.solveInit);
    m_max_profile.solveVelocity =
        b2Max(m_max_profile.solveVelocity, p.solveVelocity);
    m_max_profile.solvePosition =
        b2Max(m_max_profile.solvePosition, p.solvePosition);
    m_max_profile.solveTOI = b2Max(m_max_profile.solveTOI, p.solveTOI);
    m_max_profile.broadphase = b2Max(m_max_profile.broadphase, p.broadphase);

    m_total_profile.step += p.step;
    m_total_profile.collide += p.collide;
    m_total_profile.solve += p.solve;
    m_total_profile.solveInit += p.solveInit;
    m_total_profile.solveVelocity += p.solveVelocity;
    m_total_profile.solvePosition += p.solvePosition;
    m_total_profile.solveTOI += p.solveTOI;
    m_total_profile.broadphase += p.broadphase;
  }

  if (settings->draw_profile) {
    const b2Profile &p = m_world->GetProfile();

    b2Profile ave_profile;
    memset(&ave_profile, 0, sizeof(b2Profile));
    if (m_step_count > 0) {
      float32 scale = 1.0f / m_step_count;
      ave_profile.step = scale * m_total_profile.step;
      ave_profile.collide = scale * m_total_profile.collide;
      ave_profile.solve = scale * m_total_profile.solve;
      ave_profile.solveInit = scale * m_total_profile.solveInit;
      ave_profile.solveVelocity = scale * m_total_profile.solveVelocity;
      ave_profile.solvePosition = scale * m_total_profile.solvePosition;
      ave_profile.solveTOI = scale * m_total_profile.solveTOI;
      ave_profile.broadphase = scale * m_total_profile.broadphase;
    }

    m_debug_draw.DrawString(5, m_text_line,
                            "step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.step,
                            ave_profile.step, m_max_profile.step);
    m_text_line += 15;
    m_debug_draw.DrawString(
        5, m_text_line, "collide [ave] (max) = %5.2f [%6.2f] (%6.2f)",
        p.collide, ave_profile.collide, m_max_profile.collide);
    m_text_line += 15;
    m_debug_draw.DrawString(5, m_text_line,
                            "solve [ave] (max) = %5.2f [%6.2f] (%6.2f)",
                            p.solve, ave_profile.solve, m_max_profile.solve);
    m_text_line += 15;
    m_debug_draw.DrawString(
        5, m_text_line, "solve init [ave] (max) = %5.2f [%6.2f] (%6.2f)",
        p.solveInit, ave_profile.solveInit, m_max_profile.solveInit);
    m_text_line += 15;
    m_debug_draw.DrawString(
        5, m_text_line, "solve velocity [ave] (max) = %5.2f [%6.2f] (%6.2f)",
        p.solveVelocity, ave_profile.solveVelocity,
        m_max_profile.solveVelocity);
    m_text_line += 15;
    m_debug_draw.DrawString(
        5, m_text_line, "solve position [ave] (max) = %5.2f [%6.2f] (%6.2f)",
        p.solvePosition, ave_profile.solvePosition,
        m_max_profile.solvePosition);
    m_text_line += 15;
    m_debug_draw.DrawString(
        5, m_text_line, "solveTOI [ave] (max) = %5.2f [%6.2f] (%6.2f)",
        p.solveTOI, ave_profile.solveTOI, m_max_profile.solveTOI);
    m_text_line += 15;
    m_debug_draw.DrawString(
        5, m_text_line, "broad-phase [ave] (max) = %5.2f [%6.2f] (%6.2f)",
        p.broadphase, ave_profile.broadphase, m_max_profile.broadphase);
    m_text_line += 15;
  }

  if (settings->draw_contact_points) {
    // const float32 k_impulseScale = 0.1f;
    const float32 k_axis_scale = 0.3f;

    for (int32 i = 0; i < m_point_count; ++i) {
      contact_point_t *point = m_points + i;

      if (point->state == b2_addState) {
        // Add
        m_debug_draw.DrawPoint(point->position, 10.0f,
                               b2Color(0.3f, 0.95f, 0.3f));
      } else if (point->state == b2_persistState) {
        // Persist
        m_debug_draw.DrawPoint(point->position, 5.0f,
                               b2Color(0.3f, 0.3f, 0.95f));
      }

      if (settings->draw_contact_normals == 1) {
        b2Vec2 p1 = point->position;
        b2Vec2 p2 = p1 + k_axis_scale * point->normal;
        m_debug_draw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.9f));
      } else if (settings->draw_contact_forces == 1) {
        // b2Vec2 p1 = point->position;
        // b2Vec2 p2 = p1 + k_forceScale * point->normalForce * point->normal;
        // DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
      }

      if (settings->draw_friction_forces == 1) {
        // b2Vec2 tangent = b2Cross(point->normal, 1.0f);
        // b2Vec2 p1 = point->position;
        // b2Vec2 p2 = p1 + k_forceScale * point->tangentForce * tangent;
        // DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
      }
    }
  }
}
