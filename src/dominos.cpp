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

/*
 * Base code for CS 251 Software Systems Lab
 * Department of Computer Science and Engineering, IIT Bombay
 *
 */

#include "cs251_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

namespace cs251 {
/** The is the constructor
 *  This is the documentation block for the constructor.
 */

dominos_t::dominos_t() {
  // Original-Commented Code
  {

      // Ground
      /*! \var b1
       * \brief pointer to the body ground
       */
      // b2Body* b1;
      // {

      //  b2EdgeShape shape;
      //  shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      //  b2BodyDef bd;
      //  b1 = m_world->CreateBody(&bd);
      //  b1->CreateFixture(&shape, 0.0f);
      // }

      //    //Top horizontal shelf
      //    {
      //      b2PolygonShape shape;
      //      shape.SetAsBox(6.0f, 0.25f);

      //      b2BodyDef bd;
      //      bd.position.Set(-31.0f, 30.0f);
      //      b2Body* ground = m_world->CreateBody(&bd);
      //      ground->CreateFixture(&shape, 0.0f);
      //    }

      //    //Dominos
      //    {
      //      b2PolygonShape shape;
      //      shape.SetAsBox(0.1f, 1.0f);

      //      b2FixtureDef fd;
      //      fd.shape = &shape;
      //      fd.density = 20.0f;
      //      fd.friction = 0.1f;

      //      for (int i = 0; i < 10; ++i)
      // {
      //   b2BodyDef bd;
      //   bd.type = b2_dynamicBody;
      //   bd.position.Set(-35.5f + 1.0f * i, 31.25f);
      //   b2Body* body = m_world->CreateBody(&bd);
      //   body->CreateFixture(&fd);
      // }
      //    }

      //    //Another horizontal shelf
      //    {
      //      b2PolygonShape shape;
      //      shape.SetAsBox(7.0f, 0.25f, b2Vec2(-20.f,20.f), 0.0f);

      //      b2BodyDef bd;
      //      bd.position.Set(1.0f, 6.0f);
      //      b2Body* ground = m_world->CreateBody(&bd);
      //      ground->CreateFixture(&shape, 0.0f);
      //    }

      // The pendulum that knocks the dominos off
      //     {
      //       b2Body* b2;
      //       {
      //  b2PolygonShape shape;
      //  shape.SetAsBox(0.25f, 1.5f);

      // b2BodyDef bd;
      // bd.position.Set(-36.5f, 28.0f);
      // b2 = m_world->CreateBody(&bd);
      // b2->CreateFixture(&shape, 10.0f);
      //      }

      //      b2Body* b4;
      //      {
      // b2PolygonShape shape;
      // shape.SetAsBox(0.25f, 0.25f);

      // b2BodyDef bd;
      // bd.type = b2_dynamicBody;
      // bd.position.Set(-40.0f, 33.0f);
      // b4 = m_world->CreateBody(&bd);
      // b4->CreateFixture(&shape, 2.0f);
      //      }

      //      b2RevoluteJointDef jd;
      //      b2Vec2 anchor;
      //      anchor.Set(-37.0f, 40.0f);
      //      jd.Initialize(b2, b4, anchor);
      //      m_world->CreateJoint(&jd);
      //    }

      //    //The train of small spheres
      //    {
      //      b2Body* spherebody;

      //      b2CircleShape circle;
      //      circle.m_radius = 0.5;

      //      b2FixtureDef ballfd;
      //      ballfd.shape = &circle;
      //      ballfd.density = 1.0f;
      //      ballfd.friction = 0.0f;
      //      ballfd.restitution = 0.0f;

      //      for (int i = 0; i < 10; ++i)
      // {
      //   b2BodyDef ballbd;
      //   ballbd.type = b2_dynamicBody;
      //   ballbd.position.Set(-22.2f + i*1.0, 26.6f);
      //   spherebody = m_world->CreateBody(&ballbd);
      //   spherebody->CreateFixture(&ballfd);
      // }
      //    }

      //    //The pulley system
      // {
      //   b2BodyDef *bd = new b2BodyDef;
      //   bd->type = b2_dynamicBody;
      //   bd->position.Set(-10,15);
      //   bd->fixedRotation = true;

      // //The open box
      // b2FixtureDef *fd1 = new b2FixtureDef;
      // fd1->density = 11.37;
      // fd1->friction = 0.5;
      // fd1->restitution = 0.f;
      // fd1->shape = new b2PolygonShape;
      // b2PolygonShape bs1;
      // bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      // fd1->shape = &bs1;
      // b2FixtureDef *fd2 = new b2FixtureDef;
      // fd2->density = 11.37;
      // fd2->friction = 0.5;
      // fd2->restitution = 0.f;
      // fd2->shape = new b2PolygonShape;
      // b2PolygonShape bs2;
      // bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      // fd2->shape = &bs2;
      // b2FixtureDef *fd3 = new b2FixtureDef;
      // fd3->density = 11.37;
      // fd3->friction = 0.5;
      // fd3->restitution = 0.f;
      // fd3->shape = new b2PolygonShape;
      // b2PolygonShape bs3;
      // bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      // fd3->shape = &bs3;

      // b2Body* box1 = m_world->CreateBody(bd);
      // box1->CreateFixture(fd1);
      // box1->CreateFixture(fd2);
      // box1->CreateFixture(fd3);

      //      //The bar
      //      bd->position.Set(10,15);
      //      fd1->density = 34.0;
      //      b2Body* box2 = m_world->CreateBody(bd);
      //      box2->CreateFixture(fd1);

      // // The pulley joint
      // b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      // b2Vec2 worldAnchorOnBody1(-10, 15); // Anchor point on body 1 in world
      // axis b2Vec2 worldAnchorOnBody2(10, 15); // Anchor point on body 2 in
      // world axis b2Vec2 worldAnchorGround1(-10, 20); // Anchor point for
      // ground 1 in world axis b2Vec2 worldAnchorGround2(10, 20); // Anchor
      // point for ground 2 in world axis float32 ratio = 1.0f; // Define ratio
      // myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2,
      // box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      // m_world->CreateJoint(myjoint);
      //    }

      //    //The revolving horizontal platform
      //    {
      //      b2PolygonShape shape;
      //      shape.SetAsBox(2.2f, 0.2f);

      //      b2BodyDef bd;
      //      bd.position.Set(14.0f, 14.0f);
      //      bd.type = b2_dynamicBody;
      //      b2Body* body = m_world->CreateBody(&bd);
      //      b2FixtureDef *fd = new b2FixtureDef;
      //      fd->density = 1.f;
      //      fd->shape = new b2PolygonShape;
      //      fd->shape = &shape;
      //      body->CreateFixture(fd);

      //      b2PolygonShape shape2;
      //      shape2.SetAsBox(0.2f, 2.0f);
      //      b2BodyDef bd2;
      //      bd2.position.Set(14.0f, 16.0f);
      //      b2Body* body2 = m_world->CreateBody(&bd2);

      //      b2RevoluteJointDef jointDef;
      //      jointDef.bodyA = body;
      //      jointDef.bodyB = body2;
      //      jointDef.localAnchorA.Set(0,0);
      //      jointDef.localAnchorB.Set(0,0);
      //      jointDef.collideConnected = false;
      //      m_world->CreateJoint(&jointDef);
      //    }

      //    //The heavy sphere on the platform
      //    {
      //      b2Body* sbody;
      //      b2CircleShape circle;
      //      circle.m_radius = 1.0;

      //      b2FixtureDef ballfd;
      //      ballfd.shape = &circle;
      //      ballfd.density = 50.0f;
      //      ballfd.friction = 0.0f;
      //      ballfd.restitution = 0.0f;
      //      b2BodyDef ballbd;
      //      ballbd.type = b2_dynamicBody;
      //      ballbd.position.Set(14.0f, 18.0f);
      //      sbody = m_world->CreateBody(&ballbd);
      //      sbody->CreateFixture(&ballfd);
      //    }

      //    //The see-saw system at the bottom
      //    {
      //      //The triangle wedge
      //      b2Body* sbody;
      //      b2PolygonShape poly;
      //      b2Vec2 vertices[3];
      //      vertices[0].Set(-1,0);
      //      vertices[1].Set(1,0);
      //      vertices[2].Set(0,1.5);
      //      poly.Set(vertices, 3);
      //      b2FixtureDef wedgefd;
      //      wedgefd.shape = &poly;
      //      wedgefd.density = 10.0f;
      //      wedgefd.friction = 0.0f;
      //      wedgefd.restitution = 0.0f;
      //      b2BodyDef wedgebd;
      //      wedgebd.position.Set(30.0f, 0.0f);
      //      sbody = m_world->CreateBody(&wedgebd);
      //      sbody->CreateFixture(&wedgefd);

      //      //The plank on top of the wedge
      //      b2PolygonShape shape;
      //      shape.SetAsBox(15.0f, 0.2f);
      //      b2BodyDef bd2;
      //      bd2.position.Set(30.0f, 1.5f);
      //      bd2.type = b2_dynamicBody;
      //      b2Body* body = m_world->CreateBody(&bd2);
      //      b2FixtureDef *fd2 = new b2FixtureDef;
      //      fd2->density = 1.f;
      //      fd2->shape = new b2PolygonShape;
      //      fd2->shape = &shape;
      //      body->CreateFixture(fd2);

      //      b2RevoluteJointDef jd;
      //      b2Vec2 anchor;
      //      anchor.Set(30.0f, 1.5f);
      //      jd.Initialize(sbody, body, anchor);
      //      m_world->CreateJoint(&jd);

      //      //The light box on the right side of the see-saw
      //      b2PolygonShape shape2;
      //      shape2.SetAsBox(2.0f, 2.0f);
      //      b2BodyDef bd3;
      //      bd3.position.Set(40.0f, 2.0f);
      //      bd3.type = b2_dynamicBody;
      //      b2Body* body3 = m_world->CreateBody(&bd3);
      //      b2FixtureDef *fd3 = new b2FixtureDef;
      //      fd3->density = 18.f;
      //      fd3->friction = 0.1f;
      //      fd3->shape = new b2PolygonShape;
      //      fd3->shape = &shape2;
      //      body3->CreateFixture(fd3);
      //    }

      //    //The Propellor
      // {
      //  b2PolygonShape shape;
      //  shape.SetAsBox(2.2f, 0.2f);

      //  b2BodyDef bd;
      //  bd.position.Set(-10.2f, 30.6f);
      //  bd.type = b2_dynamicBody;
      //  b2Body* body = m_world->CreateBody(&bd);

      //  b2FixtureDef *fd = new b2FixtureDef;
      //  fd->density = 20.f;
      //  fd->shape = new b2PolygonShape;
      //  fd->shape = &shape;
      //  fd->restitution = 1.f;
      //  body->CreateFixture(fd);

      //  b2PolygonShape shape2;
      //  shape2.SetAsBox(2.0f, 0.2f);
      //  b2BodyDef bd2;
      //  bd2.position.Set(-10.2f, 20.7f);
      //  b2Body* body2 = m_world->CreateBody(&bd2);

      //  b2RevoluteJointDef jointDef;
      //  jointDef.bodyA = body;
      //  jointDef.bodyB = body2;
      //  jointDef.localAnchorA.Set(0,-6);
      //  jointDef.localAnchorB.Set(0,0);
      //  jointDef.collideConnected = false;
      //  m_world->CreateJoint(&jointDef);
      // }

      // /// ## Middle horizontal shelf
      // {
      //  b2PolygonShape shape;
      //  shape.SetAsBox(1.f, 0.25f); /// create a rectangle

      //  b2BodyDef bd;
      //  bd.position.Set(-10.2f, 13.0f); /// Set the position
      //  b2Body* ground = m_world->CreateBody(&bd);
      //  ground->CreateFixture(&shape, 0.0f); /// Fixture
      // }

      // /// ## Small Sphere On Plank
      // {
      //  b2Body* spherebody;

      //      b2CircleShape circle;
      //      circle.m_radius = 3;

      //      b2FixtureDef ballfd;
      //      ballfd.shape = &circle;
      //      ballfd.density = 0.001f;
      //      ballfd.friction = 0.0f;
      //      ballfd.restitution = 0.0f;

      //   b2BodyDef ballbd;
      //   ballbd.type = b2_dynamicBody;
      //   ballbd.position.Set(35.0f, 2.0f);
      //   spherebody = m_world->CreateBody(&ballbd);
      //   spherebody->CreateFixture(&ballfd);
      // }
  }

  {
    /** Section-3
     *  This Section Contains the top-right part of the simulation.
     *  Different Parts are:
     *  1. Spring-System
     *  2. Stop & Push Hammer
     *  3. 360 degrees Pendulum with 2 boxes
     *  4. Dominos-Shots
     *  Each part of this section is wrapped in a separate block of code
     *  Each one begins with 'Position and Scalability Factors'
     * (self-explanatory) sec3_x and sec3_y can be used to change the
     * coordinates of the entire Section3. sec3_scale can be used to scale the
     * size of Section3
     */
    float sec3_x = 6, sec3_y = 27.5;
    float sec3_scale = 0.4;
    {
      /** OVER-BALANCED WHEEl
       *  This is a perpetual machine, a combination of 'size' ball-objects,
       * 'size' stick-objects and one central rotating body The central body
       * contains 12 triangular fixtures They are all connected by Revolute
       * Joints. Additional float variable weight_scale scales the weight of all
       * bodies in the wheel Manually Defined functions - rotation_joint and
       * two_body_rotation_joint were used.
       */
      // Position and Scalability Factors
      float x = -30 * sec3_scale + sec3_x, y = 0 * sec3_scale + sec3_y;
      float scale = 0.8 * sec3_scale;
      float weight_scale = 0.1;
      const int size = 15; // Number of Balls at corners
      {                    // CODE
        b2Body *body;

        // Body definition
        b2BodyDef myBodyDef;
        myBodyDef.type = b2_dynamicBody;

        /** Fixture density and restitution are defined
         *  Only the shape of the fixture is changed in the For-Loop
         */
        b2PolygonShape polygonShape;
        b2FixtureDef myFixtureDef;
        myFixtureDef.shape =
            &polygonShape; // Change only Polygon shape later in For-Loop
        myFixtureDef.density = 50 * weight_scale;
        myFixtureDef.restitution = 0;

        //// Central Body is created
        body = m_world->CreateBody(&myBodyDef);

        /** Fixture shape is defined using b2PolygonShape
         *  Multiple fixtures are attached to the Central Body
         */
        float length1 = 7.2, length2 = 8; // lenght1 < length2
        b2Vec2 vertices[3];
        float angle = 0;
        for (int i = 0; i < size;) { // Attaching Fixtures
          vertices[0].Set(0, 0);
          vertices[1].Set(scale * length1 * sinf(angle),
                          scale * length1 * cosf(angle));
          i++;
          angle = -i * 360 * DEGTORAD * 1.0 / size;
          vertices[2].Set(scale * length2 * sinf(angle),
                          scale * length2 * cosf(angle));
          polygonShape.Set(vertices, 3);
          body->CreateFixture(&myFixtureDef);
        }

        /** Outer Stick and Balls are created as fixtures, added to a body and
         * attached to the Central Body using b2RevoluteJointDef
         * dominos_t::two_body_rotation_joint Sticks are created using
         * b2PolygonShape::SetAsBox(); Balls are created using b2CircleShape.
         *  The stick-ball body is made more sensitive to gravity using
         * b2Body.SetGravityScale()
         */
        angle = 360 * DEGTORAD / size;
        double length3 = length1 * length1 + length2 * length2 -
                         2 * length1 * length2 * cosf(angle);
        length3 = sqrt(length3);
        double angle_limit =
            180 * DEGTORAD -
            acosf((length1 * length1 + length3 * length3 - length2 * length2) /
                  (2 * length1 * length3));
        for (int i = 0; i < size; i++) { // Outer Balls

          angle = -i * 360 * DEGTORAD * 1.0 / size;
          b2BodyDef stickDef;
          stickDef.type = b2_dynamicBody;
          b2Body *stickBody = m_world->CreateBody(&stickDef);
          float box_length = length1 / 2;
          // Circle shape
          b2CircleShape circle;
          circle.m_radius = 0.4 * scale;
          circle.m_p.Set(0, box_length * scale);
          // Box shape
          polygonShape.SetAsBox(0.05f * scale, box_length * scale);
          myFixtureDef.density = 0.1 * weight_scale;
          myFixtureDef.shape = &circle;
          myFixtureDef.restitution = 0;
          stickBody->CreateFixture(&myFixtureDef);
          myFixtureDef.density = 0.1;
          myFixtureDef.shape = &polygonShape;
          myFixtureDef.restitution = 0;
          stickBody->CreateFixture(&myFixtureDef);
          stickBody->SetTransform(b2Vec2(x + scale * length1 * sinf(angle),
                                         y + scale * length1 * cosf(angle)),
                                  -angle);
          stickBody->SetGravityScale(3);

          b2RevoluteJointDef jointDef = two_body_rotation_joint(
              body, stickBody, scale * length1 * sinf(angle),
              scale * length1 * cosf(angle), 0, -box_length * scale);
          jointDef.collideConnected = false;
          jointDef.enableLimit = true;
          jointDef.lowerAngle = -angle + 0 * DEGTORAD;
          jointDef.upperAngle = -angle + angle_limit;
          // create the joint
          m_world->CreateJoint(&jointDef);
        }

        /** The Central Body is fixed about its centre by creating a
         * b2RevoluteJoint using dominos_t::rotation_joint
         */
        rotation_joint(body, x, y, 0, 0);

        /** The Central Body and each stick-ball body is set into position using
         * b2Body.SetTransform()
         */
        body->SetTransform(b2Vec2(x, y), 0);
        // body->SetAngularVelocity(0.1);
      }
    }

    {
      /** SPRING-SYSTEM
       *  This system contains multiple bodies that interact with each other. It
       * is a collection of a) a Spring (attached to a veritical bar) b) a Ball
       *  c) a Push-Hammer
       *  d) an Upper Platform
       *  e) and a Lower Platform
       *  Manually Defined functions - dominos_t::rotation_joint and
       * dominos_t::two_body_rotation_joint were used.
       */
      // Position and Scalability Factors
      float x = -50 * sec3_scale + sec3_x, y = 20 * sec3_scale + sec3_y;
      float scale = 0.5 * sec3_scale;
      const int size = 4; // Length of Spring
      {                   // CODE
        {
          /** Upper - Platform
           *  This is a b2EdgeShape Body that supports the spring on its top
           */
          float length = size * 8 * scale;
          float height = 5 * scale;
          b2Body *b1;
          b2EdgeShape shape;
          shape.Set(b2Vec2(x, y + height), b2Vec2(x + length, y + height));
          b2BodyDef bd;
          b1 = m_world->CreateBody(&bd);
          b1->CreateFixture(&shape, 0.0f);
        }
        {
          /** Lower - Platform
           *  This is a b2EdgeShape Body that supports the ball and the spring
           * (partially) rest on this platform
           */
          float length = (size * 5 + 30) * scale;
          float height = -5 * scale;
          b2Body *b1;
          b2EdgeShape shape;
          shape.Set(b2Vec2(x + (size - 1) * 5 * scale, y + height),
                    b2Vec2(x + (size - 1) * 7 * scale + length, y + height));
          b2BodyDef bd;
          b1 = m_world->CreateBody(&bd);
          b1->CreateFixture(&shape, 0.0f);
        }
        {
          /** Side - Platform
           *  This is a b2EdgeShape body that supports the spring to its left.
           * The left end of the spring is attached to it
           */
          float length = 10 * scale;
          b2Body *b1;
          b2EdgeShape shape;
          shape.Set(b2Vec2(x, y - length / 2), b2Vec2(x, y + length / 2));
          b2BodyDef bd;
          b1 = m_world->CreateBody(&bd);
          b1->CreateFixture(&shape, 0.0f);
        }
        {
          /** Spring
           *  This is a complicated collection of bodies connected to each other
           * using b2RevoluteJoints (dominos_t::two_body_rotation_joint) This
           * converts vertical thrust into horizontal thurst
           */
          // Position and Scalability Factors
          float x_spring = x, y_spring = y;
          float scale_spring = 1 * scale;
          float length = 5 * scale_spring;
          float thickness = 0.1 * scale_spring;
          float density = 1;
          float angle = 30 * DEGTORAD;

          { // CODE
            /** Each Segment of the spring is a separate body.
             *  The bodies are defined as Boxes using b2PolygonShape::SetAsBox()
             * and initaialized using b2Body::SetTransform(). All bodies are
             * made weighless using 2Body::SetGravityScale() to prevent
             * spontaneous collapse.
             */
            b2Body *body1, *body2, *body1_prev, *body2_prev;

            b2BodyDef bodyDef;
            bodyDef.type = b2_dynamicBody;

            b2PolygonShape polyShape;
            polyShape.SetAsBox(length / 2, thickness);
            b2FixtureDef FixtureDef;
            FixtureDef.shape = &polyShape;
            FixtureDef.density = density;
            FixtureDef.friction = 0;

            body1 = m_world->CreateBody(&bodyDef);
            body2 = m_world->CreateBody(&bodyDef);

            /** The first two bodies (which are smaller than the rest) are
             * joined to the side platform by their ends.
             */
            rotation_joint(body1, x, y, -length / 2, 0);
            rotation_joint(body2, x, y, -length / 2, 0);

            body1->SetGravityScale(0);
            body2->SetGravityScale(0);
            body1->CreateFixture(&FixtureDef);
            body2->CreateFixture(&FixtureDef);
            b2RevoluteJointDef tempJointDef;
            polyShape.SetAsBox(length, thickness);

            body1_prev = body1;
            body2_prev = body2;

            int i = 1;

            body1 = m_world->CreateBody(&bodyDef);
            body2 = m_world->CreateBody(&bodyDef);
            body1->SetGravityScale(0);
            body2->SetGravityScale(0);
            body1->CreateFixture(&FixtureDef);
            body2->CreateFixture(&FixtureDef);
            tempJointDef = two_body_rotation_joint(body1, body2, 0, 0, 0, 0);
            m_world->CreateJoint(&tempJointDef);
            tempJointDef = two_body_rotation_joint(body1, body2_prev, -length,
                                                   0, length / 2, 0);
            m_world->CreateJoint(&tempJointDef);
            tempJointDef = two_body_rotation_joint(body2, body1_prev, -length,
                                                   0, length / 2, 0);
            m_world->CreateJoint(&tempJointDef);

            /** The next two bodies are joined to each other at the centre.
             * Their ends are also joined to the ends of the previous 2 bodies.
             * (All using dominos_t::two_body_rotation_joint) Similarly, the
             * remaining bodies are connected to each other iteratively.
             */
            body1_prev->SetTransform(
                b2Vec2(x_spring + (i - 1) * length * cosf(angle), y_spring),
                -angle);
            body2_prev->SetTransform(
                b2Vec2(x_spring + (i - 1) * length * cosf(angle), y_spring),
                angle);

            body1_prev = body1;
            body2_prev = body2;

            for (i = 2; i < size; ++i) {
              body1 = m_world->CreateBody(&bodyDef);
              body2 = m_world->CreateBody(&bodyDef);
              body1->SetGravityScale(0);
              body2->SetGravityScale(0);
              body1->CreateFixture(&FixtureDef);
              body2->CreateFixture(&FixtureDef);
              tempJointDef = two_body_rotation_joint(body1, body2, 0, 0, 0, 0);
              m_world->CreateJoint(&tempJointDef);
              tempJointDef = two_body_rotation_joint(body1, body2_prev, -length,
                                                     0, length, 0);
              m_world->CreateJoint(&tempJointDef);
              tempJointDef = two_body_rotation_joint(body2, body1_prev, -length,
                                                     0, length, 0);
              m_world->CreateJoint(&tempJointDef);

              body1_prev->SetTransform(
                  b2Vec2(x_spring + (i - 1) * length * cosf(angle), y_spring),
                  angle);
              body2_prev->SetTransform(
                  b2Vec2(x_spring + (i - 1) * length * cosf(angle), y_spring),
                  -angle);

              body1_prev = body1;
              body2_prev = body2;
            }

            body1->SetTransform(
                b2Vec2(x_spring + (i - 1) * length * cosf(angle), y_spring),
                angle);
            body2->SetTransform(
                b2Vec2(x_spring + (i - 1) * length * cosf(angle), y_spring),
                -angle);
          }
        }
        {
          /** Ball
           *  This is a simple dynamic ball which will be pushed by the spring
           * during the simulation.
           */
          b2Body *body;
          b2BodyDef myBodyDef;
          myBodyDef.type = b2_dynamicBody;

          b2CircleShape circle;
          circle.m_radius = 3 * scale;
          circle.m_p.Set(x + (size)*7 * scale, y - 4 * scale);

          b2FixtureDef myFixture;
          myFixture.shape = &circle;
          myFixture.density = 1;
          myFixture.restitution = 0;
          myFixture.friction = 0;

          body = m_world->CreateBody(&myBodyDef);
          body->CreateFixture(&myFixture);
          body->SetGravityScale(10);
        }

        {
          /** Push-Hammer
           *  This is a Hammer shaped body containing 2 fixtures - the narrow
           * stick and the big heavy box. It is hinged about its centre on the
           * stick using dominos_t::rotation_joint. Initially placed in unstable
           * equilibrium, a slight perturbation results in the hammer falling
           * and ... well why don't you see it yourself? :)
           */
          /// Position and Scalability Factors
          float scale_hammer = 2 * scale;
          float length = 10 * scale_hammer;
          float length_box = 2 * scale_hammer;
          float thickness = 0.2 * scale_hammer;
          float density = 100;

          float x_hammer = x + (size)*7 * scale + 30 * scale,
                y_hammer = y + length - 2 * scale;

          b2Body *body;
          b2BodyDef bodyDef;
          bodyDef.type = b2_dynamicBody;

          /** The narrow stick is defined using b2PolygonShape::SetAsBox(); The
           * Big Box is defined by its vertices using b2PolygonShape::Set().
           */
          b2PolygonShape polyShape;
          polyShape.SetAsBox(thickness, length);
          b2FixtureDef FixtureDef;
          FixtureDef.shape = &polyShape;
          FixtureDef.density = 1;
          FixtureDef.friction = 0;
          FixtureDef.restitution = 0;

          body = m_world->CreateBody(&bodyDef);
          body->CreateFixture(&FixtureDef);

          b2Vec2 vertices[4];
          vertices[0].Set(length_box, length);
          vertices[1].Set(length_box, length + length_box * 1.6);
          vertices[2].Set(-length_box, length + length_box * 1.6);
          vertices[3].Set(-length_box, length);
          polyShape.Set(vertices, 4);
          FixtureDef.density = density;

          body->CreateFixture(&FixtureDef);
          rotation_joint(body, x_hammer, y_hammer + length_box * 0.8, 0,
                         length_box * 0.8);
          body->SetTransform(b2Vec2(x_hammer, y_hammer), 0);

          /** To increase inpact-thrust, the hammer's Sensitivity to  gravity is
           * increased using b2Body::SetGravityScale();
           */
          body->SetGravityScale(5);
        }
      }
    }

    {
      /** PENDULUM-SYSTEM
       *  This system contains:
       *  a) a Horizontal Platform (Edge)
       *  b) 2 dynamic boxes on the horizontal platform
       *  c) a Vertical Edge
       *  d) a Pendulum (attached to the veritical Edge)
       *  Manually Defined function - dominos_t::rotation_joint was used.
       *  Additional float variable weight_scale scales the weight of all bodies
       * in the pendulum system
       */
      float x = 22 * sec3_scale + sec3_x, y = 5 * sec3_scale + sec3_y;
      float scale = 2 * sec3_scale;
      float weight_scale = 0.08;
      { // CODE
        float height = 5 * scale;
        {
          /** Upper-Platform
           *  This is a b2EdgeShape Body that supports the 2 boxes on its top
           */
          b2Body *b;
          float width = 4 * scale;
          b2EdgeShape shape;
          shape.Set(b2Vec2(-width / 2, height), b2Vec2(width / 2, height));

          b2BodyDef bd;
          bd.position.Set(0, 0);
          b = m_world->CreateBody(&bd);
          b->CreateFixture(&shape, 0.0f);
          b->SetTransform(b2Vec2(x, y), 0.0f);
        }

        b2Body *b1;
        {
          /** Vertical-Platform
           *  This is a b2EdgeShape body that supports the spring to its left.
           * The left end of the spring is attached to it.
           */
          float length = height;
          b2EdgeShape shape;
          shape.Set(b2Vec2(0, 0), b2Vec2(0, length));

          b2BodyDef bd;
          bd.position.Set(0, 0);
          b1 = m_world->CreateBody(&bd);
          b1->CreateFixture(&shape, 10.0f);
          b1->SetTransform(b2Vec2(x, y), 0.0f);
        }

        b2Body *b2;
        {
          /** Pendulum-Bob
           *  This is a b2CircleShape body that has been hinged to the bottom of
           * the Vertical-Platform using dominos_t::rotation_joint.
           */
          b2CircleShape shape;
          shape.m_radius = 0.5 * scale;

          b2BodyDef bd;
          bd.type = b2_dynamicBody;
          bd.position.Set(0, 0);
          b2 = m_world->CreateBody(&bd);

          b2FixtureDef fix;
          fix.shape = &shape;
          fix.density = 9 * weight_scale;
          fix.restitution = 1;
          b2->CreateFixture(&fix);
          b2->SetTransform(
              b2Vec2(x, y - (height + 0.5 * scale + shape.m_radius)), 0.0f);
          rotation_joint(b2, x, y, 0, (height + 0.5 * scale + shape.m_radius));
        }

        {
          /** 2 Boxes on Platform
           *  These are b2PolygonShape square bodies defined using
           * b2PolygonShape::SetAsBox().
           */
          float side = 0.75 * scale;
          b2Body *box1, *box2;

          b2PolygonShape shape;
          shape.SetAsBox(side, side);

          b2BodyDef def;
          def.type = b2_dynamicBody;
          box1 = m_world->CreateBody(&def);
          box2 = m_world->CreateBody(&def);

          b2FixtureDef fix;
          fix.shape = &shape;
          fix.restitution = 1;
          fix.density = 15 * weight_scale;

          box1->CreateFixture(&fix);
          box2->CreateFixture(&fix);

          box1->SetTransform(b2Vec2(x - side - scale * 0.1, y + height + side),
                             0);
          box2->SetTransform(b2Vec2(x + side + scale * 0.1, y + height + side),
                             0);
        }
      }
    }

    {
      /** DOMINOS SHOTS
       *  This is a collection of "containers" created using (manually defined)
       * dominos_t::container() filled with (fluid like) balls using (manually
       * defined) dominos_t::fill_fluid() The aim is to keep multiple small
       * "containers" on top of edges of multiple big "containers" in close
       * proximity to create a dominos-like effect. PS: As you may have guessed,
       * this was motivated by Bar-Shot Dominos :P
       */
      // Position and Scalability Factors
      float x = 50 * sec3_scale + sec3_x, y = -12 * sec3_scale + sec3_y;

      /**
       *  'number' contains the "length" of the dominos
       */

      int number = 6;
      float scale = 2 * sec3_scale;
      { // CODE
        float x_start = x, y_start = y + 0.1;
        float height = 7.5 * scale, width = 4 * scale, thickness = 0.2 * scale;
        {
          /** Big Glasses
           *  Height and width variabless are defined outside code block because
           * they are needed in other code blocks also. Bodies (containers) with
           * height, width and thickness are created using
           * dominos_t::container() and are filled inside using
           * dominos_t::fill_fluid()
           */
          b2Body *b1;
          float x_new = x_start + width / 2;
          for (int i = 0; i < number; i++) {
            b1 = container(height, width, thickness, 5);
            b1->SetTransform(b2Vec2(x_new, y_start), 0);
            fill_fluid(x_new + thickness - width / 2, y_start + thickness,
                       0.8f * height, width - 2 * thickness, 0.25, 0.1);
            x_new += width + 0.1 * scale; /// A gap of 0.1 units is left between
                                          /// 2 consecutive "Big Glasses"
          }
        }

        {
          /** Small Shots
           *  Height and width variabless from previous code block are used.
           *  Bodies (containers) with shot_height, shot_width and thickness are
           * created using dominos_t::container() and are filled inside using
           * dominos_t::fill_fluid() They are positioned above the "Big Glasses"
           */
          float shot_height = 4 * scale, shot_width = 2 * scale,
                thickness = 0.1 * scale;
          float shot_x_start = x_start + thickness,
                shot_y_start = y_start + height;

          b2Body *b1;
          float shot_x_new = shot_x_start;
          for (int i = 0; i < number; i++) {
            b1 = container(shot_height, shot_width, thickness, 5);
            b1->SetTransform(b2Vec2(shot_x_new, shot_y_start), 0);
            fill_fluid(shot_x_new + thickness - shot_width / 2,
                       shot_y_start + thickness, 0.8 * shot_height,
                       shot_width - 2 * thickness, 0.25, 0.1);
            shot_x_new += width + 0.1 * scale;
          }
        }

        {
          /** Horizontal-Platform
           *  This is a b2EdgeShape Body that supports all the "containers" on
           * its top
           */
          float x1 = x, y1 = y;
          float width = 5 * (number)*scale;
          b2Body *b1;
          b2EdgeShape shape;
          shape.Set(b2Vec2(0, 0.0f), b2Vec2(width, 0.0f));
          b2BodyDef bd;
          b1 = m_world->CreateBody(&bd);
          b1->CreateFixture(&shape, 0.0f);
          b1->SetTransform(b2Vec2(x1, y1), 0);
        }
      }
    }

    {
      /** CONTAINER-PENDULUM with DOMINOS nad BALLOON
       *
       */
      // Position and Scalability Factors
      float x = -84 * sec3_scale + sec3_x, y = -20 * sec3_scale + sec3_y;
      float scale = 2 * sec3_scale;
      float weight_scale = 0.1;

      float height = 2 * scale;
      {
        /** Container
         *
         */

        b2Body *b1;
        float thickness = 0.1 * scale;
        float pendulum_length = 5 * scale;
        float density = 2 * weight_scale;
        b1 = container(height, height, thickness, density);
        b1->SetTransform(b2Vec2(x, y), 0);
        rotation_joint(b1, x, y + height / 2 + pendulum_length, 0,
                       height / 2 + pendulum_length);
      }

      float dominos_size = 3 * scale;
      int num = 8;
      {
        /** Dominos
         *
         */

        b2PolygonShape shape;
        shape.SetAsBox(0.1f * scale, dominos_size / 2);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 10.0f * weight_scale;
        fd.friction = 0.1f;

        for (int i = 0; i < num; ++i) {
          b2BodyDef bd;
          bd.type = b2_dynamicBody;
          bd.position.Set(x + height / 2 + 1.0f * (i + 1) * scale, y);
          b2Body *body = m_world->CreateBody(&bd);
          body->CreateFixture(&fd);
        }
      }

      {
        /** Lower-Platform
         *  This is a b2EdgeShape Body that supports the dominos and the
         * anti-gravity ball
         */
        b2Body *b;
        b2EdgeShape shape;
        shape.Set(b2Vec2(height / 2, -dominos_size / 2),
                  b2Vec2(height / 2 + 1.0f * (num)*scale + 6 * scale,
                         -dominos_size / 2));

        b2BodyDef bd;
        bd.position.Set(0, 0);
        b = m_world->CreateBody(&bd);
        b->CreateFixture(&shape, 0.0f);
        b->SetTransform(b2Vec2(x, y), 0.0f);
      }

      float radius = 0.5 * scale;
      ;
      {
        /** Anti-Gravity Ball
         *  This is a dynamic ball which rises up in the air. This has been done
         * using b2Body::SetGravityScale()
         */
        b2Body *body;
        b2BodyDef myBodyDef;
        myBodyDef.type = b2_dynamicBody;

        b2CircleShape circle;
        circle.m_radius = radius;
        circle.m_p.Set(x + height / 2 + 1.0f * (num)*scale + 3 * scale + radius,
                       y - dominos_size / 2 + radius);

        b2FixtureDef myFixture;
        myFixture.shape = &circle;
        myFixture.density = 10 * weight_scale;
        myFixture.restitution = 0;
        myFixture.friction = 0;

        body = m_world->CreateBody(&myBodyDef);
        body->CreateFixture(&myFixture);
        body->SetGravityScale(-1);
      }

      {
        /** Upper-Platform
         *  This is a b2EdgeShape Body that prevents the anti-gravity ball from
         * rising
         */
        b2Body *b;
        b2EdgeShape shape;
        shape.Set(b2Vec2(height / 2 + 1.0f * (num)*scale + 3 * scale,
                         -dominos_size / 2 + radius * 2),
                  b2Vec2(height / 2 + 1.0f * (num)*scale + 6 * scale,
                         -dominos_size / 2 + radius * 2));

        b2BodyDef bd;
        bd.position.Set(0, 0);
        b = m_world->CreateBody(&bd);
        b->CreateFixture(&shape, 0.0f);
        b->SetTransform(b2Vec2(x, y), 0.0f);
      }
    }
  }

  {
    /** Section-1
     *  This Section Contains the top-left part of the simulation.
     *  Different Parts are:
     *  1.
     *  Each part of this section is wrapped in a separate block of code
     *  Each one begins with 'Position and Scalability Factors'
     * (self-explanatory) sec1_x and sec1_y can be used to change the
     * coordinates of the entire Section3. sec1_scale can be used to scale the
     * size of Section3
     */

    float sec1_x = -15, sec1_y = 20;
    float sec1_scale = 0.5;

    { // CODE
      // Spin  Wheel
      {
        float x = 12 * sec1_scale + sec1_x, y = -30 * sec1_scale + sec1_y;
        float scale = 1 * sec1_scale;
        float inner_radius = 3.0 * scale, outer_radius = 6.0 * scale;
        int numSpokes = 8;
        float inc = (360.0) / numSpokes;

        b2BodyDef myBodyDef;
        myBodyDef.type = b2_dynamicBody; // this will be a dynamic body
        myBodyDef.position.Set(x, y);    // a little to the left

        b2Body *dynamicBody1 = m_world->CreateBody(&myBodyDef);
        dynamicBody1->SetFixedRotation(false);
        b2FixtureDef myFixtureDef;
        b2CircleShape circle;
        b2PolygonShape shape;

        {
          circle.m_p.Set(0, 0);
          circle.m_radius = inner_radius;
          myFixtureDef.shape = &circle;
          myFixtureDef.density = 0.0;
          myFixtureDef.restitution = 0;
          dynamicBody1->CreateFixture(&myFixtureDef);
        }

        myFixtureDef.density = 1.0;
        {
          b2ChainShape chain;
          b2Vec2 *vertex = new b2Vec2[numSpokes];
          for (int i = 0; i < numSpokes; ++i) {
            vertex[i].Set(outer_radius * cosf(i * inc * DEGTORAD),
                          outer_radius * sinf(i * inc * DEGTORAD));
          }
          chain.CreateLoop(vertex, numSpokes);
          myFixtureDef.shape = &chain;
          myFixtureDef.density = 1.0;
          myFixtureDef.restitution = 0;
          dynamicBody1->CreateFixture(&myFixtureDef);
        }

        for (int i = 0; i < numSpokes; ++i) {

          // b2PolygonShape p;
          // b2Vec2 vertices[4];
          // vertices[0].Set(inner_radius*cosf(i*inc*DEGTORAD) -
          // 0.001,inner_radius*sinf(i*inc*DEGTORAD) - 0.001);
          // vertices[1].Set(inner_radius*cosf(i*inc*DEGTORAD) +
          // 0.001,inner_radius*sinf(i*inc*DEGTORAD) + 0.001);
          // vertices[2].Set(outer_radius*cosf((i-1)*inc*DEGTORAD) -
          // 0.001,outer_radius*sinf((i-1)*inc*DEGTORAD) - 0.001);
          // vertices[3].Set(outer_radius*cosf((i-1)*inc*DEGTORAD) +
          // 0.001,outer_radius*sinf((i-1)*inc*DEGTORAD) + 0.001);
          // p.Set(vertices,4);

          b2EdgeShape edge;
          edge.Set(b2Vec2(inner_radius * cosf(i * inc * DEGTORAD),
                          inner_radius * sinf(i * inc * DEGTORAD)),
                   b2Vec2(outer_radius * cosf((i - 1) * inc * DEGTORAD),
                          outer_radius * sinf((i - 1) * inc * DEGTORAD)));
          myFixtureDef.shape = &edge;
          dynamicBody1->CreateFixture(&myFixtureDef);
        }

        b2RevoluteJointDef anchorJoint;
        b2BodyDef anchorDef;
        anchorDef.type = b2_staticBody;
        anchorDef.position.Set(x, y);
        b2Body *anchorPoint = m_world->CreateBody(&anchorDef);
        anchorPoint->SetTransform(b2Vec2(x, y), 0);
        anchorJoint.bodyA = dynamicBody1;
        anchorJoint.bodyB = anchorPoint;
        anchorJoint.localAnchorA.Set(0, 0);
        anchorJoint.localAnchorB.Set(0, 0);
        m_world->CreateJoint(&anchorJoint);

        b2Body **ball = new b2Body *[numSpokes];
        dynamicBody1->SetAngularVelocity(-0.75);

        myFixtureDef.density = 2.0;
        for (int i = 0; i < numSpokes; ++i) {

          b2BodyDef myBodyDef;
          myBodyDef.type = b2_dynamicBody; // this will be a dynamic body
          myBodyDef.position.Set(x, y);

          ball[i] = m_world->CreateBody(&myBodyDef);

          circle.m_p.Set(scale * 5.0 * cosf((i * inc - 0.6 * inc) * DEGTORAD),
                         scale * 5.0 * sinf((i * inc - 0.6 * inc) * DEGTORAD));
          circle.m_radius = 0.4 * scale;
          myFixtureDef.shape = &circle;
          ball[i]->CreateFixture(&myFixtureDef);
        }
      }

      { // Sub section
        float sub_sec1_x = 0 + sec1_x, sub_sec1_y = 0 + sec1_y;
        float sub_sec1_scale = 1 * sec1_scale;
        // Top horizontal shelf
        {
          float x = -31.0f * sub_sec1_scale + sub_sec1_x,
                y = 30.0f * sub_sec1_scale + sub_sec1_y;
          float scale = 1 * sub_sec1_scale;
          b2PolygonShape shape;
          shape.SetAsBox(6.0f * scale, 0.25f * scale);

          b2BodyDef bd;
          bd.position.Set(x, y);
          b2Body *ground = m_world->CreateBody(&bd);
          ground->CreateFixture(&shape, 0.0f);
        }

        // Dominos and ball
        {
          float x = -35.0f * sub_sec1_scale + sub_sec1_x,
                y = 31.25f * sub_sec1_scale + sub_sec1_y;
          float scale = 1 * sub_sec1_scale;

          b2PolygonShape shape;
          shape.SetAsBox(0.1f * scale, 1.0f * scale);

          b2FixtureDef fd;
          fd.shape = &shape;
          fd.density = 1000.0f;
          fd.friction = 0.1f;

          for (int i = 0; i < 6; ++i) {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(x + 1.0f * i * scale, y);
            b2Body *body = m_world->CreateBody(&bd);
            body->CreateFixture(&fd);
          }

          b2Body *ball;

          b2CircleShape circle;
          circle.m_radius = 1.0 * scale;

          fd.shape = &circle;
          fd.density = 10.0f;
          fd.friction = 0.0f;
          fd.restitution = 0.0f;

          b2BodyDef ballbd;
          ballbd.type = b2_dynamicBody;
          ballbd.position.Set(x + 7 * scale, y);
          ball = m_world->CreateBody(&ballbd);
          ball->CreateFixture(&fd);
        }

        // The pendulum that knocks the dominos off
        {

          b2Body *b2;
          {
            float x = -36.5f * sub_sec1_scale + sub_sec1_x,
                  y = 28.0f * sub_sec1_scale + sub_sec1_y;
            float scale = 1 * sub_sec1_scale;
            b2PolygonShape shape;
            shape.SetAsBox(0.25f * scale, 1.5f * scale);

            b2BodyDef bd;
            bd.position.Set(x, y);
            b2 = m_world->CreateBody(&bd);
            b2->CreateFixture(&shape, 10.0f);
          }

          b2Body *b4;
          {
            float x = -40.0f * sub_sec1_scale + sub_sec1_x,
                  y = 33.0f * sub_sec1_scale + sub_sec1_y;
            float scale = 1 * sub_sec1_scale;
            b2PolygonShape shape;
            shape.SetAsBox(0.50f * scale, 0.50f * scale);

            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(x, y);
            b4 = m_world->CreateBody(&bd);
            b4->CreateFixture(&shape, 100.0f);
          }

          {
            float x = -37 * sub_sec1_scale + sub_sec1_x,
                  y = 40.0 * sub_sec1_scale + sub_sec1_y;

            b2RevoluteJointDef jd;
            b2Vec2 anchor;
            anchor.Set(x, y);
            jd.Initialize(b2, b4, anchor);
            m_world->CreateJoint(&jd);
          }
        }
      }

      // Circular arc
      {
        float x = -25.5 * sec1_scale + sec1_x, y = 24.80 * sec1_scale + sec1_y;
        float scale = 1 * sec1_scale;
        float radius = 7.5 * scale;
        int Sangle = -160, Eangle = 90;
        float VperDegree = 0.3;
        float inc = 1.0 / VperDegree;
        int size = (Eangle - Sangle) * VperDegree;

        b2BodyDef myBodyDef;
        myBodyDef.type = b2_staticBody; // this will be a dynamic body
        myBodyDef.position.Set(x, y);   // a little to the left

        b2Body *arc = m_world->CreateBody(&myBodyDef);
        arc->SetFixedRotation(false);
        b2FixtureDef myFixtureDef;

        myFixtureDef.density = 1.0;
        {
          b2ChainShape chain;
          b2Vec2 *vertex = new b2Vec2[size];
          for (int i = 0; i < size; ++i) {
            vertex[i].Set(radius * cosf((Sangle + i * inc) * DEGTORAD),
                          radius * sinf((Sangle + i * inc) * DEGTORAD));
          }
          chain.CreateChain(vertex, size);
          myFixtureDef.shape = &chain;
          myFixtureDef.density = 1.0;
          myFixtureDef.restitution = 0.0;
          arc->CreateFixture(&myFixtureDef);
        }
      }

      // The see-saw system at the bottom
      {
        float x = -40 * sec1_scale + sec1_x, y = 9 * sec1_scale + sec1_y;
        float scale = 0.4 * sec1_scale;

        {
          b2Body *b1;
          b2EdgeShape shape;
          shape.Set(b2Vec2(x - 10.0f / 0.4 * scale, y - 0.6f / 0.4 * scale),
                    b2Vec2(x + 10.0f / 0.4 * scale, y - 0.6f / 0.4 * scale));
          b2BodyDef bd;
          b1 = m_world->CreateBody(&bd);
          b1->CreateFixture(&shape, 0.0f);
        }

        // The triangle wedge
        b2Body *sbody;
        b2PolygonShape poly;
        b2Vec2 vertices[3];
        vertices[0].Set(-1 * scale, -1.5 * scale);
        vertices[1].Set(1 * scale, -1.5 * scale);
        vertices[2].Set(0 * scale, 0 * scale);
        poly.Set(vertices, 3);
        b2FixtureDef wedgefd;
        wedgefd.shape = &poly;
        wedgefd.density = 10.0f;
        wedgefd.friction = 0.0f;
        wedgefd.restitution = 0.0f;
        b2BodyDef wedgebd;
        wedgebd.position.Set(x, y);
        sbody = m_world->CreateBody(&wedgebd);
        sbody->CreateFixture(&wedgefd);

        // The plank on top of the wedge
        b2PolygonShape shape;
        shape.SetAsBox(15.0f * scale, 0.2f * scale);
        b2BodyDef bd2;
        bd2.position.Set(x, y);
        bd2.type = b2_dynamicBody;
        b2Body *body = m_world->CreateBody(&bd2);
        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = 1.0f;
        fd2->shape = new b2PolygonShape;
        fd2->shape = &shape;
        body->CreateFixture(fd2);

        b2RevoluteJointDef jd;
        b2Vec2 anchor;
        anchor.Set(x, y);
        jd.Initialize(sbody, body, anchor);
        m_world->CreateJoint(&jd);

        // The light box on the right side of the see-saw
        b2PolygonShape shape2;
        shape2.SetAsBox(2.0f * scale, 2.0f * scale);
        b2BodyDef bd3;
        bd3.position.Set(x - 5.0 / 0.4 * scale,
                         y + 0.3 / 0.4 * scale + 2.0 / 0.4 * scale);
        bd3.type = b2_dynamicBody;
        b2Body *body3 = m_world->CreateBody(&bd3);
        b2FixtureDef *fd3 = new b2FixtureDef;
        fd3->density = 1.0f;
        fd3->friction = 0.1f;
        fd3->shape = new b2PolygonShape;
        fd3->shape = &shape2;
        body3->CreateFixture(fd3);
      }
    }
  }
}

b2Body *dominos_t::container(
    float height, float width, float thickness = 0.02,
    float density = 10.0) { // Returns an Upward open container at 0,0
  /** dominos_t::container returns a upward-open "container" given height,
   * width, thickness and density Effectively, it returns a body with 3
   * fixtures. Each fixture has a b2PolygonShape shape which was defined using
   * b2PolygonShape::SetAsBox() The three fixtures correspond to the left, right
   * and bottom walls of the container. The local origin of the body lies at the
   * center of the bottom fixture.
   */
  b2BodyDef *bd = new b2BodyDef;
  bd->type = b2_dynamicBody;
  bd->position.Set(0, 0);

  // The open box
  /** For each fixture, friction = 0.5; restitution = 0;
   */
  b2FixtureDef *fd1 = new b2FixtureDef;
  fd1->density = density;
  fd1->friction = 0.5;
  fd1->restitution = 0.f;
  fd1->shape = new b2PolygonShape;
  b2PolygonShape bs1;
  bs1.SetAsBox(width / 2, thickness / 2, b2Vec2(0.f, 0.f), 0);
  fd1->shape = &bs1;
  b2FixtureDef *fd2 = new b2FixtureDef;
  fd2->density = density;
  fd2->friction = 0.5;
  fd2->restitution = 0.f;
  fd2->shape = new b2PolygonShape;
  b2PolygonShape bs2;
  bs2.SetAsBox(thickness / 2, height / 2,
               b2Vec2(width / 2 - thickness / 2, height / 2), 0);
  fd2->shape = &bs2;
  b2FixtureDef *fd3 = new b2FixtureDef;
  fd3->density = density;
  fd3->friction = 0.5;
  fd3->restitution = 0.f;
  fd3->shape = new b2PolygonShape;
  b2PolygonShape bs3;
  bs3.SetAsBox(thickness / 2, height / 2,
               b2Vec2(thickness / 2 - width / 2, height / 2), 0);
  fd3->shape = &bs3;

  b2Body *box1 = m_world->CreateBody(bd);
  box1->CreateFixture(fd1);
  box1->CreateFixture(fd2);
  box1->CreateFixture(fd3);
  return box1;
}

void dominos_t::fill_fluid(float x, float y, float height, float width,
                           float radius = 0.2, float density = 0.1) {
  /** dominos_t::fill_fluid fills the rectangular region marked by (x,y) - (x +
   * width, y + height) with "balls" Each ball is a b2Body defined using a
   * Fixture having shape = b2CircleShape (having m_radius = 'radius'); density
   * = 'density'; friction = 0; restitution = 0.1; The balls are positioned in a
   * rectangular fashion by using b2BodyDef::position.Set() in nested For loops
   */
  b2Body *drop;

  b2CircleShape circle;
  circle.m_radius = radius;

  b2FixtureDef ballfd;
  ballfd.shape = &circle;
  ballfd.density = density;
  ballfd.friction = 0.0f;
  ballfd.restitution = 0.1f;

  for (float i = x + radius; i <= x + width - radius; i += 2 * radius) {
    for (float j = y + radius; j <= y + height - radius; j += 2 * radius) {
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(i, j);
      drop = m_world->CreateBody(&ballbd);
      drop->CreateFixture(&ballfd);
    }
  }
  return;
}

void dominos_t::rotation_joint(b2Body *body, float fulcrum_x, float fulcrum_y,
                               float body_x, float body_y) {
  /** dominos_t::rotation_joint hinges b2Body* body on world -axis at
   * (fulcrum_x, fulcrum_y) by local_position (body_x, body_y). This is done by
   * creating a b2Body* anchorPoint at  (fulcrum_x, fulcrum_y) and connecting
   * b2Body* body to it using dominos_t::two_body_rotation_joint.
   */
  b2BodyDef anchorDef;
  anchorDef.type = b2_staticBody;
  anchorDef.position.Set(0, 0);
  b2Body *anchorPoint = m_world->CreateBody(&anchorDef);

  anchorPoint->SetTransform(b2Vec2(fulcrum_x, fulcrum_y), 0);
  b2RevoluteJointDef anchorJoint =
      two_body_rotation_joint(body, anchorPoint, body_x, body_y, 0, 0);
  m_world->CreateJoint(&anchorJoint);
  return;
}

b2RevoluteJointDef
dominos_t::two_body_rotation_joint(b2Body *body_a, b2Body *body_b,
                                   float body_a_x, float body_a_y,
                                   float body_b_x, float body_b_y) {
  /** dominos_t::two_body_rotation_joint returns a b2RevoluteJointDef which
   * defines a connection between b2Body*body_a and b2Body*body_b This is done
   * by defining local anchors on body_a and body_b at (body_a_x,  body_a_y) and
   * (body_b_x, body_b_y) respectively
   */
  b2RevoluteJointDef jointDef;
  jointDef.bodyA = body_a;
  jointDef.bodyB = body_b;
  jointDef.localAnchorA.Set(body_a_x, body_a_y);
  jointDef.localAnchorB.Set(body_b_x, body_b_y);

  /// collideConnected has been set to false to prevent unwanted interaction
  /// between the two bodies.
  jointDef.collideConnected = false;
  return jointDef;
}

void dominos_t::create_pulleyJoint(b2Body *body_a, b2Body *body_b,
                                   float body_a_x, float body_a_y,
                                   float anchor_a_x, float anchor_a_y,
                                   float body_b_x, float body_b_y,
                                   float anchor_b_x, float anchor_b_y) {
  /** dominos_t::create_pulleyJoint creates a pulley joint between b2Body*body_a
   * and b2Body*body_b at (body_a_x,  body_a_y) and (body_b_x, body_b_y)
   * respectively using anchors  (anchor_a_x,  anchor_a_y) and (anchor_b_x,
   * anchor_b_y) respectively. Note that ALL Parameters should be in world axis
   * (not local) A b2PulleyJointDef is first defined and then initialized. (The
   * ratio is ser to 1)
   */

  b2PulleyJointDef *myjoint = new b2PulleyJointDef();
  b2Vec2 worldAnchorOnBody1(body_a_x,
                            body_a_y); // Anchor point on body 1 in world axis
  b2Vec2 worldAnchorOnBody2(body_b_x,
                            body_b_y); // Anchor point on body 2 in world axis
  b2Vec2 worldAnchorGround1(
      anchor_a_x, anchor_a_y); // Anchor point for ground 1 in world axis
  b2Vec2 worldAnchorGround2(
      anchor_b_x, anchor_b_y); // Anchor point for ground 2 in world axis
  float32 ratio = 1.0f;        // Define ratio
  myjoint->Initialize(body_a, body_b, worldAnchorGround1, worldAnchorGround2,
                      body_a->GetWorldCenter(), body_b->GetWorldCenter(),
                      ratio);
  m_world->CreateJoint(myjoint);
}

sim_t *sim = new sim_t("Dominos", dominos_t::create);
} // namespace cs251
