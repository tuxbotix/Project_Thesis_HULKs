#pragma once

#include "NaoPoseInfo.hpp"
#include "GeomUtils.hpp"

/**
 * Self-collision avoidance [ simple]
 * 
 * Inspiration:
 * http://doc.aldebaran.com/2-8/naoqi/motion/reflexes-collision-avoidance.html?highlight=collision
 * 
 * Also:
 * 
 */

namespace Collision
{
enum SHAPETYPE
{
    SPHERE,
    CAPSULE
};

class Sphere;
class Capsule
{
    const float radius;
    float length;
    Vector3f begin;
    Vector3f end;

  public:
    Capsule(const float &radius) : radius(radius),
                                   length(0),
                                   begin(Vector3f::Zero()),
                                   end(Vector3f::Zero())
    {
    }
    Capsule(const float &radius, const Vector3f &begin, const Vector3f &end) : radius(radius),
                                                                               length((begin - end).norm()),
                                                                               begin(begin),
                                                                               end(end)
    {
    }

    void update(const Vector3f &begin, const Vector3f &end);

    float getLength() const;

    bool pointInCapsuleCore(const Vector3f &pt) const;

    bool isColliding(const Sphere &spr);

    bool isColliding(const Capsule &caps);
};

class Sphere
{
  public:
    const float radius;
    Vector3f origin;
    Sphere(const float &radius, const Vector3f &origin) : radius(radius),
                                                          origin(origin)
    {
    }
    Sphere(const float &radius) : radius(radius),
                                  origin(Vector3f::Zero())
    {
    }

    void update(const Vector3f &origin)
    {
        this->origin = origin;
    }
};

class CollisionModel
{
    Capsule lThigh;
    Capsule rThigh;
    Capsule lTibia;
    Capsule rTibia;
    Capsule lFootInnerEdge;
    Capsule rFootInnerEdge;

  public:
    /**
     out[0] = LHipYawPitch2Torso;
     out[1] = LHipRoll2Torso;
     out[2] = LHipPitch2Torso;
     out[3] = LKneePitch2Torso;
     out[4] = LAnklePitch2Torso;
     out[5] = LAnkleRoll2Torso;
     out[6] = LFoot2Torso;
     */
    CollisionModel() : lThigh(40),
                       rThigh(40),
                       lTibia(40),
                       rTibia(40),
                       lFootInnerEdge(20),
                       rFootInnerEdge(20)
    {
    }

    void updateModel(rawPoseT &jointAngles);

    bool isLegCollission();

    bool updateAndTest(rawPoseT &jointAngles);
};

} // namespace Collision
