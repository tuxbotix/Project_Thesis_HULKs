#include "NaoCollision.hpp"
#include <algorithm>

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

void Capsule::update(const Vector3f &begin, const Vector3f &end)
{
    length = (begin - end).norm();
    this->begin = begin;
    this->end = end;
}

float Capsule::getLength() const
{
    return length;
}

bool Capsule::pointInCapsuleCore(const Vector3f &pt) const
{
    return (getLength() - (begin - pt).norm() - (end - pt).norm()) < __FLT_EPSILON__;
}

bool Capsule::isColliding(const Sphere &spr)
{
    const Vector3f lineUnitDir = (end - begin).normalized();
    const Vector3f projectedOrigin = spr.origin.dot(lineUnitDir) * lineUnitDir;
    const float radiusTotal = spr.radius + radius;
    // projection inside the capsule.
    if (pointInCapsuleCore(projectedOrigin))
    {
        // if distance is smaller than the two radiuses, we collide!
        return (projectedOrigin - spr.origin).norm() < radiusTotal;
    }
    else // projection outside.
    {
        return std::min((spr.origin - begin).norm(), (spr.origin - end).norm()) < radiusTotal;
    }
}

bool Capsule::isColliding(const Capsule &caps)
{
    std::pair<Vector3f, Vector3f> closestPts;
    const float radiusTotal = caps.radius + radius;

    float dist = GeomUtils::closestPtRayToRay(begin, end, caps.begin, caps.end, closestPts);
    if (dist < __FLT_EPSILON__)
    {
        /// if P(sc) inside caps1
        bool intersectInCaps1 = pointInCapsuleCore(closestPts.first);
        bool intersectInCaps2 = caps.pointInCapsuleCore(closestPts.second);

        // both intersections are inside each capsule -> in 2D plane this is intersection
        if (intersectInCaps1 && intersectInCaps2)
        {
            return true;
        }
        // for caps 1
        if (intersectInCaps1)
        {
            float minDist = std::min(GeomUtils::distPointToLine(begin, end, caps.begin),
                                     GeomUtils::distPointToLine(begin, end, caps.end));
            return minDist < radiusTotal;
        }
        else if (intersectInCaps2) // caps 2
        {
            float minDist = std::min(GeomUtils::distPointToLine(caps.begin, caps.end, begin),
                                     GeomUtils::distPointToLine(caps.begin, caps.end, end));
            return minDist < radiusTotal;
        }
        else // intersection is outside both
        {
            float minDist1 = std::min((begin - caps.begin).norm(), (end - caps.end).norm());
            float minDist2 = std::min((begin - caps.end).norm(), (end - caps.begin).norm());
            return std::min(minDist1, minDist2) < radiusTotal;
        }
        return false;
    }
    else
    {
        std::cout << "Not intersecting? Dist: " << dist << std::endl;
        return false;
    }
}

void CollisionModel::updateModel(rawPoseT &jointAngles)
{
    std::vector<KinematicMatrix> lLegJointPoses = ForwardKinematics::getLLeg(jointAngles);
    std::vector<KinematicMatrix> rLegJointPoses = ForwardKinematics::getRLeg(jointAngles);
    lThigh.update(lLegJointPoses[2].posV, lLegJointPoses[3].posV);
    rThigh.update(rLegJointPoses[2].posV, rLegJointPoses[3].posV);
    lTibia.update(lLegJointPoses[3].posV, lLegJointPoses[4].posV);
    rTibia.update(rLegJointPoses[3].posV, rLegJointPoses[4].posV);

    Vector3f innerEdgeInitFront(38, 0, 20);
    Vector3f innerEdgeInitBack(-80, 0, 20);
    Vector3f leftShift(0, -20, 0);

    lFootInnerEdge.update(lLegJointPoses[6] * (innerEdgeInitFront + leftShift),
                          lLegJointPoses[6] * (innerEdgeInitBack + leftShift));
    rFootInnerEdge.update(rLegJointPoses[6] * (innerEdgeInitFront - leftShift),
                          rLegJointPoses[6] * (innerEdgeInitBack - leftShift));
}

bool CollisionModel::isLegCollission()
{
    // most common cases
    if (lFootInnerEdge.isColliding(rFootInnerEdge))
    {
        return true;
    }
    if (lTibia.isColliding(rTibia))
    {
        return true;
    }
    if (lThigh.isColliding(rThigh))
    {
        return true;
    }
    // a bit less common
    if (lFootInnerEdge.isColliding(rTibia))
    {
        return true;
    }
    if (lTibia.isColliding(rFootInnerEdge))
    {
        return true;
    }
    if (lTibia.isColliding(rThigh))
    {
        return true;
    }
    if (lThigh.isColliding(rTibia))
    {
        return true;
    }
    // quite uncommon
    if (lFootInnerEdge.isColliding(rThigh))
    {
        return true;
    }
    if (lThigh.isColliding(rFootInnerEdge))
    {
        return true;
    }
    return false;
}

bool CollisionModel::updateAndTest(rawPoseT &jointAngles)
{
    updateModel(jointAngles);
    return isLegCollission();
}

} // namespace Collision
