#pragma once

#include <Tools/Kinematics/Com.h>
#include <Tools/Kinematics/ForwardKinematics.h>
#include <Tools/Kinematics/InverseKinematics.h>
#include <Tools/Kinematics/KinematicMatrix.h>

typedef float angleT;
typedef angleT dataT;

typedef std::vector<angleT> jointAnglesT;
// const dataT deltaTheta = 1; // 1 deg +-
typedef jointAnglesT rawAnglesT;
/// More typedefs below..

enum SUPPORT_FOOT
{
    SF_RIGHT = -1,
    SF_DOUBLE = 0,
    SF_LEFT = 1,
    SF_NONE = 2
};
enum HEAD_PARAMS
{
    P_HEAD_YAW,
    P_HEAD_PITCH,
    P_HEAD_YAW_PITCH_MAX
};
enum PARAMS
{
    P_SUPPORT_FOOT,
    P_TORSO_POS_X,
    P_TORSO_POS_Y,
    P_TORSO_POS_Z,
    P_TORSO_ROT_X,
    P_TORSO_ROT_Y,
    P_TORSO_ROT_Z,
    P_O_FOOT_POS_X,
    P_O_FOOT_POS_Y,
    P_O_FOOT_POS_Z,
    P_O_FOOT_ROT_X,
    P_O_FOOT_ROT_Y,
    P_O_FOOT_ROT_Z,
    P_MAX
};
typedef PARAMS paramNameT;

class PoseUtils
{
  public:
    template <typename T>
    static Vector3<T> getTorsoPosVFromParams(const std::vector<T> &params)
    {
        return Vector3<T>(params[P_TORSO_POS_X], params[P_TORSO_POS_Y], params[P_TORSO_POS_Z]);
    }
    template <typename T>
    static Vector3<T> getTorsoRotVFromParams(const std::vector<T> &params)
    {
        return Vector3<T>(params[P_TORSO_ROT_X], params[P_TORSO_ROT_Y], params[P_TORSO_ROT_Z]);
    }
    template <typename T>
    static Vector3<T> getOtherFootPosVFromParams(const std::vector<T> &params)
    {
        return Vector3<T>(params[P_O_FOOT_POS_X], params[P_O_FOOT_POS_Y], params[P_O_FOOT_POS_Z]);
    }
    template <typename T>
    static Vector3<T> getOtherFootRotVFromParams(const std::vector<T> &params)
    {
        return Vector3<T>(params[P_O_FOOT_ROT_X], params[P_O_FOOT_ROT_Y], params[P_O_FOOT_ROT_Z]);
    }
    template <typename T>
    static SUPPORT_FOOT getSupportFootTorsoPosVFromParams(const std::vector<T> &params)
    {
        return static_cast<SUPPORT_FOOT>(params[P_SUPPORT_FOOT]);
    }

    template <typename T>
    static std::vector<Vector3<T>> getVector3List(const Vector3<T> &minVec, const Vector3<T> &maxVec, const Vector3<T> &iterator)
    {
        Vector3<T> diff = maxVec - minVec;
        std::vector<Vector3<T>> outputList;
        for (T i = minVec.x(); i <= maxVec.x(); i += iterator.x())
        {
            if (std::abs(i) < iterator.x())
            {
                i = 0;
            }
            for (T j = minVec.y(); j <= maxVec.y(); j += iterator.y())
            {
                if (std::abs(j) < iterator.y())
                {
                    j = 0;
                }
                for (T k = minVec.z(); k <= maxVec.z(); k += iterator.z())
                {
                    if (std::abs(k) < iterator.z())
                    {
                        k = 0;
                    }
                    outputList.emplace_back(i, j, k);
                    if (std::abs(diff.z()) < __FLT_EPSILON__)
                    {
                        break;
                    }
                }
                if (std::abs(diff.y()) < __FLT_EPSILON__)
                {
                    break;
                }
            }
            if (std::abs(diff.x()) < __FLT_EPSILON__)
            {
                break;
            }
        }

        return outputList;
    }
};

struct HeadYawPitch
{
    angleT yaw;
    angleT pitch;
    HeadYawPitch(const angleT &y, const angleT &p) : yaw(y), pitch(p)
    {
    }
};
/**
 * All angles recorded in degrees
 */
template <typename T = angleT>
class NaoPose
{
  public:
    SUPPORT_FOOT supportFoot;
    HeadYawPitch headYawPitch;
    Vector3<T> torsoPosV;
    Vector3<T> torsoRotV;
    Vector3<T> otherFootPosV;
    Vector3<T> otherFootRotV;

    NaoPose(SUPPORT_FOOT sf, HeadYawPitch hyp, Vector3<T> tPosV, Vector3<T> tRotV, Vector3<T> OFPosV, Vector3<T> OFRotV)
        : supportFoot(sf), headYawPitch(hyp), torsoPosV(tPosV), torsoRotV(tRotV), otherFootPosV(OFPosV),
          otherFootRotV(OFRotV)
    {
    }
    friend std::ostream &operator<<(std::ostream &out, const NaoPose &p)
    {
        out << p.supportFoot << " " << p.headYawPitch.yaw << " " << p.headYawPitch.pitch << " "
            << p.torsoPosV.x() << " " << p.torsoPosV.y() << " " << p.torsoPosV.z() << " "
            << p.torsoRotV.x() << " " << p.torsoRotV.y() << " " << p.torsoRotV.z() << " "
            << p.otherFootPosV.x() << " " << p.otherFootPosV.y() << " " << p.otherFootPosV.z()
            << " " << p.otherFootRotV.x() << " " << p.otherFootRotV.y() << " " << p.otherFootRotV.z();
        return out;
    }
    friend std::istream &operator>>(std::istream &in, NaoPose &p)
    {
        int i;
        in >> i >> p.headYawPitch.yaw >> p.headYawPitch.pitch >>
            p.torsoPosV.x() >> p.torsoPosV.y() >> p.torsoPosV.z() >>
            p.torsoRotV.x() >> p.torsoRotV.y() >> p.torsoRotV.z() >>
            p.otherFootPosV.x() >> p.otherFootPosV.y() >> p.otherFootPosV.z() >>
            p.otherFootRotV.x() >> p.otherFootRotV.y() >> p.otherFootRotV.z();
        p.supportFoot = static_cast<SUPPORT_FOOT>(i);
        return in;
    }
};

typedef std::vector<NaoPose<dataT>> poseListT;
typedef std::vector<angleT> rawPoseT;
typedef std::vector<rawPoseT> rawPoseListT;
typedef std::vector<Vector3<dataT>> vector3ListT;
