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

enum SENSOR_NAME
{
    TOP_CAMERA,
    BOTTOM_CAMERA
    // Future : 3d measurements, etc on lifted foot, etc.
};

class Sensor
{
  public:
    SENSOR_NAME name;

    const static std::vector<JOINTS::JOINT> CAM_OBS_L_SUP_FOOT;
    const static std::vector<JOINTS::JOINT> CAM_OBS_R_SUP_FOOT;
};

const std::vector<JOINTS::JOINT> Sensor::CAM_OBS_L_SUP_FOOT = {
    JOINTS::JOINT::L_HIP_YAW_PITCH,
    JOINTS::JOINT::L_HIP_ROLL,
    JOINTS::JOINT::L_HIP_PITCH,
    JOINTS::JOINT::L_KNEE_PITCH,
    JOINTS::JOINT::L_ANKLE_PITCH,
    JOINTS::JOINT::L_ANKLE_ROLL};
const std::vector<JOINTS::JOINT> Sensor::CAM_OBS_R_SUP_FOOT = {
    JOINTS::JOINT::R_HIP_YAW_PITCH,
    JOINTS::JOINT::R_HIP_ROLL,
    JOINTS::JOINT::R_HIP_PITCH,
    JOINTS::JOINT::R_KNEE_PITCH,
    JOINTS::JOINT::R_ANKLE_PITCH,
    JOINTS::JOINT::R_ANKLE_ROLL};

/**
 * IMPORTANT : The type T MUST have implemented a Zero() method (static)
 */
template <typename T = Vector3f>
class PoseSensitivity
{
    /**
     * This class store sensitivity per joint (except arms and head) and also by which sensor.
     */
    SENSOR_NAME sensorName;
    std::vector<T> sensitivities;
    std::vector<bool> observationMask;

  public:
    PoseSensitivity(const SENSOR_NAME &sn)
        : sensorName(sn),
          sensitivities(std::vector<T>(JOINTS::JOINT::JOINTS_MAX, T::Zero())),
          observationMask(std::vector<bool>(JOINTS::JOINT::JOINTS_MAX))
    {
    }
    SENSOR_NAME getSensorName()
    {
        return sensorName;
    }
    void setSensitivity(const JOINTS::JOINT &joint, const T &val, const bool &obs)
    {
        sensitivities[joint] = val;
        observationMask[joint] = obs;
    }
    void getSensitivity(const JOINTS::JOINT &joint, T &val, bool &obs)
    {
        obs = observationMask[joint];
        if (obs)
        {
            val = sensitivities[joint];
        }
        else
        {
            val = T::Zero();
        }
    }
};

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
  private:
    static const std::string className;
    bool valid; // set true once valid data is loaded.

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
    NaoPose() : supportFoot(SUPPORT_FOOT::SF_NONE), headYawPitch(0, 0), torsoPosV(), torsoRotV(), otherFootPosV(),
                otherFootRotV()
    {
    }
    friend std::ostream &operator<<(std::ostream &out, const NaoPose &p)
    {
        out << className << " " << p.supportFoot << " " << p.headYawPitch.yaw << " " << p.headYawPitch.pitch << " "
            << p.torsoPosV.x() << " " << p.torsoPosV.y() << " " << p.torsoPosV.z() << " "
            << p.torsoRotV.x() << " " << p.torsoRotV.y() << " " << p.torsoRotV.z() << " "
            << p.otherFootPosV.x() << " " << p.otherFootPosV.y() << " " << p.otherFootPosV.z()
            << " " << p.otherFootRotV.x() << " " << p.otherFootRotV.y() << " " << p.otherFootRotV.z();
        return out;
    }
    friend std::istream &operator>>(std::istream &in, NaoPose &p)
    {
        p.valid = false;
        std::string name;
        int i;
        in >> name;
        if (name.compare(className) == 0)
        {
            in >> i >> p.headYawPitch.yaw >> p.headYawPitch.pitch >>
                p.torsoPosV.x() >> p.torsoPosV.y() >> p.torsoPosV.z() >>
                p.torsoRotV.x() >> p.torsoRotV.y() >> p.torsoRotV.z() >>
                p.otherFootPosV.x() >> p.otherFootPosV.y() >> p.otherFootPosV.z() >>
                p.otherFootRotV.x() >> p.otherFootRotV.y() >> p.otherFootRotV.z();
            p.supportFoot = static_cast<SUPPORT_FOOT>(i);
            p.valid = true;
        }
        else
        {
            throw "Cannot deserialize this stream to NaoPose, Header mismatch!!!";
        }

        return in;
    }
    bool isGood() { return valid; }
};

template <typename T>
const std::string NaoPose<T>::className = "NaoPose";

template <typename T = angleT>
class NaoPoseAndRawAngles
{
  private:
    static const size_t maxJoints = static_cast<size_t>(JOINTS::JOINT::JOINTS_MAX);
    static const std::string className;
    bool valid; // set true once valid data is loaded.
  public:
    NaoPose<T> pose;
    std::vector<T> angles;

  public:
    NaoPoseAndRawAngles(const NaoPose<T> &pose, const std::vector<T> &jointAngles)
        : pose(pose), angles(jointAngles)
    {
        valid = true;
    }
    NaoPoseAndRawAngles() : pose(), angles(maxJoints, 0.0f)
    {
        valid = false;
    }
    // TODO make these "safe"
    friend std::ostream &operator<<(std::ostream &out, const NaoPoseAndRawAngles &p)
    {
        if (p.valid)
        {
            out << className << " " << p.pose; // pose is streamable.
            for (size_t i = 0; i < maxJoints; i++)
            {
                out << " " << p.angles[i];
            }
        }
        return out;
    }
    friend std::istream &operator>>(std::istream &in, NaoPoseAndRawAngles &p)
    {

        p.valid = false;
        std::string name;
        in >> name;
        if (name.compare(className) == 0)
        {
            in >> p.pose; // pose is streamable.
            for (size_t i = 0; i < maxJoints; i++)
            {
                if (in.good())
                {
                    in >> p.angles[i];
                }
                else
                {
                    std::cerr << "Input stream \"not good\", can't extract joint angle." << std::endl;
                    break;
                }
            }
            p.valid = true;
        }
        else
        {
            std::cerr << "NaoPoseAndRawAngle deserialize issue." << std::endl;
            throw "Cannot deserialize this stream to NaoPoseAndRawAngles, Header mismatch!!!";
        }
        return in;
    }
    bool isGood() { return valid; }
};
template <typename T>
const std::string NaoPoseAndRawAngles<T>::className = "NaoPoseAndRawAngles";

typedef NaoPoseAndRawAngles<dataT> poseAndRawAngleT;
typedef std::vector<NaoPoseAndRawAngles<dataT>> poseAndRawAngleListT;
typedef NaoPose<dataT> poseT;

typedef std::vector<poseT> poseListT;
typedef std::vector<angleT> rawPoseT;
typedef std::vector<rawPoseT> rawPoseListT;
typedef std::vector<Vector3<dataT>> vector3ListT;
