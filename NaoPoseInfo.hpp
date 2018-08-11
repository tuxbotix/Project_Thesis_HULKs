#pragma once

#include <Tools/Kinematics/Com.h>
#include <Tools/Kinematics/ForwardKinematics.h>
#include <Tools/Kinematics/InverseKinematics.h>
#include <Tools/Kinematics/KinematicMatrix.h>

enum SUPPORT_FOOT
{
    SF_RIGHT = -1,
    SF_DOUBLE = 0,
    SF_LEFT = 1,
    SF_NONE = 2
};
enum PARAMS
{
    P_SUPPORT_FOOT,
    P_HEAD_YAW,
    P_HEAD_PITCH,
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
        return Vector3<T>(params[P_TORSO_ROT_X], params[P_TORSO_POS_Y], params[P_TORSO_ROT_Z]);
    }
    template <typename T>
    static Vector3<T> getOtherFootPosVFromParams(const std::vector<T> &params)
    {
        return Vector3<T>(params[P_O_FOOT_POS_X], params[P_TORSO_POS_Y], params[P_O_FOOT_POS_Z]);
    }
    template <typename T>
    static Vector3<T> getOtherFootRotVFromParams(const std::vector<T> &params)
    {
        return Vector3<T>(params[P_O_FOOT_ROT_X], params[P_TORSO_POS_Y], params[P_O_FOOT_ROT_Z]);
    }
    template <typename T>
    static SUPPORT_FOOT getSupportFootTorsoPosVFromParams(const std::vector<T> &params)
    {
        return static_cast<SUPPORT_FOOT>(params[P_SUPPORT_FOOT]);
    }
};
/**
 * All angles recorded in degrees
 */
// template <typename T>
// class PoseDefinition
// {
//     Vector3<T> torsoPosV;
//     Vector3<T> torsoRotV;
//     Vector3<T> otherFootPosV;
//     Vector3<T> otherFootRotV;
//     SUPPORT_FOOT supportFoot;

//     SUPPORT_FOOT &operator[](const PARAMS &index)
//     {
//         if (index != PARAMS::P_SUPPORT_FOOT)
//         {
//             std::cout << "ERROR ERROR Support foot .." << std::endl;
//         }
//         return supportFoot;
//     }
//     const SUPPORT_FOOT &operator[](const PARAMS &index) const
//     {
//         if (index != PARAMS::P_SUPPORT_FOOT)
//         {
//             std::cout << "ERROR ERROR Support foot .." << std::endl;
//         }
//         return supportFoot;
//     }
//     bool T &operator[](const PARAMS &index)
//     {
//         if (index >= PARAMS::P_MAX || index < PARAMS::P_TORSO_POS_X)
//         {
//             std::cout << "ERROR ERROR Val Idx .." << std::endl;
//         }
//         int out = static_cast<int>(index) / 3;
//         int remains = static_cast<int>(index) % 3;
//         switch (out)
//         {
//         case 0:
//         {
//             return torsoPosV[remains];
//             break;
//         }
//         case 1:
//         {
//             return torsoRotV[remains];
//             break;
//         }
//         case 2:
//         {
//             return otherFootPosV[remains];
//             break;
//         }
//         case 3:
//         {
//             return otherFootRotV[remains];
//             break;
//         }
//         case 4:
//         {
//             return this->[index] = static_cast<SUPPORT_FOOT>(std::round(val)));
//             break;
//         }
//         }
//     }

//     const bool T &operator[](const PARAMS &index) const
//     {
//         if (index >= PARAMS::P_MAX || index < PARAMS::P_TORSO_POS_X)
//         {
//             return false;
//         }
//         int out = static_cast<int>(index) / 3;
//         int remains = static_cast<int>(index) % 3;
//         switch (out)
//         {
//         case 0:
//         {
//             return torsoPosV[remains];
//             break;
//         }
//         case 1:
//         {
//             return torsoRotV[remains];
//             break;
//         }
//         case 2:
//         {
//             return otherFootPosV[remains];
//             break;
//         }
//         case 3:
//         {
//             return otherFootRotV[remains];
//             break;
//         }
//         case 4:
//         {
//             return this->[index] = static_cast<SUPPORT_FOOT>(std::round(val)));
//             break;
//         }
//         }
//     }
// };