// #include "NaoPoseInfo.hpp"
// #include <Tools/Kinematics/Com.h>
// #include <Tools/Kinematics/ForwardKinematics.h>
// #include <Tools/Kinematics/InverseKinematics.h>
// #include <Tools/Kinematics/KinematicMatrix.h>

// const std::vector<JOINTS::JOINT> Sensor::CAM_OBS_L_SUP_FOOT = {
//     JOINTS::JOINT::L_HIP_YAW_PITCH,
//     JOINTS::JOINT::L_HIP_ROLL,
//     JOINTS::JOINT::L_HIP_PITCH,
//     JOINTS::JOINT::L_KNEE_PITCH,
//     JOINTS::JOINT::L_ANKLE_PITCH,
//     JOINTS::JOINT::L_ANKLE_ROLL};
// const std::vector<JOINTS::JOINT> Sensor::CAM_OBS_R_SUP_FOOT = {
//     JOINTS::JOINT::R_HIP_YAW_PITCH,
//     JOINTS::JOINT::R_HIP_ROLL,
//     JOINTS::JOINT::R_HIP_PITCH,
//     JOINTS::JOINT::R_KNEE_PITCH,
//     JOINTS::JOINT::R_ANKLE_PITCH,
//     JOINTS::JOINT::R_ANKLE_ROLL};

// template <typename T>
// static std::vector<Vector3<T>> PoseUtils::getVector3List(const Vector3<T> &minVec, const Vector3<T> &maxVec, const Vector3<T> &iterator)
// {
//     Vector3<T> diff = maxVec - minVec;
//     std::vector<Vector3<T>> outputList;
//     for (T i = minVec.x(); i <= maxVec.x(); i += iterator.x())
//     {
//         if (std::abs(i) < iterator.x())
//         {
//             i = 0;
//         }
//         for (T j = minVec.y(); j <= maxVec.y(); j += iterator.y())
//         {
//             if (std::abs(j) < iterator.y())
//             {
//                 j = 0;
//             }
//             for (T k = minVec.z(); k <= maxVec.z(); k += iterator.z())
//             {
//                 if (std::abs(k) < iterator.z())
//                 {
//                     k = 0;
//                 }
//                 outputList.emplace_back(i, j, k);
//                 if (std::abs(diff.z()) < __FLT_EPSILON__)
//                 {
//                     break;
//                 }
//             }
//             if (std::abs(diff.y()) < __FLT_EPSILON__)
//             {
//                 break;
//             }
//         }
//         if (std::abs(diff.x()) < __FLT_EPSILON__)
//         {
//             break;
//         }
//     }
//     return outputList;
// }

// struct HeadYawPitch
// {
//     angleT yaw;
//     angleT pitch;
//     HeadYawPitch(const angleT &y, const angleT &p) : yaw(y), pitch(p)
//     {
//     }
// };
// /**
//  * All angles recorded in degrees
//  */

// friend std::ostream &NaoPose::operator<<(std::ostream &out, const NaoPose &p)
// {
//     out << p.supportFoot << " " << p.headYawPitch.yaw << " " << p.headYawPitch.pitch << " "
//         << p.torsoPosV.x() << " " << p.torsoPosV.y() << " " << p.torsoPosV.z() << " "
//         << p.torsoRotV.x() << " " << p.torsoRotV.y() << " " << p.torsoRotV.z() << " "
//         << p.otherFootPosV.x() << " " << p.otherFootPosV.y() << " " << p.otherFootPosV.z()
//         << " " << p.otherFootRotV.x() << " " << p.otherFootRotV.y() << " " << p.otherFootRotV.z();
//     return out;
// }
// friend std::istream &NaoPose::operator>>(std::istream &in, NaoPose &p)
// {
//     int i;
//     in >> i >> p.headYawPitch.yaw >> p.headYawPitch.pitch >>
//         p.torsoPosV.x() >> p.torsoPosV.y() >> p.torsoPosV.z() >>
//         p.torsoRotV.x() >> p.torsoRotV.y() >> p.torsoRotV.z() >>
//         p.otherFootPosV.x() >> p.otherFootPosV.y() >> p.otherFootPosV.z() >>
//         p.otherFootRotV.x() >> p.otherFootRotV.y() >> p.otherFootRotV.z();
//     p.supportFoot = static_cast<SUPPORT_FOOT>(i);
//     return in;
// }
