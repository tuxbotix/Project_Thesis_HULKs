#include <Tools/Kinematics/Com.h>
#include <Tools/Kinematics/ForwardKinematics.h>
#include <Tools/Kinematics/InverseKinematics.h>
#include <Tools/Kinematics/KinematicMatrix.h>

#include "NaoPoseInfo.hpp"

class NaoTorsoPose
{
  public:
    // enum SUPPORT_FOOT
    // {
    //     SF_RIGHT = -1,
    //     SF_DOUBLE = 0,
    //     SF_LEFT = 1,
    //     SF_NO = 2
    // };
    typedef float angleT;
    typedef std::vector<angleT> jointAnglesT;

  private:
    /**
     * @brief compute body angles
     * @param bodyAngles - output
     * @param target torso position relative to support foot.
     * @param target torso rotation relative to support foot.
     * @param other foot's position relative to support foot
     * @param other foot's rotation relative to support foot
     * @param Support foot (-1 right, 0 double, 1 left)
     * 
     * **Warning** No safety checks done!!!
     */
    static bool leanToTorsoPose(jointAnglesT &jointAngles,
                                const KinematicMatrix &targetTorso2supportFoot,
                                const Vector3f &otherFootPos2supportFoot,
                                const Vector3f &otherFootRot2SupportFoot,
                                const SUPPORT_FOOT &supportFoot)
    {
        KinematicMatrix otherFoot2SF_copy; // = otherFoot2supportFoot;
        /// Get the other foot to support foot's (bottom) plane (if double support)
        if (supportFoot == SUPPORT_FOOT::SF_DOUBLE)
        {
            if (std::abs(otherFoot2SF_copy.posV.z()) > __FLT_EPSILON__ ||
                std::abs(otherFootRot2SupportFoot.x()) > __FLT_EPSILON__ ||
                std::abs(otherFootRot2SupportFoot.y()) > __FLT_EPSILON__)
            {
                // the feet are not in plane, no point of continuing.
                return false;
            }
            otherFoot2SF_copy.posV = otherFootPos2supportFoot;
            // no step height
            otherFoot2SF_copy.posV.z() = 0;
            // No foot roll or pitch relative to support foot.
            otherFoot2SF_copy.rotM = AngleAxisf(otherFootRot2SupportFoot.z(), Vector3f::UnitZ());
        }
        else
        {
            otherFoot2SF_copy.posV = otherFootPos2supportFoot;
            otherFoot2SF_copy.rotM = AngleAxisf(otherFootRot2SupportFoot.z(), Vector3f::UnitZ()) *
                                     AngleAxisf(otherFootRot2SupportFoot.y(), Vector3f::UnitY()) *
                                     AngleAxisf(otherFootRot2SupportFoot.x(), Vector3f::UnitX());
        }

        KinematicMatrix supFoot2targetPose = targetTorso2supportFoot.invert();
        KinematicMatrix otherFoot2targetTorso = supFoot2targetPose * otherFoot2SF_copy;

        std::vector<float> lLegAngles, rLegAngles;

        /// make the leg fixed to the side the nao is leaning.
        if (supportFoot == SUPPORT_FOOT::SF_LEFT || supportFoot == SUPPORT_FOOT::SF_DOUBLE)
        {
            lLegAngles = InverseKinematics::getLLegAngles(supFoot2targetPose);
            // KinematicMatrix mat = ForwardKinematics::getLFoot(lLegAngles);
            rLegAngles = InverseKinematics::getFixedRLegAngles(otherFoot2targetTorso, lLegAngles[0]);
        }
        else
        {
            rLegAngles = InverseKinematics::getRLegAngles(supFoot2targetPose);
            lLegAngles = InverseKinematics::getFixedLLegAngles(otherFoot2targetTorso, rLegAngles[0]);
        }
        /// put computed leg angles in joint angle vector for whole body
        for (int j = 0; j < JOINTS_L_LEG::L_LEG_MAX; j++)
        {
            jointAngles[JOINTS::L_HIP_YAW_PITCH + j] = lLegAngles[j];
            jointAngles[JOINTS::R_HIP_YAW_PITCH + j] = rLegAngles[j];
        }
        // TODO add more validation..
        return true;
    }

  public:
    static bool getPose(jointAnglesT &jointAngles,
                        const Vector3f &targetTorsoPos2supportFoot,
                        const Vector3f &targetTorsoRot2supportFoot,
                        const Vector3f &otherFootPos2supportFoot,
                        const Vector3f &otherFootRot2SupportFoot,
                        const SUPPORT_FOOT &supportFoot)
    {
        KinematicMatrix torsoPose(targetTorsoPos2supportFoot * 1000);
        torsoPose.rotM =
            AngleAxisf(targetTorsoRot2supportFoot.z(), Vector3f::UnitZ()) *
            AngleAxisf(targetTorsoRot2supportFoot.y(), Vector3f::UnitY()) *
            AngleAxisf(targetTorsoRot2supportFoot.x(), Vector3f::UnitX());
        return leanToTorsoPose(jointAngles, torsoPose,
                                       otherFootPos2supportFoot * 1000,
                                       otherFootRot2SupportFoot,
                                       supportFoot);
    }
};
