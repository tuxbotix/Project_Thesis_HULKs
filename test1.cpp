#include <iostream>

#include "Data/CameraMatrix.hpp"
#include "Modules/NaoProvider.h"
#include "Modules/Configuration/Configuration.h"
#include "Modules/Configuration/UnixSocketConfig.hpp"
#include <Modules/Poses.h>
#include <Tools/Kinematics/ForwardKinematics.h>
#include <Tools/Kinematics/InverseKinematics.h>
#include "Tools/Kinematics/KinematicMatrix.h"
#include "Tools/Kinematics/Com.h"
#include "Tools/Storage/Image.hpp"
#include "Tools/Storage/Image422.hpp"
#include "Hardware/RobotInterface.hpp"

class NaoFoot
{
    Vector3f footTL;
    Vector3f footBR;
    Vector3f footBL;
    Vector3f footTR;

  public:
    NaoFoot(Vector3f tl, Vector3f tr, Vector3f bl, Vector3f br) : footTL(tl),
                                                                  footBR(br),
                                                                  footBL(bl),
                                                                  footTR(tr) {}

    NaoFoot(Vector3f tl, Vector3f br) : footTL(tl),
                                        footBR(br),
                                        footBL(Vector3f(br.x(), tl.y(), tl.z())),
                                        footTR(Vector3f(tl.x(), br.y(), br.z())) {}

    const Vector3f tl()
    {
        return footTL;
    }

    const Vector3f tr()
    {
        return footTR;
    }

    const Vector3f bl()
    {
        return footBL;
    }

    const Vector3f br()
    {
        return footBR;
    }

    /**
     * Transform the foot by given kinematicMatrix
     * @param transform
     * @return transformed foot.
     */
    NaoFoot getTransformedFoot(KinematicMatrix transform) const
    {
        return NaoFoot(transform * footTL, transform * footTR, transform * footBL, transform * footBR);
    }
};

class SupportPolygon
{
    const NaoFoot leftFoot;
    const NaoFoot rightFoot;

  public:
    SupportPolygon() : leftFoot(NaoFoot(Vector3f(100, 45, 0), Vector3f(-57, -31, 0))),
                       rightFoot(NaoFoot(Vector3f(100, 31, 0), Vector3f(-57, -45, 0)))
    {
    }

    /**
     * @param lFoot left foot position in torso coords
     * @param rFoot ..
     * @param COM in torso coords
     * @return if COM within support polygon
     */
    bool isComWithinSupport(KinematicMatrix lFootPose, KinematicMatrix rFootPose, KinematicMatrix com)
    {
        KinematicMatrix rFoot2LeftFoot = lFootPose.invert() * rFootPose;
        bool leftFootForward = rFoot2LeftFoot.posV.x() < 0;

        // right foot in left's coordinates
        NaoFoot rFootInLFoot = rightFoot.getTransformedFoot(rFoot2LeftFoot);
        NaoFoot lFoot = leftFoot;
        KinematicMatrix comInLeftFoot = lFootPose.invert() * com;

        const float comX = comInLeftFoot.posV.x();
        const float comY = comInLeftFoot.posV.y();

        // std::cout << "leftForward " << leftFootForward << " \n"
        //           << comX << " " << comY << std::endl;
        if (leftFootForward)
        {
            // std::cout << "feet bounding box: lfoot\n"
            //           << lFoot.tl() << "\n rfoot \n"
            //           << rFootInLFoot.br() << std::endl;
            // if in the bounding box
            if (lFoot.tl().x() > comX && rFootInLFoot.br().x() < comX &&
                lFoot.tl().y() > comY && rFootInLFoot.br().y() < comY)
            {
                // if in the two triangles
                Vector3f diff = (rFootInLFoot.tr() - lFoot.tr());
                // when diff x ~= 0, then the slope is inf~, thus support poly. is rectangular ~= bounding box
                if (abs(diff.x()) > __FLT_EPSILON__)
                {
                    const float gradientTR = diff.y() / diff.x();
                    const float y1 = gradientTR * (comX - rFootInLFoot.tr().x()) + rFootInLFoot.tr().y();
                    if (rFootInLFoot.tr().x() > comX && comY < y1)
                    {
                        // std::cout << "Upper triangle " << gradientTR << " " << y1 << std::endl;
                        return false;
                    }
                    const float y2 = gradientTR * (comX - rFootInLFoot.bl().x()) + rFootInLFoot.bl().y();
                    if (lFoot.bl().x() > comX && comY > y2)
                    {
                        // std::cout << "Lower triangle " << y2 << std::endl;
                        return false;
                    }
                }
                // if all else, return true.
                return true;
            }
            else
            {
                // std::cout << "Not in bounding box" << std::endl;
                return false;
            }
        }
        else // right foot forward
        {
            // std::cout << "feet bounding box: rfoot\n"
            //           << rFootInLFoot.tr() << "\n lfoot \n"
            //           << lFoot.bl() << std::endl;
            // if in the bounding box
            if (rFootInLFoot.tr().x() > comX && lFoot.bl().x() < comX &&
                rFootInLFoot.tr().y() < comY && lFoot.bl().y() > comY)
            {
                // if in the two triangles
                Vector3f diff = (rFootInLFoot.tl() - lFoot.tl());
                const float gradientTR = diff.y() / diff.x();
                // when diff x ~= 0, then the slope is inf~, thus support poly. is rectangular ~= bounding box
                if (abs(diff.x()) > __FLT_EPSILON__)
                {
                    const float y1 = gradientTR * (comX - rFootInLFoot.tl().x()) + rFootInLFoot.tl().y();
                    if (lFoot.tl().x() > comX && comY > y1)
                    {
                        // std::cout << "Upper triangle" << std::endl;
                        return false;
                    }
                    const float y2 = gradientTR * (comX - rFootInLFoot.br().x()) + rFootInLFoot.br().y();
                    if (rFootInLFoot.br().x() < comX && comY < y2)
                    {
                        // std::cout << "lower triangle" << std::endl;
                        return false;
                    }
                }
                // if all else, return true.
                return true;
            }
            else
            {
                // std::cout << "Not in bounding box" << std::endl;
                return false;
            }
        }
        return false;
    }
};

SupportPolygon supportPoly;

/**
 * Called at end of each update of joint angle
 */
bool poseCallback(std::vector<float> &pose)
{
    /// Where would the com be after setting these angles?
    KinematicMatrix com2torso = KinematicMatrix(Com::getCom(pose));

    KinematicMatrix lFoot2torso = ForwardKinematics::getLFoot(std::vector<float>(&pose[JOINTS::L_HIP_YAW_PITCH], &pose[JOINTS::L_ANKLE_ROLL]));
    KinematicMatrix rFoot2torso = ForwardKinematics::getRFoot(std::vector<float>(&pose[JOINTS::R_HIP_YAW_PITCH], &pose[JOINTS::R_ANKLE_ROLL]));

    // std::cout << "com \n"
    //           << Com::getCom(pose) << "\n lfoot " << lFoot2torso.posV << "\n rfoot \n"
    //           << rFoot2torso.posV << std::endl;

    bool isStable = supportPoly.isComWithinSupport(lFoot2torso, rFoot2torso, com2torso);
    return isStable;
}

/**
 * Iterate through a given joint, and recursively call the next joint..
 */
void jointIterFunc(const JOINTS::JOINT &jointIndex, float incrementInRad, std::vector<std::vector<float>> &poseList, const std::vector<float> &defaultPose /*, void (*poseCallback)(std::vector<float> &pose)*/)
{
    std::vector<float> pose(JOINTS::JOINTS_MAX);
    // TODO check this
    for (auto &item : pose)
    {
        item = 0;
    }
    // Skip arms, hands..
    if ((jointIndex >= JOINTS::L_SHOULDER_PITCH && jointIndex <= JOINTS::L_HAND) ||
        (jointIndex >= JOINTS::R_SHOULDER_PITCH && jointIndex <= JOINTS::R_HAND))
    {
        // put ready pose value as default
        pose[jointIndex] = defaultPose[jointIndex];
    }
    else
    {
        float minLim = NaoProvider::minRange(jointIndex);
        float maxLim = NaoProvider::maxRange(jointIndex);
        if (jointIndex == JOINTS::HEAD_PITCH)
        {
            // feed the current head yaw to get corresponding head pitch limits
            minLim = NaoProvider::minRangeHeadPitch(pose[JOINTS::HEAD_YAW]);
            maxLim = NaoProvider::maxRangeHeadPitch(pose[JOINTS::HEAD_YAW]);
        }
        else if (jointIndex == JOINTS::L_ANKLE_ROLL)
        {
            // feed the current head yaw to get corresponding head pitch limits
            minLim = NaoProvider::minRangeLAnkleRoll(pose[JOINTS::L_ANKLE_PITCH]);
            maxLim = NaoProvider::maxRangeLAnkleRoll(pose[JOINTS::L_ANKLE_PITCH]);
        }
        else if (jointIndex == JOINTS::R_ANKLE_ROLL)
        {
            // feed the current head yaw to get corresponding head pitch limits
            minLim = NaoProvider::minRangeRAnkleRoll(pose[JOINTS::R_ANKLE_PITCH]);
            maxLim = NaoProvider::maxRangeRAnkleRoll(pose[JOINTS::R_ANKLE_PITCH]);
        }

        std::cout << "try joint " << jointIndex << " min " << minLim << " max " << maxLim << std::endl;

        std::vector<float> joints(JOINTS::JOINTS_MAX);
        for (float j = minLim; j < maxLim; j += incrementInRad)
        {
            // std::cout << "iter joint " << jointIndex << std::endl;
            /// skip validation for right hip yaw pitch as it is same as left hip yaw pitch
            if (jointIndex == JOINTS::R_HIP_YAW_PITCH)
            {
                pose[jointIndex] = pose[JOINTS::L_HIP_YAW_PITCH];
            }
            else
            {
                pose[jointIndex] = j;
                if (poseCallback(pose))
                {
                    poseList.push_back(pose);
                }
            }
        }
    }
    JOINTS::JOINT nextIndex = static_cast<JOINTS::JOINT>(jointIndex + 1);
    if (nextIndex < JOINTS::JOINTS_MAX)
    {
        jointIterFunc(nextIndex, incrementInRad, poseList, defaultPose /*, poseCallback*/);
        return;
    }
    std::cout << "end of recursion" << std::endl;
}

/**
 * Minimal TUHH class impl. in order to use configuration
 * This is a rather hacky way, but doesn't need touching the actual belly of the beast xD
 */
class Configuration;
class TUHH
{

  public:
    Configuration config_;
    TUHH(std::string fileRoot) : config_(fileRoot)
    {
        NaoInfo info;
        info.bodyVersion = NaoVersion::V5;
        info.headVersion = NaoVersion::V5;
        info.bodyName = "default";
        info.headName = "default";

        config_.setLocationName("default");
        config_.setNaoHeadName(info.headName);
        config_.setNaoBodyName(info.bodyName);

        NaoProvider::init(config_, info);
        std::cout << "initialize TUHH" << std::endl;
        std::cout << Poses::init(fileRoot) << std::endl;
    }
};

int main()
{
    TUHH tuhhInstance("/home/darshana/Documents/HULKs/nao/home/");
    const std::vector<float> readyPose = Poses::getPose(Poses::READY);

    int imageWidth = 640;
    int imageHeight = 480;

    CameraMatrix camMatrix;

    camMatrix.fc = Vector2f(imageWidth, imageHeight) / 2;
    camMatrix.cc = Vector2f(imageWidth, imageHeight) / 2;

    Vector2i pixelVal(1, 1);

    std::vector<std::vector<float>> poseList;

    // std::vector<float> curPose(JOINTS::JOINTS_MAX);
    // for (auto &item : curPose)
    // {
    //     item = 0;
    // }
    // std::cout << "com \n"
    //           << Com::getCom(accumJoints) << std::endl;
    // poseCallback(accumJoints);

    std::cout << "init" << std::endl;
    const float incrementInRad = 10 * TO_RAD; // 10 deg increment
    // recursively try all joint combinations.
    jointIterFunc(JOINTS::HEAD_YAW, incrementInRad, poseList, readyPose);

    return 0;
}
