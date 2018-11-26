#include "NaoStability.hpp"

#include <Tools/Kinematics/Com.h>
#include <Tools/Kinematics/ForwardKinematics.h>
#include <Tools/Kinematics/InverseKinematics.h>

#include "NaoPoseInfo.hpp"

class NaoFoot {
  Vector3f footTL;
  Vector3f footBR;
  Vector3f footBL;
  Vector3f footTR;

public:
  NaoFoot(Vector3f tl, Vector3f tr, Vector3f bl, Vector3f br)
      : footTL(tl), footBR(br), footBL(bl), footTR(tr) {}

  NaoFoot(Vector3f tl, Vector3f br)
      : footTL(tl), footBR(br), footBL(Vector3f(br.x(), tl.y(), tl.z())),
        footTR(Vector3f(tl.x(), br.y(), br.z())) {}

  const Vector3f tl() { return footTL; }

  const Vector3f tr() { return footTR; }

  const Vector3f bl() { return footBL; }

  const Vector3f br() { return footBR; }

  /**
   * Transform the foot by given kinematicMatrix
   * @param transform
   * @return transformed foot.
   */
  inline NaoFoot getTransformedFoot(KinematicMatrix transform) const {
    return NaoFoot(transform * footTL, transform * footTR, transform * footBL,
                   transform * footBR);
  }
  inline bool isWithinFoot(float &x, float &y) {
    return (x < footTL.x() && x > footBR.x() && y < footTL.y() &&
            y > footBR.y());
  }
};

class SupportPolygon {
  const NaoFoot leftFoot;
  const NaoFoot rightFoot;

public:
  SupportPolygon()
      : leftFoot(NaoFoot(Vector3f(100, 45, 0), Vector3f(-57, -31, 0))),
        rightFoot(NaoFoot(Vector3f(100, 31, 0), Vector3f(-57, -45, 0))) {}

private:
  inline bool isComWithinSupportDoubleFoot(const KinematicMatrix &lFootPose,
                                           const KinematicMatrix &rFootPose,
                                           const KinematicMatrix &com) const {
    KinematicMatrix rFoot2LeftFoot = lFootPose.invert() * rFootPose;
    bool leftFootForward = rFoot2LeftFoot.posV.x() < 0;

    // right foot in left's coordinates
    NaoFoot rFootInLFoot = rightFoot.getTransformedFoot(rFoot2LeftFoot);
    NaoFoot lFoot = leftFoot;
    KinematicMatrix comInLeftFoot = lFootPose.invert() * com;

    const float comX = comInLeftFoot.posV.x();
    const float comY = comInLeftFoot.posV.y();

    if (leftFootForward) {
      // if in the bounding box
      if (lFoot.tl().x() > comX && rFootInLFoot.br().x() < comX &&
          lFoot.tl().y() > comY && rFootInLFoot.br().y() < comY) {
        // if in the two triangles
        Vector3f diff = (rFootInLFoot.tr() - lFoot.tr());
        // when diff x ~= 0, then the slope is inf~, thus support poly. is
        // rectangular ~= bounding box
        if (abs(diff.x()) > __FLT_EPSILON__) {
          const float gradientTR = diff.y() / diff.x();
          const float y1 = gradientTR * (comX - rFootInLFoot.tr().x()) +
                           rFootInLFoot.tr().y();
          if (rFootInLFoot.tr().x() > comX && comY < y1) {
            return false;
          }
          const float y2 = gradientTR * (comX - rFootInLFoot.bl().x()) +
                           rFootInLFoot.bl().y();
          if (lFoot.bl().x() > comX && comY > y2) {
            return false;
          }
        }
        // if all else, return true.
        return true;
      } else {
        return false;
      }
    } else // right foot forward
    {
      // if in the bounding box
      if (rFootInLFoot.tr().x() > comX && lFoot.bl().x() < comX &&
          rFootInLFoot.tr().y() < comY && lFoot.bl().y() > comY) {
        // if in the two triangles
        Vector3f diff = (rFootInLFoot.tl() - lFoot.tl());
        const float gradientTR = diff.y() / diff.x();
        // when diff x ~= 0, then the slope is inf~, thus support poly. is
        // rectangular ~= bounding box
        if (abs(diff.x()) > __FLT_EPSILON__) {
          const float y1 = gradientTR * (comX - rFootInLFoot.tl().x()) +
                           rFootInLFoot.tl().y();
          if (lFoot.tl().x() > comX && comY > y1) {
            return false;
          }
          const float y2 = gradientTR * (comX - rFootInLFoot.br().x()) +
                           rFootInLFoot.br().y();
          if (rFootInLFoot.br().x() < comX && comY < y2) {
            return false;
          }
        }
        // if all else, return true.
        return true;
      } else {
        return false;
      }
    }
    return false;
  }

public:
  /**
   * @param lFoot left foot position in torso coords
   * @param rFoot ..
   * @param COM in torso coords
   * @return if COM within support polygon
   *
   */
  inline bool isComWithinSupport(
      const KinematicMatrix &lFootPose, const KinematicMatrix &rFootPose,
      const KinematicMatrix &com,
      const SUPPORT_FOOT &supFoot = SUPPORT_FOOT::SF_DOUBLE) const {
    if (supFoot == SUPPORT_FOOT::SF_LEFT) {
      KinematicMatrix comInLeftFoot = lFootPose.invert() * com;
      NaoFoot supFoot = leftFoot;
      float comX = comInLeftFoot.posV.x();
      float comY = comInLeftFoot.posV.y();
      return supFoot.isWithinFoot(comX, comY);
    } else if (supFoot == SUPPORT_FOOT::SF_RIGHT) {
      KinematicMatrix comInRight = rFootPose.invert() * com;
      NaoFoot supFoot = rightFoot;
      float comX = comInRight.posV.x();
      float comY = comInRight.posV.y();
      return supFoot.isWithinFoot(comX, comY);
    } else if (supFoot == SUPPORT_FOOT::SF_DOUBLE) {
      return isComWithinSupportDoubleFoot(lFootPose, rFootPose, com);
    } else {
      return false;
    }
  }
};
