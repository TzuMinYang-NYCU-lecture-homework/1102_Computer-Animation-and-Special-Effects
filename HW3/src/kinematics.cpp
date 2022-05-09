#include "kinematics.h"

#include <algorithm>

// add by myself
#include <iostream>

#include "utils.h"
void forwardKinematics(const Posture& posture, Bone* bone) {
  // TODO (FK)
  // Same as HW2, but have some minor change
  // Hint:
  //   1. If you don't use `axis` in this function, you can copy-paste your code
  // Note:
  //   1. bone.axis becomes quaternion instead of vector3f
  // 
    // from HW2
    // add by myself 

    if (bone->idx == 0)
    {
        bone->rotation = posture.rotations[bone->idx];
        bone->startPosition = posture.translations[bone->idx];
        bone->endPosition = bone->startPosition + bone->rotation * bone->direction * bone->length;
    }
    else
    {
        bone->rotation = bone->parent->rotation * bone->rotationParentCurrent * posture.rotations[bone->idx]; // �e��ӭ��_�ӬORasf�A�᭱�ORamc
        bone->startPosition = posture.translations[bone->idx] + bone->parent->endPosition;  // ���Ҽ{translations�]�S���Y�A�Droot�����O0,0,0
        bone->endPosition = bone->startPosition + bone->rotation * bone->direction * bone->length;
    }

    if (bone->child != nullptr) // ���U����
        forwardKinematics(posture, bone->child);
    if (bone->sibling != nullptr) // ��V���ʡAsibling���|�^��ۤv
        forwardKinematics(posture, bone->sibling);
    // end of my code
}

Eigen::VectorXf leastSquareSolver(const Eigen::Matrix3Xf& jacobian, const Eigen::Vector3f& target) {
  // TODO (find x which min(| jacobian * x - target |))
  // Hint:
  //   1. Linear algebra - least squares solution
  //   2. https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse#Construction
  // Note:
  //   1. SVD or other pseudo-inverse method is useful
  //   2. Some of them have some limitation, if you use that method you should check it.
  Eigen::VectorXf solution(jacobian.cols());

  // add by myself
  solution = jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(target); // �γo�椧��sĶ�ܫܺC
  // end of my add

  return solution;
}

void inverseKinematics(const Eigen::Vector3f& target, Bone* start, Bone* end, Posture& posture) {
  constexpr int maxIterations = 10000;
  constexpr float epsilon = 1E-3f;
  constexpr float step = 0.1f;
  // Since bone stores in bones[i] that i == bone->idx, we can use bone - bone->idx to find bones[0] which is root.
  Bone* root = start - start->idx;
  std::vector<Bone*> boneList;

  // TODO
  // Hint:
  //   1. Traverse from end to start is easier than start to end (since there is only 1 parent)
  //   2. If start bone is not reachable from end. Go to root first.
  // Note:
  //   1. Both start and end should be in the list

  // Write your code here.
  // add by myself
  // ���Ostart ~ end�����Ҧ��s����bone���i�H���ʡA�ӬOend��start����parent-child���Y���~�i�H����
  // �B����إi��1.start�Oend��ancestor node�A����end��start�Y�i 2.start���Oend��ancestor node�A�ܦ�end��root�Aroot��start
  // 1.end to start�Y�i
  // 2.traverse�ɦ]���@��node��parent�u�|���@�ӡA�ҥH��end to root�A�qstart to root����n�A�n�`�N���n��root�Q��i�h�⦸
  Bone* current = end, *from = end;

  while (true)
  {
      boneList.emplace_back(current);

      // �n�קKroot�Q��i�h�⦸�Ψ���root��parent�����p
      if (current == start && from == end) break; // ���p1. ����
      else if (current == root && from == end) // ���p2. end to root�������A����start
      {
          current = start;
          from = start;
          boneList.emplace_back(current); // �n�O�o��start��i�h�A���M�����|��start��parent�A�o��start�|�S���
      }

      current = current->parent;
      if (current == root && from == start) break; // ���p2. start to end�������A�]�����ΦA��root��i�hlist�@���A��������
  }
  // end of by add

  size_t boneNum = boneList.size();
  Eigen::Matrix3Xf jacobian(3, 3 * boneNum);
  jacobian.setZero();

  for (int i = 0; i < maxIterations; ++i) {
    forwardKinematics(posture, root);
    // TODO (compute jacobian)
    //   1. Compute jacobian columns
    //   2. Compute dTheta
    // Hint:
    //   1. You should not put rotation in jacobian if it doesn't have that DoF.
    //   2. jacobian.col(/* some column index */) = /* jacobian column */
    //   3. Call leastSquareSolver to compute dTheta

    // Write your code here.
    // add by myself
    Eigen::Vector3f V_end_effector_to_target = target - end->endPosition; // ��������V
    if (V_end_effector_to_target.norm() < epsilon) break; // �p�G��target����N�i�H���F

    for (size_t j = 0; j < boneNum; j++) 
    {
        const auto& bone = *boneList[j];
        // �C�Ӷb��W�p��jacobian column
        //!!! ��(bone.rotation.vec()(0), 0, 0)��@x�A�Byz�P�ˤ�k�h�⪺�ܡA�̫ᵲ�G�|���A���ӬO�]��rotation���O�o�˥�
        Eigen::Vector3f bone_rx = bone.rotation * Eigen::Vector3f::UnitX(), bone_ry = bone.rotation * Eigen::Vector3f::UnitY(), bone_rz = bone.rotation * Eigen::Vector3f::UnitZ();
        Eigen::Vector3f r = end->endPosition - bone.startPosition; // ��������r�A�]�N�Op-ri

        // �p�G�����Ӧۥѫת��ܤ~���s���A���Mjacobian������column�N���M����0
        if (bone.dofrx) jacobian.col(j * 3) = bone_rx.cross(r);
        if (bone.dofry) jacobian.col(j * 3 + 1) = bone_ry.cross(r);
        if (bone.dofrz) jacobian.col(j * 3 + 2) = bone_rz.cross(r);
    }
    Eigen::VectorXf dTheta_mul_step = leastSquareSolver(jacobian, V_end_effector_to_target) * step; // �]���᭱�p��ɨC�����n*step�A�ҥH�o�䪽��*�F
    // end of my add

    for (size_t j = 0; j < boneNum; j++) {
      const auto& bone = *boneList[j];
      // TODO (update rotation)
      //   1. Update posture's eulerAngle using deltaTheta
      // Hint:
      //   1. Use posture.eulerAngle to get posture's eulerAngle
      //   2. All angles are in radians.
      //   3. You can ignore rotation limit of the bone.
      // Bonus:
      //   1. You cannot ignore rotation limit of the bone.

      // Write your code here.

      // add by myself
      posture.eulerAngle[bone.idx] += dTheta_mul_step.segment(j * 3, 3);  // ��stheta�W�h

        // bonus: �W�X����d�򪺨��״N�]����ɪ��ȴN�n�F
      if (posture.eulerAngle[bone.idx](0) < bone.rxmin) posture.eulerAngle[bone.idx](0) = bone.rxmin;
      else if (posture.eulerAngle[bone.idx](0) > bone.rxmax) posture.eulerAngle[bone.idx](0) = bone.rxmax;

      if (posture.eulerAngle[bone.idx](1) < bone.rymin) posture.eulerAngle[bone.idx](1) = bone.rymin;
      else if (posture.eulerAngle[bone.idx](1) > bone.rymax) posture.eulerAngle[bone.idx](1) = bone.rymax;

      if (posture.eulerAngle[bone.idx](2) < bone.rzmin) posture.eulerAngle[bone.idx](2) = bone.rzmin;
      else if (posture.eulerAngle[bone.idx](2) > bone.rzmax) posture.eulerAngle[bone.idx](2) = bone.rzmax;
        // end of bonus

      // end of my add

      posture.rotations[bone.idx] = Eigen::AngleAxisf(posture.eulerAngle[bone.idx][2], Eigen::Vector3f::UnitZ()) *
                                    Eigen::AngleAxisf(posture.eulerAngle[bone.idx][1], Eigen::Vector3f::UnitY()) *
                                    Eigen::AngleAxisf(posture.eulerAngle[bone.idx][0], Eigen::Vector3f::UnitX());
    }
  }
}
