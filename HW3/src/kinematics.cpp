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
        bone->rotation = bone->parent->rotation * bone->rotationParentCurrent * posture.rotations[bone->idx]; // 前兩個乘起來是Rasf，後面是Ramc
        bone->startPosition = posture.translations[bone->idx] + bone->parent->endPosition;  // 不考慮translations也沒關係，非root的都是0,0,0
        bone->endPosition = bone->startPosition + bone->rotation * bone->direction * bone->length;
    }

    if (bone->child != nullptr) // 往下移動
        forwardKinematics(posture, bone->child);
    if (bone->sibling != nullptr) // 橫向移動，sibling不會回到自己
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
  solution = jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(target); // 用這行之後編譯變很慢
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
  // 不是start ~ end中間所有編號的bone都可以活動，而是end到start中有parent-child關係的才可以活動
  // 且有兩種可能1.start是end的ancestor node，直接end到start即可 2.start不是end的ancestor node，變成end到root，root到start
  // 1.end to start即可
  // 2.traverse時因為一個node的parent只會有一個，所以用end to root再從start to root比較好，要注意不要讓root被放進去兩次
  Bone* current = end, *from = end;

  while (true)
  {
      boneList.emplace_back(current);

      // 要避免root被放進去兩次或取到root的parent的情況
      if (current == start && from == end) break; // 情況1. 結束
      else if (current == root && from == end) // 情況2. end to root的部分，跳到start
      {
          current = start;
          from = start;
          boneList.emplace_back(current); // 要記得把start放進去，不然等等會取start的parent，這樣start會沒放到
      }

      current = current->parent;
      if (current == root && from == start) break; // 情況2. start to end的部分，因為不用再把root放進去list一次，直接結束
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
    Eigen::Vector3f V_end_effector_to_target = target - end->endPosition; // 公式中的V
    if (V_end_effector_to_target.norm() < epsilon) break; // 如果離target夠近就可以停了

    for (size_t j = 0; j < boneNum; j++) 
    {
        const auto& bone = *boneList[j];
        // 每個軸單獨計算jacobian column
        //!!! 用(bone.rotation.vec()(0), 0, 0)當作x，且yz同樣方法去算的話，最後結果會錯，應該是因為rotation不是這樣用
        Eigen::Vector3f bone_rx = bone.rotation * Eigen::Vector3f::UnitX(), bone_ry = bone.rotation * Eigen::Vector3f::UnitY(), bone_rz = bone.rotation * Eigen::Vector3f::UnitZ();
        Eigen::Vector3f r = end->endPosition - bone.startPosition; // 公式中的r，也就是p-ri

        // 如果有那個自由度的話才能彎曲，不然jacobian對應的column就仍然維持0
        if (bone.dofrx) jacobian.col(j * 3) = bone_rx.cross(r);
        if (bone.dofry) jacobian.col(j * 3 + 1) = bone_ry.cross(r);
        if (bone.dofrz) jacobian.col(j * 3 + 2) = bone_rz.cross(r);
    }
    Eigen::VectorXf dTheta_mul_step = leastSquareSolver(jacobian, V_end_effector_to_target) * step; // 因為後面計算時每次都要*step，所以這邊直接*了
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
      posture.eulerAngle[bone.idx] += dTheta_mul_step.segment(j * 3, 3);  // 更新theta上去

        // bonus: 超出限制範圍的角度就設為邊界的值就好了
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
