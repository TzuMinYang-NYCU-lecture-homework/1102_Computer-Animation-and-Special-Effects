#include "kinematics.h"

#include <algorithm>

#include "utils.h"
void forwardKinematics(const Posture& posture, Bone* bone) {
  // TODO (FK)
  // You should set these variables:
  //     bone->startPosition = Eigen::Vector3f::Zero();
  //     bone->endPosition = Eigen::Vector3f::Zero();
  //     bone->rotation = Eigen::Quaternionf::Identity();
  // The sample above just set everything to initial state
  // Hint:
  //   1. posture.translations, posture.rotations
  // Note:
  //   1. This function will be called with bone == root bone of the skeleton

  // Write your code here
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

Motion motionWarp(const Motion& motion, int oldKeyframe, int newKeyframe) {
  Motion newMotion = motion;
  int totalFrames = static_cast<int>(motion.size());
  int totalBones = static_cast<int>(motion.posture(0).rotations.size());

  // add by myself
  float ratio = (float)oldKeyframe / (float)newKeyframe;
  float target_frame = 0;   // new frame �| map �� original frame �����Ӽv��
  // end of my add

  for (int i = 0; i < totalFrames; ++i) {
    // Maybe set some per=Frame variables here
    // add by myself
      if (i < newKeyframe) target_frame = ratio * i;
      else if (i == newKeyframe)
      {
          ratio = (float)(totalFrames - oldKeyframe) / (float)(totalFrames - newKeyframe);  // ��b�����n���s��ratio
          target_frame = oldKeyframe;
      }
      else if (i < totalFrames - 1) target_frame = ratio * (i - newKeyframe) + oldKeyframe;
      else target_frame = totalFrames - 1 - 1 + 0.99; // �קK�̫�@�ӶW�X�}�C�d��
    // end of my code

    for (int j = 0; j < totalBones; ++j) {
      // TODO (Time warping)
      // original: |--------------|---------------|
      // new     : |------------------|-----------|
      // OR
      // original: |--------------|---------------|
      // new     : |----------|-------------------|
      // You should set these variables:
      //     newMotion.posture(i).translations[j] = Eigen::Vector3f::Zero();
      //     newMotion.posture(i).rotations[j] = Eigen::Quaternionf::Identity();
      // The sample above just set to initial state
      // Hint:
      //   1. Your should scale the frames before and after key frames.
      //   2. You can use linear interpolation with translations.
      //   3. You should use spherical linear interpolation for rotations.

      // Write your code here
      // add by myself
        float interplation_ratio = target_frame - (int)target_frame;
        newMotion.posture(i).translations[j] = motion.posture((int)target_frame).translations[j] * (1 - interplation_ratio) + motion.posture((int)target_frame + 1).translations[j] * interplation_ratio;
        newMotion.posture(i).rotations[j] = motion.posture((int)target_frame).rotations[j].slerp(interplation_ratio, motion.posture((int)target_frame + 1).rotations[j]);
      // end of my code
    }
  }
  return newMotion;
}

Motion motionBlend(const Motion& motionA, const Motion& motionB) {
  Motion newMotion;
  constexpr int blendFrameCount = 20;
  constexpr float blendFactor = 1.0f / blendFrameCount;
  constexpr int matchRange = 10;
  float difference[matchRange] = {};
  // TODO (Bonus)
  // motionA: |--------------|--matchRange--|--blendFrameCount--|
  // motionB:                               |--blendFrameCount--|--------------|
  // The starting frame of `blendFrameCount` can be in `matchRange`
  // Hint:
  //   1. Find motionB's starting posture
  //   2. Match it with the minimum cost posture in `matchRange`
  //   3. Find to translation and rotation offset between matched motionA and motionB's start
  //   4. Begin from the matched frame, blend `blendFrameCount` of frames,
  //      with a blendFactor from 1 / `blendFrameCount` to 1
  //   5. Add remaining motionB to newMotion
  // Note:
  //   1. The offset found in 3 should apply to 4 and 5
  //   2. A simple cost function is translation offsets between two posture.
  //   3. A better one considered both translations and rotations.
  //   4. Your animation should smoothly change from motionA to motionB.
  //   5. You can adjust those `constexpr`s above by yourself if you need.

  // Write your code here
  // add by my self

  // step.1&2
  float cur_difference = 0, min_difference = 1e10;
  int a_frame = 0, b_frame = 0;

  for (int i = motionA.size() - matchRange - 1; i >= motionA.size() / 5 * 4; --i)   // ���}�l�α����a��᭱�@�I�M�U�Ъ������
  {
      for (int j = 0; j < motionB.size() - matchRange; ++j)
      {
          cur_difference = 0;
          for (int k = 0; k < matchRange; ++k)
          {
              for (int l = 0; l < motionA.posture()[i + k].translations.size(); ++l)
                cur_difference += (motionA.posture()[i + k].translations[l] - motionB.posture()[j + k].translations[l]).norm();
              for (int l = 0; l < motionA.posture()[i + k].rotations.size(); ++l)
                cur_difference += (motionA.posture()[i + k].rotations[l].vec() - motionB.posture()[j + k].rotations[l].vec()).norm();
              // rotation����Ƶ��c���Ӥ��ઽ���۴�A�n�নvector�~��A�����D��]
          }

          if (min_difference > cur_difference)
          {
              min_difference = cur_difference;
              a_frame = i;
              b_frame = j;
          }
      }
  }

  // step.3
  Eigen::Vector3f offset = motionA.posture()[a_frame].translations[0] - motionB.posture()[b_frame].translations[0];

  // step.3.5 ��motionA���[�i�h
  for (int i = 0; i < a_frame; ++i)
      newMotion.posture().emplace_back(motionA.posture()[i]);
  
  // step.4
  for (int i = 0; i < blendFrameCount; ++i)
  {
      newMotion.posture().emplace_back(motionA.posture()[i + a_frame]); // �H�K�[�@��frame�i�hnewMotion�A�᭱�A��L����

      float ratio = blendFactor * i;
      // �u��root��translations
      newMotion.posture(a_frame + i).translations[0] = motionA.posture(i + a_frame).translations[0] * (1 - ratio) + (motionB.posture(i + b_frame).translations[0] + offset) * ratio;
      for (int j = 0; j < motionA.posture()[i + a_frame].rotations.size(); ++j)
          newMotion.posture(a_frame + i).rotations[j] = motionA.posture(i + a_frame).rotations[j].slerp(ratio, motionB.posture(i + b_frame).rotations[j]);
  }

  // step.5
  for (int i = b_frame + blendFrameCount; i < motionB.size(); ++i) // ���|��blending���a��(�j�p��blendFrameCount��window)�N���ΦA�[��newMotion�F
      newMotion.posture().emplace_back(motionB.posture()[i]);

  for (int i = a_frame + blendFrameCount; i < newMotion.posture().size(); ++i)  // motionB���[offset�N�n
      newMotion.posture()[i].translations[0] += offset;

  // end of my add
  return newMotion;
}
