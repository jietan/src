#ifndef SEHOON_NICK01A_H
#define SEHOON_NICK01A_H

namespace sehoon {
    namespace Nick01a {
        enum Nodes {
            Nick01a_Hips, // 0
            Nick01a_LeftUpLeg,
            Nick01a_LeftLeg,
            Nick01a_LeftFoot,
            Nick01a_RightUpLeg,
            Nick01a_RightLeg, // 5
            Nick01a_RightFoot,
            Nick01a_Spine1,
            Nick01a_Neck,
            Nick01a_Head,
            Nick01a_LeftShoulder, // 10
            Nick01a_LeftArm,
            Nick01a_LeftForeArm,
            Nick01a_LeftHand,
            Nick01a_RightShoulder,
            Nick01a_RightArm, // 15
            Nick01a_RightForeArm,
            Nick01a_RightHand,
            NUM_NODES
        }; // enum Nodes
        enum Dofs {
            Nick01a_Hips_tFree0,
            Nick01a_Hips_tFree1,
            Nick01a_Hips_tFree2,
            Nick01a_Hips_aFree3,
            Nick01a_Hips_aFree4,
            Nick01a_Hips_aFree5,
            Nick01a_LeftUpLeg_aBall0,
            Nick01a_LeftUpLeg_aBall1,
            Nick01a_LeftUpLeg_aBall2,
            Nick01a_LeftLeg_aHinge0,
            Nick01a_LeftHill_aBall0,
            Nick01a_LeftHill_aBall1,
            Nick01a_LeftHill_aBall2,
            Nick01a_RightUpLeg_aBall0,
            Nick01a_RightUpLeg_aBall1,
            Nick01a_RightUpLeg_aBall2,
            Nick01a_RightLeg_aHinge0,
            Nick01a_RightHill_aBall0,
            Nick01a_RightHill_aBall1,
            Nick01a_RightHill_aBall2,
            Nick01a_Spine1_aBall0,
            Nick01a_Spine1_aBall1,
            Nick01a_Spine1_aBall2,
            Nick01a_Neck_aBall0,
            Nick01a_Neck_aBall1,
            Nick01a_Neck_aBall2,
            Nick01a_Head_aBall0,
            Nick01a_Head_aBall1,
            Nick01a_Head_aBall2,
            Nick01a_LeftShoulder_aBall0,
            Nick01a_LeftShoulder_aBall1,
            Nick01a_LeftShoulder_aBall2,
            Nick01a_LeftArm_aBall0,
            Nick01a_LeftArm_aBall1,
            Nick01a_LeftArm_aBall2,
            Nick01a_LeftForeArm_aHinge0,
            Nick01a_LeftWrist_aBall0,
            Nick01a_LeftWrist_aBall1,
            Nick01a_LeftWrist_aBall2,
            Nick01a_RightShoulder_aBall0,
            Nick01a_RightShoulder_aBall1,
            Nick01a_RightShoulder_aBall2,
            Nick01a_RightArm_aBall0,
            Nick01a_RightArm_aBall1,
            Nick01a_RightArm_aBall2,
            Nick01a_RightForeArm_aHinge0,
            Nick01a_RightWrist_aBall0,
            Nick01a_RightWrist_aBall1,
            Nick01a_RightWrist_aBall2,
            NUM_DOFS
        }; // enum Dofs
        enum Markers {
            Nick01a_ARIEL,
            Nick01a_LFHD,
            Nick01a_LBHD,
            Nick01a_RFHD,
            Nick01a_RBHD,
            Nick01a_C7,
            Nick01a_T10,
            Nick01a_CLAV,
            Nick01a_STRN,
            Nick01a_LFSH,
            Nick01a_LBSH,
            Nick01a_LUPA,
            Nick01a_LELB,
            Nick01a_LIEL,
            Nick01a_LOWR,
            Nick01a_LIWR,
            Nick01a_LWRE,
            Nick01a_LIHAND,
            Nick01a_LOHAND,
            Nick01a_RFSH,
            Nick01a_RBSH,
            Nick01a_RUPA,
            Nick01a_RELB,
            Nick01a_RIEL,
            Nick01a_ROWR,
            Nick01a_RIWR,
            Nick01a_RWRE,
            Nick01a_RIHAND,
            Nick01a_ROHAND,
            Nick01a_LFWT,
            Nick01a_LMWT,
            Nick01a_LBWT,
            Nick01a_RFWT,
            Nick01a_RMWT,
            Nick01a_RBWT,
            Nick01a_LHIP,
            Nick01a_LKNE,
            Nick01a_LKNI,
            Nick01a_LSHN,
            Nick01a_LANK,
            Nick01a_LHEL,
            Nick01a_LMT5,
            Nick01a_LMT1,
            Nick01a_LTOE,
            Nick01a_RHIP,
            Nick01a_RKNE,
            Nick01a_RKNI,
            Nick01a_RSHN,
            Nick01a_RANK,
            Nick01a_RHEL,
            Nick01a_RMT5,
            Nick01a_RMT1,
            Nick01a_RTOE,
            NUM_MARKERS
        }; // enum Markers
    } // namespace Nick01a
} // namespace sehoon

#endif // #ifndef SEHOON_NICK01A_H