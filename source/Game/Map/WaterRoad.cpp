#include "Game/Map/WaterRoad.h"
#include "Game/AudioLib/AudSoundObject.h"
#include "Game/LiveActor/HitSensor.h"
#include "Game/LiveActor/LiveActor.h"
#include "Game/LiveActor/Spine.h"
#include "Game/Scene/SceneFunction.h"
#include "Game/Util.h"
#include "JSystem/JUtility/JUTTexture.h"
#include "revolution/gd/GDBase.h"
#include "revolution/gx/GXEnum.h"
#include "revolution/gx/GXGeometry.h"
#include "revolution/gx/GXLighting.h"
#include "revolution/gx/GXStruct.h"
#include "revolution/gx/GXTransform.h"
#include "revolution/gx/GXVert.h"

#include "JSystem/JMATrigonometric.h"

namespace {
    static const f32 sRiderSpeedMin = 100.0f;
    static const f32 sRiderSpeedMax = 150.0f;
    static const f32 sRiderSpinAccel = 50.0f;
    static const f32 sRiderFrictionRate = 0.99f;
    static const s32 sStepRideSpin = 30;
    static const s32 sDisappearStep = 30;
    static const f32 sSensorOffsetY = 300.0f;
    static const f32 sPointIntervalLine = 100.0f;
    static const f32 sDotPointCreateHigh = 0.998f;
    static const f32 sDotPointCreateLow = 0.99f;
    static const f32 sFloatToShortShift = 32768.0f;
    static const f32 sDefaultRadius = 500.0f;
    static const f32 sDotUpdateAxis = 0.99f;
    // static const f32 sPlayerFlowSpeed =
    static const f32 sTexRateU0 = 0.05f;
    static const f32 sTexRateV0 = 0.0001f;
    static const f32 sTexRateU1 = 0.05f;
    static const f32 sTexRateV1 = 0.0001f;
    static const f32 sTexRateU2 = 0.1f;
    static const f32 sTexRateV2 = 0.0001f;
    static const f32 sTexSpeedU0 = 0.002f;
    static const f32 sTexSpeedV0 = -0.005f;
    static const f32 sTexSpeedU1 = -0.0015f;
    static const f32 sTexSpeedV1 = -0.005f;
    static const f32 sTexSpeedV2 = -0.005f;
    static const f32 sIndirectScale = 0.2f;
    // static const f32 sEnvMapScale =
    static const f32 sClippingRadiusOffset = 100.0f;
    static const f32 sDistanceToLow = 10000.0f;
    static const f32 sTexSpeedU2 = 0.0f;

    static GXColor sTevColor1 = {0x28, 0x28, 0x28, 0x14};
    static GXColor sTevColor2 = {0x55, 0x96, 0xBE, 0xFF};

    inline s32 calcNumDrawLinePoints(WaterRoad* pRoad, f32 dotPointCreate) {
        MR::moveCoordToStartPos(pRoad);

        s32 numTestPoints = static_cast< s32 >(MR::getRailTotalLength(pRoad) / ::sPointIntervalLine) + 1;
        f32 testDelta = MR::getRailTotalLength(pRoad) / (numTestPoints - 1);

        TVec3f testRailDir = MR::getRailDirection(pRoad);

        s32 numPoints = 1;
        for (s32 idx = 1; idx < numTestPoints; idx++) {
            MR::moveCoord(pRoad, testDelta);

            if (MR::getRailDirection(pRoad).dot(testRailDir) < dotPointCreate) {
                numPoints++;
                testRailDir.set(MR::getRailDirection(pRoad));
            }
        }

        return numPoints;
    }

    inline f32 toRadian(f32 angle) {
        return angle * (JMath::PI / 180.0f);
    }

    inline void PSMTX44Copy_ASM(const register Mtx44 src, register Mtx44 dst)
    {
        __asm {
            psq_l       fp1, 0(src), 0, 0;
            psq_st      fp1, 0(dst), 0, 0;
            psq_l       fp1, 8(src), 0, 0;
            psq_st      fp1, 8(dst), 0, 0;
            psq_l       fp1, 16(src), 0, 0;
            psq_st      fp1, 16(dst), 0, 0;
            psq_l       fp1, 24(src), 0, 0;
            psq_st      fp1, 24(dst), 0, 0;
            psq_l       fp1, 32(src), 0, 0;
            psq_st      fp1, 32(dst), 0, 0;
            psq_l       fp1, 40(src), 0, 0;
            psq_st      fp1, 40(dst), 0, 0;
            psq_l       fp1, 48(src), 0, 0;
            psq_st      fp1, 48(dst), 0, 0;
            psq_l       fp1, 56(src), 0, 0;
            psq_st      fp1, 56(dst), 0, 0;
        }
    }

    inline void loadTexProjectionMtx(u32 id) {
        Mtx44 cameraProjection;
        PSMTX44Copy_ASM(&MR::getCameraProjectionMtx()[0], &cameraProjection[0]);
        cameraProjection[2][0] = 0.0f;
        cameraProjection[2][1] = 0.0f;
        cameraProjection[2][2] = -1.0f;
        cameraProjection[2][3] = 0.0f;
        cameraProjection[3][0] = 0.0f;
        cameraProjection[3][1] = 0.0f;
        cameraProjection[3][2] = 0.0f;
        cameraProjection[3][3] = 1.0f;
        Mtx matrix;
        PSMTXConcat((const MtxPtr)&cameraProjection[0][0], MR::getCameraViewMtx(), matrix);
        TMtx34f mat;
        mat.identity();
        mat[0][0] = 0.5f;
        mat[0][2] = 0.5f;
        mat[1][1] = -0.5f;
        mat[1][2] = 0.5f;
        PSMTXConcat(mat, matrix, matrix);
        GXLoadTexMtxImm(matrix, id, GX_MTX3x4);
    }
};  // namespace

namespace NrvWaterRoad {
    FULL_NERVE(WaterRoadNrvWait, WaterRoad, Wait);
    FULL_NERVE(WaterRoadNrvWaitInvalid, WaterRoad, WaitInvalid);
    FULL_NERVE(WaterRoadNrvRideStart, WaterRoad, RideStart);
    FULL_NERVE(WaterRoadNrvRideWait, WaterRoad, RideWait);
    FULL_NERVE(WaterRoadNrvRideSpin, WaterRoad, RideSpin);
    FULL_NERVE(WaterRoadNrvDemoStart, WaterRoad, DemoStart);
    FULL_NERVE(WaterRoadNrvDemoGrowUp, WaterRoad, DemoGrowUp);
    FULL_NERVE(WaterRoadNrvDemoWaitEnd, WaterRoad, DemoWaitEnd);
    FULL_NERVE(WaterRoadNrvDisappear, WaterRoad, Disappear);
};  // namespace NrvWaterRoad

WaterRoadModelInfo::WaterRoadModelInfo(WaterRoad* road, bool isLow)
    : mIsLow(isLow), mNumPoints(), mNumLinePoints(), mNumLoopPoints(12), mPoints(), mNormals(), mRailCoords(), mDispListLength(), mDispList() {
    initPoints(road);
    initDisplayList();
}

void WaterRoadModelInfo::initPoints(WaterRoad* pRoad) {
    f32 dotPointCreate = mIsLow ? ::sDotPointCreateLow : ::sDotPointCreateHigh;

    s32 numPoints = ::calcNumDrawLinePoints(pRoad, dotPointCreate);

    mNumLinePoints = numPoints < 2 ? 2 : numPoints;
    mNumPoints = mNumLinePoints * mNumLoopPoints;

    f32 lineDelta = MR::getRailTotalLength(pRoad) / static_cast< s32 >(MR::getRailTotalLength(pRoad) / ::sPointIntervalLine);

    TVec3f up = pRoad->mBaseUp;
    f32 loopInterval = ::toRadian(360.0f / mNumLoopPoints);

    mPoints = new (0x20) TVec3f[mNumPoints];
    mNormals = new (0x20) TVec3s[mNumPoints];
    mRailCoords = new f32[mNumPoints];

    MR::moveCoordToStartPos(pRoad);
    TVec3f front = MR::getRailDirection(pRoad);

    s32 pointIdx = 0;
    for (s32 lineIdx = 0; lineIdx < mNumLinePoints; lineIdx++) {
        if (lineIdx == mNumLinePoints - 1) {
            MR::moveCoordToEndPos(pRoad);
        }

        TVec3f side = MR::getRailDirection(pRoad).cross(up);
        MR::normalize(&side);

        TPos3f loopRot;
        loopRot.identity();
        TVec3f rotAxis = MR::getRailDirection(pRoad) * -1.0f;
        loopRot.setRotate(rotAxis, loopInterval);

        TVec3f raildir = MR::getRailDirection(pRoad);
        PSVECCrossProduct(side, raildir, up);

        mRailCoords[lineIdx] = MR::getRailCoord(pRoad);

        for (s32 loopIdx = 0; loopIdx < mNumLoopPoints; loopIdx++) {
            TVec3f pos = side;
            pos.scale(pRoad->mRadius);
            pos.add(MR::getRailPos(pRoad));
            mPoints[pointIdx].set(pos);
            mNormals[pointIdx].set(side.x * ::sFloatToShortShift, side.y * ::sFloatToShortShift, side.z * ::sFloatToShortShift);
            pointIdx++;
            loopRot.mult(side, side);
        }

        while (pointIdx < mNumPoints && !MR::isRailReachedGoal(pRoad) && MR::getRailDirection(pRoad).dot(front) > dotPointCreate) {
            MR::moveCoord(pRoad, lineDelta);
        }

        front.set(MR::getRailDirection(pRoad));
    }
}

void WaterRoadModelInfo::initDisplayList() {
    MR::ProhibitSchedulerAndInterrupts scheduler(false);

    u32 GDDataSize =
        (((((mIsLow ? sizeof(f32) * 4 : sizeof(f32) * 6) + sizeof(u16) * 2) * 2) * mNumLinePoints) + sizeof(u8) + sizeof(u16)) * mNumLoopPoints;

    u32 length = ((GDDataSize / 32) + 2) * 32;
    mDispList = new (0x20) u8[length];
    DCInvalidateRange(mDispList, length);
    GDLObj obj;
    GDInitGDLObj(&obj, mDispList, length);
    GDSetCurrent(&obj);
    sendGD();
    GDPadCurr32();
    mDispListLength = GDGetGDLObjOffset(&obj);
    DCStoreRange(mDispList, length);
}

void WaterRoadModelInfo::sendGD() const {
    f32 texU0A = 0.0f;
    f32 texU0B = ::sTexRateU0;
    f32 texU1A = 0.0f;
    f32 texU1B = ::sTexRateU1;
    f32 texU2A = 0.0f;
    f32 texU2B = ::sTexRateU2;

    for (s32 loopIdx = 0; loopIdx < mNumLoopPoints; loopIdx++) {
        s32 nextLoopIdx = loopIdx + 1;
        if (loopIdx == mNumLoopPoints - 1) {
            nextLoopIdx = 0;
        }

        u16 numPoints = mNumLinePoints * 2;
        GDWrite_u8(GX_TRIANGLESTRIP);
        GDWrite_u16(numPoints);

        for (s32 lineIdx = 0; lineIdx < mNumLinePoints; lineIdx++) {
            f32 texV0 = mRailCoords[lineIdx] * ::sTexRateV0;
            f32 texV1 = mRailCoords[lineIdx] * ::sTexRateV1;
            f32 texV2 = mRailCoords[lineIdx] * ::sTexRateV2;

            u16 pointIdx = calcPointIndex(lineIdx, loopIdx);
            u16 nextIdx = calcPointIndex(lineIdx, nextLoopIdx);

            GDWrite_u16(pointIdx);
            GDWrite_u16(pointIdx);
            GDWrite_f32(texU0A);
            GDWrite_f32(texV0);
            GDWrite_f32(texU1A);
            GDWrite_f32(texV1);
            if (!mIsLow) {
                GDWrite_f32(texU2A);
                GDWrite_f32(texV2);
            }

            GDWrite_u16(nextIdx);
            GDWrite_u16(nextIdx);
            GDWrite_f32(texU0B);
            GDWrite_f32(texV0);
            GDWrite_f32(texU1B);
            GDWrite_f32(texV1);
            if (!mIsLow) {
                GDWrite_f32(texU2B);
                GDWrite_f32(texV2);
            }
        }

        texU0A = texU0B;
        texU1A = texU1B;
        texU2A = texU2B;
        texU0B += ::sTexRateU0;
        texU1B += ::sTexRateU1;
        texU2B += ::sTexRateU2;
    }
}

void WaterRoadModelInfo::drawGD() const {
    if (mIsLow) {
        loadMaterialLow();
    } else {
        loadMaterialHigh(nullptr);
    }

    GXCallDisplayList(mDispList, mDispListLength);
}

void WaterRoadModelInfo::drawDirect(const WaterRoad* pRoad) const {
    // FIXME: regswaps in const ref section
    // https://decomp.me/scratch/CA4sP

    loadMaterialHigh(pRoad);
    GXClearVtxDesc();
    GXSetVtxDesc(GX_VA_POS, GX_DIRECT);
    GXSetVtxDesc(GX_VA_NRM, GX_DIRECT);
    GXSetVtxDesc(GX_VA_TEX0, GX_DIRECT);
    GXSetVtxDesc(GX_VA_TEX1, GX_DIRECT);
    GXSetVtxDesc(GX_VA_TEX2, GX_DIRECT);

    s32 pointNum = calcDemoDrawPointNum(pRoad);
    u16 numPoints = pointNum * 2;
    if (pointNum < mNumLinePoints) {
        numPoints += 2;
    }

    f32 texU0A = 0.0f;
    f32 texU0B = ::sTexRateU0;
    f32 texU1A = 0.0f;
    f32 texU1B = ::sTexRateU1;
    f32 texU2A = 0.0f;
    f32 texU2B = ::sTexRateU2;

    for (s32 loopIdx = 0; loopIdx < mNumLoopPoints; loopIdx++) {
        s32 nextLoopIdx = loopIdx + 1;
        if (loopIdx == mNumLoopPoints - 1) {
            nextLoopIdx = 0;
        }

        GXBegin(GX_TRIANGLESTRIP, GX_VTXFMT0, numPoints);

        for (s32 lineIdx = 0; lineIdx < pointNum; lineIdx++) {
            f32 texV0 = mRailCoords[lineIdx] * ::sTexRateV0;
            f32 texV1 = mRailCoords[lineIdx] * ::sTexRateV1;
            f32 texV2 = mRailCoords[lineIdx] * ::sTexRateV2;

            const TVec3f& pointA = mPoints[calcPointIndex(lineIdx, loopIdx)];
            const TVec3f& pointB = mPoints[calcPointIndex(lineIdx, nextLoopIdx)];

            const TVec3s& normA = mNormals[calcPointIndex(lineIdx, loopIdx)];
            const TVec3s& normB = mNormals[calcPointIndex(lineIdx, nextLoopIdx)];

            GXPosition3f32(pointA.x, pointA.y, pointA.z);
            GXPosition3s16(normA.x, normA.y, normA.z);
            GXTexCoord2f32(texU0A, texV0);
            GXTexCoord2f32(texU1A, texV1);
            GXTexCoord2f32(texU2A, texV2);

            GXPosition3f32(pointB.x, pointB.y, pointB.z);
            GXPosition3s16(normB.x, normB.y, normB.z);
            GXTexCoord2f32(texU0B, texV0);
            GXTexCoord2f32(texU1B, texV1);
            GXTexCoord2f32(texU2B, texV2);
        }

        if (pointNum < mNumLinePoints) {
            f32 t = (MR::getRailCoord(pRoad) - mRailCoords[pointNum - 1]) / (mRailCoords[pointNum] - mRailCoords[pointNum - 1]);

            // FIXME: regswap
            const TVec3s& n1 = mNormals[calcPointIndex(pointNum - 1, loopIdx)];
            const TVec3s& n2 = mNormals[calcPointIndex(pointNum, loopIdx)];
            const TVec3s& n3 = mNormals[calcPointIndex(pointNum - 1, nextLoopIdx)];
            const TVec3s& n4 = mNormals[calcPointIndex(pointNum, nextLoopIdx)];

            const TVec3f& v1 = mPoints[calcPointIndex(pointNum - 1, loopIdx)];
            const TVec3f& v2 = mPoints[calcPointIndex(pointNum, loopIdx)];
            const TVec3f& v3 = mPoints[calcPointIndex(pointNum - 1, nextLoopIdx)];
            const TVec3f& v4 = mPoints[calcPointIndex(pointNum, nextLoopIdx)];

            TVec3f vA = v1 * (1.0f - t) + v2 * t;
            TVec3f vB = v3 * (1.0f - t) + v4 * t;

            TVec3s nA;
            nA.x = n1.x * (1.0f - t) + n2.x * t;
            nA.y = n1.y * (1.0f - t) + n2.y * t;
            nA.z = n1.z * (1.0f - t) + n2.z * t;

            TVec3s nB;
            nB.x = n3.x * (1.0f - t) + n4.x * t;
            nB.y = n3.y * (1.0f - t) + n4.y * t;
            nB.z = n3.z * (1.0f - t) + n4.z * t;

            f32 texV0 = MR::getRailCoord(pRoad) * ::sTexRateV0;
            f32 texV1 = MR::getRailCoord(pRoad) * ::sTexRateV1;
            f32 texV2 = MR::getRailCoord(pRoad) * ::sTexRateV2;

            GXPosition3f32(vA.x, vA.y, vA.z);
            GXPosition3s16(nA.x, nA.y, nA.z);
            GXTexCoord2f32(texU0A, texV0);
            GXTexCoord2f32(texU1A, texV1);
            GXTexCoord2f32(texU2A, texV2);

            GXPosition3f32(vB.x, vB.y, vB.z);
            GXPosition3s16(nB.x, nB.y, nB.z);
            GXTexCoord2f32(texU0B, texV0);
            GXTexCoord2f32(texU1B, texV1);
            GXTexCoord2f32(texU2B, texV2);
        }

        texU0A = texU0B;
        texU1A = texU1B;
        texU2A = texU2B;
        texU0B += ::sTexRateU0;
        texU1B += ::sTexRateU1;
        texU2B += ::sTexRateU2;
    }
}

s32 WaterRoadModelInfo::calcDemoDrawPointNum(const WaterRoad* pRoad) const {
    for (s32 pointNum = 1; pointNum < mNumLinePoints; pointNum++) {
        if (MR::getRailCoord(pRoad) < mRailCoords[pointNum]) {
            return pointNum;
        }
    }

    return mNumLinePoints;
}

void WaterRoadModelInfo::loadMaterialHigh(const WaterRoad* pRoad) const {
    GXSetVtxAttrFmt(GX_VTXFMT0, GX_VA_POS, GX_POS_XYZ, GX_F32, 0);
    GXSetVtxAttrFmt(GX_VTXFMT0, GX_VA_NRM, GX_POS_XY, GX_S16, 16);
    GXSetVtxAttrFmt(GX_VTXFMT0, GX_VA_TEX0, GX_POS_XYZ, GX_F32, 0);
    GXSetVtxAttrFmt(GX_VTXFMT0, GX_VA_TEX1, GX_POS_XYZ, GX_F32, 0);
    GXSetVtxAttrFmt(GX_VTXFMT0, GX_VA_TEX2, GX_POS_XYZ, GX_F32, 0);
    GXClearVtxDesc();
    GXSetVtxDesc(GX_VA_POS, GX_INDEX16);
    GXSetVtxDesc(GX_VA_NRM, GX_INDEX16);
    GXSetVtxDesc(GX_VA_TEX0, GX_DIRECT);
    GXSetVtxDesc(GX_VA_TEX1, GX_DIRECT);
    GXSetVtxDesc(GX_VA_TEX2, GX_DIRECT);
    GXSetArray(GX_VA_POS, mPoints, sizeof(TVec3f));
    GXSetArray(GX_VA_NRM, mNormals, sizeof(TVec3s));
    GXLoadPosMtxImm(MR::getCameraViewMtx(), 0);
    GXLoadNrmMtxImm(MR::getCameraViewMtx(), 0);
    GXSetCurrentMtx(0);
    GXSetNumChans(0);
    GXSetNumTexGens(5);
    GXSetTexCoordGen2(GX_TEXCOORD0, GX_TG_MTX2x4, GX_TG_TEX0, 0x1E, 0, 0x7D);
    GXSetTexCoordGen2(GX_TEXCOORD1, GX_TG_MTX2x4, GX_TG_TEX1, 0x21, 0, 0x7D);
    GXSetTexCoordGen2(GX_TEXCOORD2, GX_TG_MTX2x4, GX_TG_TEX2, 0x24, 0, 0x7D);
    GXSetTexCoordGen2(GX_TEXCOORD3, GX_TG_MTX3x4, GX_TG_POS, 0x27, 0, 0x7D);
    GXSetTexCoordGen2(GX_TEXCOORD4, GX_TG_MTX2x4, GX_TG_NRM, 0x2A, 0, 0x7D);
    ::loadTexProjectionMtx(0x27);

    TPos3f pos;
    pos.identity();
    pos.set(MR::getCameraViewMtx());
    pos.zeroTrans();

    TPos3f mtx;
    mtx.identity();
    mtx.scale(1.0f);

    static Mtx qMtx2 = {0.5f, 0.0f, 0.0f, 0.5f, 0.0f, -0.5f, 0.0f, 0.5f, 0.0f, 0.0f, 1.0f, 0.0f};

    PSMTXConcat(pos.mMtx, mtx, pos.mMtx);
    PSMTXConcat(pos.mMtx, qMtx2, pos.mMtx);
    GXLoadTexMtxImm(pos.mMtx, 0x2A, GX_MTX2x4);
    GXSetNumIndStages(1);
    GXSetIndTexOrder(GX_INDTEXSTAGE0, GX_TEXCOORD2, GX_TEXMAP1);
    GXSetTevIndWarp(GX_TEVSTAGE3, GX_INDTEXSTAGE0, GX_TRUE, GX_FALSE, GX_ITM_0);

    f32 indirectScale = ::sIndirectScale;

    f32 indMtx[2][3];
    indMtx[0][1] = 0.0f;
    indMtx[0][0] = indirectScale;
    indMtx[0][2] = 0.0f;
    indMtx[1][0] = 0.0f;
    indMtx[1][1] = indirectScale;
    indMtx[1][2] = 0.0f;

    GXSetIndTexMtx(GX_ITM_0, indMtx, 0);
    GXSetNumTevStages(5);

    GXSetTevColor(GX_TEVREG0, sTevColor1);
    GXSetTevColor(GX_TEVREG1, sTevColor2);

    if (pRoad != nullptr) {
        GXSetTevColor(GX_TEVREG2, (GXColor){ 0xFF, 0xFF, 0xFF, pRoad->mAlpha });
    }

    GXSetTevOrder(GX_TEVSTAGE0, GX_TEXCOORD0, GX_TEXMAP0, GX_COLOR_NULL);
    GXSetTevColorIn(GX_TEVSTAGE0, GX_CC_TEXC, GX_CC_ZERO, GX_CC_ZERO, GX_CC_ZERO);
    GXSetTevColorOp(GX_TEVSTAGE0, GX_TEV_ADD, GX_TB_ZERO, GX_CS_SCALE_1, GX_FALSE, GX_TEVPREV);
    GXSetTevAlphaIn(GX_TEVSTAGE0, GX_CA_ZERO, GX_CA_ZERO, GX_CA_ZERO, GX_CA_ZERO);
    GXSetTevAlphaOp(GX_TEVSTAGE0, GX_TEV_ADD, GX_TB_ZERO, GX_CS_SCALE_1, GX_FALSE, GX_TEVPREV);
    GXSetTevOrder(GX_TEVSTAGE1, GX_TEXCOORD1, GX_TEXMAP0, GX_COLOR_NULL);
    GXSetTevColorIn(GX_TEVSTAGE1, GX_CC_ZERO, GX_CC_TEXC, GX_CC_CPREV, GX_CC_ZERO);
    GXSetTevColorOp(GX_TEVSTAGE1, GX_TEV_ADD, GX_TB_ZERO, GX_CS_DIVIDE_2, GX_FALSE, GX_TEVPREV);
    GXSetTevAlphaIn(GX_TEVSTAGE1, GX_CA_ZERO, GX_CA_ZERO, GX_CA_ZERO, GX_CA_ZERO);
    GXSetTevAlphaOp(GX_TEVSTAGE1, GX_TEV_ADD, GX_TB_ZERO, GX_CS_SCALE_2, GX_FALSE, GX_TEVPREV);
    GXSetTevOrder(GX_TEVSTAGE2, GX_TEXCOORD_NULL, GX_TEXMAP_NULL, GX_COLOR_NULL);
    GXSetTevColorIn(GX_TEVSTAGE2, GX_CC_CPREV, GX_CC_A0, GX_CC_C0, GX_CC_CPREV);
    GXSetTevColorOp(GX_TEVSTAGE2, GX_TEV_COMP_R8_EQ, GX_TB_ZERO, GX_CS_SCALE_1, GX_FALSE, GX_TEVPREV);
    GXSetTevAlphaIn(GX_TEVSTAGE2, GX_CA_KONST, GX_CA_ZERO, GX_CA_ZERO, GX_CA_ZERO);
    GXSetTevAlphaOp(GX_TEVSTAGE2, GX_TEV_ADD, GX_TB_ZERO, GX_CS_SCALE_1, GX_FALSE, GX_TEVPREV);
    GXSetTevOrder(GX_TEVSTAGE3, GX_TEXCOORD3, GX_TEXMAP2, GX_COLOR_NULL);
    GXSetTevColorIn(GX_TEVSTAGE3, GX_CC_ZERO, GX_CC_TEXC, GX_CC_C1, GX_CC_CPREV);
    GXSetTevColorOp(GX_TEVSTAGE3, GX_TEV_ADD, GX_TB_ZERO, GX_CS_SCALE_1, GX_TRUE, GX_TEVPREV);
    GXSetTevAlphaIn(GX_TEVSTAGE3, GX_CA_APREV, GX_CA_ZERO, GX_CA_ZERO, GX_CA_ZERO);
    GXSetTevAlphaOp(GX_TEVSTAGE3, GX_TEV_ADD, GX_TB_ZERO, GX_CS_SCALE_1, GX_FALSE, GX_TEVPREV);
    GXSetTevOrder(GX_TEVSTAGE4, GX_TEXCOORD4, GX_TEXMAP3, GX_COLOR_NULL);
    GXSetTevColorIn(GX_TEVSTAGE4, GX_CC_TEXC, GX_CC_ZERO, GX_CC_ZERO, GX_CC_CPREV);
    GXSetTevColorOp(GX_TEVSTAGE4, GX_TEV_ADD, GX_TB_ZERO, GX_CS_SCALE_1, GX_TRUE, GX_TEVPREV);

    if (pRoad == nullptr || pRoad->mAlpha == 0xFF) {
        GXSetTevAlphaIn(GX_TEVSTAGE4, GX_CA_KONST, GX_CA_ZERO, GX_CA_ZERO, GX_CA_ZERO);
    } else {
        GXSetTevAlphaIn(GX_TEVSTAGE4, GX_CA_A2, GX_CA_ZERO, GX_CA_ZERO, GX_CA_ZERO);
    }

    GXSetTevAlphaOp(GX_TEVSTAGE4, GX_TEV_ADD, GX_TB_ZERO, GX_CS_SCALE_1, GX_TRUE, GX_TEVPREV);
    GXSetBlendMode(GX_BM_BLEND, GX_BL_SRCALPHA, GX_BL_INVSRCALPHA, GX_LO_NOOP);
    GXSetAlphaCompare(GX_GREATER, 0, GX_AOP_OR, GX_GREATER, 0);
    GXSetZMode(GX_TRUE, GX_LEQUAL, GX_TRUE);
    GXSetZCompLoc(GX_TRUE);
    GXSetCullMode(GX_CULL_NONE);
    GXSetClipMode(GX_CLIP_ENABLE);
}

void WaterRoadModelInfo::loadMaterialLow() const {
    GXSetVtxAttrFmt(GX_VTXFMT0, GX_VA_POS, GX_POS_XYZ, GX_F32, 0);
    GXSetVtxAttrFmt(GX_VTXFMT0, GX_VA_NRM, GX_POS_XY, GX_S16, 0x10);
    GXSetVtxAttrFmt(GX_VTXFMT0, GX_VA_TEX0, GX_POS_XYZ, GX_F32, 0);
    GXSetVtxAttrFmt(GX_VTXFMT0, GX_VA_TEX1, GX_POS_XYZ, GX_F32, 0);
    GXClearVtxDesc();
    GXSetVtxDesc(GX_VA_POS, GX_INDEX16);
    GXSetVtxDesc(GX_VA_NRM, GX_INDEX16);
    GXSetVtxDesc(GX_VA_TEX0, GX_DIRECT);
    GXSetVtxDesc(GX_VA_TEX1, GX_DIRECT);
    GXSetArray(GX_VA_POS, mPoints, sizeof(TVec3f));
    GXSetArray(GX_VA_NRM, mNormals, sizeof(TVec3s));
    GXLoadPosMtxImm(MR::getCameraViewMtx(), 0);
    GXLoadNrmMtxImm(MR::getCameraViewMtx(), 0);
    GXSetCurrentMtx(0);
    GXSetNumChans(0);
    GXSetNumTexGens(4);
    GXSetTexCoordGen2(GX_TEXCOORD0, GX_TG_MTX2x4, GX_TG_TEX0, 0x1E, GX_FALSE, 0x7D);
    GXSetTexCoordGen2(GX_TEXCOORD1, GX_TG_MTX2x4, GX_TG_TEX1, 0x21, GX_FALSE, 0x7D);
    GXSetTexCoordGen2(GX_TEXCOORD2, GX_TG_MTX3x4, GX_TG_POS, 0x24, GX_FALSE, 0x7D);
    GXSetTexCoordGen2(GX_TEXCOORD3, GX_TG_MTX2x4, GX_TG_NRM, 0x27, GX_FALSE, 0x7D);

    ::loadTexProjectionMtx(0x24);

    TPos3f pos;
    pos.identity();
    pos.set(MR::getCameraViewMtx());
    pos.zeroTrans();

    TPos3f mtx;
    mtx.identity();
    mtx.scale(1.0f);

    static Mtx qMtx2 = {0.5f, 0.0f, 0.0f, 0.5f, 0.0f, -0.5f, 0.0f, 0.5f, 0.0f, 0.0f, 1.0f, 0.0f};

    PSMTXConcat(pos.mMtx, mtx, pos.mMtx);
    PSMTXConcat(pos.mMtx, qMtx2, pos.mMtx);
    GXLoadTexMtxImm(pos.mMtx, 0x27, GX_MTX2x4);

    GXSetNumIndStages(0);
    GXSetNumTevStages(4);
    GXSetTevColor(GX_TEVREG0, sTevColor1);
    GXSetTevColor(GX_TEVREG1, sTevColor2);
    GXSetTevOrder(GX_TEVSTAGE0, GX_TEXCOORD0, GX_TEXMAP0, GX_COLOR_NULL);
    GXSetTevColorIn(GX_TEVSTAGE0, GX_CC_TEXC, GX_CC_ZERO, GX_CC_ZERO, GX_CC_ZERO);
    GXSetTevColorOp(GX_TEVSTAGE0, GX_TEV_ADD, GX_TB_ZERO, GX_CS_SCALE_1, GX_FALSE, GX_TEVPREV);
    GXSetTevAlphaIn(GX_TEVSTAGE0, GX_CA_ZERO, GX_CA_ZERO, GX_CA_ZERO, GX_CA_ZERO);
    GXSetTevAlphaOp(GX_TEVSTAGE0, GX_TEV_ADD, GX_TB_ZERO, GX_CS_SCALE_1, GX_FALSE, GX_TEVPREV);
    GXSetTevOrder(GX_TEVSTAGE1, GX_TEXCOORD1, GX_TEXMAP0, GX_COLOR_NULL);
    GXSetTevColorIn(GX_TEVSTAGE1, GX_CC_ZERO, GX_CC_TEXC, GX_CC_CPREV, GX_CC_ZERO);
    GXSetTevColorOp(GX_TEVSTAGE1, GX_TEV_ADD, GX_TB_ZERO, GX_CS_DIVIDE_2, GX_FALSE, GX_TEVPREV);
    GXSetTevAlphaIn(GX_TEVSTAGE1, GX_CA_ZERO, GX_CA_ZERO, GX_CA_ZERO, GX_CA_ZERO);
    GXSetTevAlphaOp(GX_TEVSTAGE1, GX_TEV_ADD, GX_TB_ZERO, GX_CS_SCALE_2, GX_FALSE, GX_TEVPREV);
    GXSetTevOrder(GX_TEVSTAGE2, GX_TEXCOORD2, GX_TEXMAP2, GX_COLOR_NULL);
    GXSetTevColorIn(GX_TEVSTAGE2, GX_CC_ZERO, GX_CC_TEXC, GX_CC_C1, GX_CC_CPREV);
    GXSetTevColorOp(GX_TEVSTAGE2, GX_TEV_ADD, GX_TB_ZERO, GX_CS_SCALE_1, GX_TRUE, GX_TEVPREV);
    GXSetTevAlphaIn(GX_TEVSTAGE2, GX_CA_ZERO, GX_CA_ZERO, GX_CA_ZERO, GX_CA_ZERO);
    GXSetTevAlphaOp(GX_TEVSTAGE2, GX_TEV_ADD, GX_TB_ZERO, GX_CS_SCALE_1, GX_FALSE, GX_TEVPREV);
    GXSetTevOrder(GX_TEVSTAGE3, GX_TEXCOORD3, GX_TEXMAP3, GX_COLOR_NULL);
    GXSetTevColorIn(GX_TEVSTAGE3, GX_CC_TEXC, GX_CC_ZERO, GX_CC_ZERO, GX_CC_CPREV);
    GXSetTevColorOp(GX_TEVSTAGE3, GX_TEV_ADD, GX_TB_ZERO, GX_CS_SCALE_1, GX_TRUE, GX_TEVPREV);
    GXSetTevAlphaIn(GX_TEVSTAGE3, GX_CA_KONST, GX_CA_ZERO, GX_CA_ZERO, GX_CA_ZERO);
    GXSetTevAlphaOp(GX_TEVSTAGE3, GX_TEV_ADD, GX_TB_ZERO, GX_CS_SCALE_1, GX_TRUE, GX_TEVPREV);
    GXSetBlendMode(GX_BM_BLEND, GX_BL_SRCALPHA, GX_BL_INVSRCALPHA, GX_LO_NOOP);
    GXSetAlphaCompare(GX_GREATER, 0, GX_AOP_OR, GX_GREATER, 0);
    GXSetZMode(GX_TRUE, GX_LEQUAL, GX_TRUE);
    GXSetZCompLoc(GX_TRUE);
    GXSetCullMode(GX_CULL_NONE);
    GXSetClipMode(GX_CLIP_ENABLE);
}

WaterRoad::WaterRoad(const char* pName)
    : LiveActor(pName), mAllowReuse(false), mModelInfoHigh(), mModelInfoLow(), mRadius(::sDefaultRadius), mAlpha(0xFF), mBaseUp(0.0f, 1.0f, 0.0f), mRider(),
      mTexUV0(0.0f, 0.0f), mTexUV1(0.0f, 0.0f), mTexUV2(0.0f, 0.0f), mTexture(), mIndirect(), mEnvMap(), mClippingCenter(0.0f, 0.0f, 0.0f),
      mCamRailNearestPos(0.0f, 0.0f, 0.0f), mSoundObj(), mCameraInfo(), mMinSpeed(sRiderSpeedMin), mMaxSpeed(sRiderSpeedMax) {
    mBaseMtx.identity();
    mRiderMtx.identity();
    mBoundingBox.mPointMax.zero();
    mBoundingBox.mPointMin.zero();
}

void WaterRoad::init(const JMapInfoIter& rIter) {
    MR::connectToScene(this, MR::MovementType_MapObj, MR::CategoryList_Null, MR::CategoryList_Null, MR::DrawType_WaterRoad);
    MR::initDefaultPos(this, rIter);
    MR::makeMtxTR(mBaseMtx, this);
    mBaseMtx.getYDir(mBaseUp);
    MR::getJMapInfoArg0NoInit(rIter, &mAllowReuse);
    MR::getJMapInfoArg1NoInit(rIter, &mRadius);
    MR::getJMapInfoArg2NoInit(rIter, &mMinSpeed);
    MR::getJMapInfoArg3NoInit(rIter, &mMaxSpeed);
    initRailRider(rIter);

    if (MR::getRailDirection(this).dot(mBaseUp) >= ::sDotUpdateAxis) {
        TRot3f stack_8;
        stack_8.identity();
        MR::makeMtxTR(stack_8, this);
        stack_8.getXDir(mBaseUp);
    }
    updateDemo(0.0f);

    initNerve(&NrvWaterRoad::WaterRoadNrvWait::sInstance, 0);

    initHitSensor(1);
    MR::addHitSensorCallbackBinder(this, "Binder", 8, mRadius);

    mModelInfoHigh = new WaterRoadModelInfo(this, false);
    mModelInfoLow = new WaterRoadModelInfo(this, true);

    MR::moveCoordToStartPos(this);

    initEffectKeeper(0, "WaterRoad", false);
    MR::setEffectHostMtx(this, "Top", mBaseMtx);
    MR::setEffectHostMtx(this, "End", mBaseMtx);

    mTexture = MR::loadTexFromArc(this, "WaterWave.arc", "Water.bti");
    mIndirect = MR::loadTexFromArc(this, "WaterWave.arc", "WaterIndirect.bti");
    mEnvMap = MR::loadTexFromArc(this, "WaterWave.arc", "EnvMap.bti");

    initSound(3, false, nullptr, TVec3f(0.0f));
    mSoundObj = new AudSoundObject(&mCamRailNearestPos, 4, MR::getCurrentHeap());

    MR::calcBoundingBox(this, &mBoundingBox, ::sClippingRadiusOffset);
    TVec3f p(mRadius);
    mBoundingBox.mPointMin.sub(p);
    mBoundingBox.mPointMax.add(p);
    JMAVECLerp(mBoundingBox.mPointMax, mBoundingBox.mPointMin, mClippingCenter, 0.5f);
    MR::setClippingTypeSphere(this, PSVECDistance(mClippingCenter, mBoundingBox.mPointMin) + ::sClippingRadiusOffset, &mClippingCenter);
    MR::setClippingFarMax(this);

    bool hasDemo = MR::tryRegisterDemoCast(this, rIter);
    MR::initMultiActorCamera(this, rIter, &mCameraInfo, "バインド中");
    MR::useStageSwitchWriteA(this, rIter);
    if (hasDemo) {
        makeActorDead();
    } else {
        if (!MR::useStageSwitchAwake(this, rIter))
            makeActorAppeared();
    }
}

void WaterRoad::appear() {
    LiveActor::appear();
    MR::invalidateClipping(this);
    mTranslation.set(MR::getRailPointPosStart(this));

    if (MR::isValidSwitchA(this)) {
        MR::onSwitchA(this);
    }

    if (MR::isDemoPartActive("ウォーターロード成長")) {
        setNerve(&NrvWaterRoad::WaterRoadNrvDemoGrowUp::sInstance);
    } else {
        setNerve(&NrvWaterRoad::WaterRoadNrvDemoStart::sInstance);
    }
}

void WaterRoad::exeWait() {
    if (MR::isFirstStep(this)) {
        MR::validateClipping(this);
    }
}

void WaterRoad::exeWaitInvalid() {
    if (MR::isStep(this, 180)) {
        setNerve(&NrvWaterRoad::WaterRoadNrvWait::sInstance);
    }
}

void WaterRoad::exeRideStart() {
    if (MR::isFirstStep(this)) {
        MR::startBckPlayer("WaterRoadIn", static_cast< const char* >(nullptr));
        MR::startSound(mRider, "SE_OJ_WATER_ROAD_BIND_IN");
    }

    if (updateRide()) {
        return;
    }

    if (MR::isCorePadSwing(0)) {
        setNerve(&NrvWaterRoad::WaterRoadNrvRideSpin::sInstance);
        return;
    }

    if (MR::isBckStopped(mRider)) {
        setNerve(&NrvWaterRoad::WaterRoadNrvRideWait::sInstance);
    }
}

void WaterRoad::exeRideWait() {
    if (MR::isFirstStep(this)) {
        MR::startBckPlayer("SwimFlutter", static_cast< const char* >(nullptr));
    }

    if (updateRide()) {
        return;
    }

    if (MR::isCorePadSwing(0)) {
        setNerve(&NrvWaterRoad::WaterRoadNrvRideSpin::sInstance);
    }
}

void WaterRoad::exeRideSpin() {
    if (MR::isFirstStep(this)) {
        MR::startBckPlayer("SwimSpin", static_cast< const char* >(nullptr));
        MR::startSound(mRider, "SE_PM_TORNADE_IN_WATER_ST");
        f32 speed = MR::getRailCoordSpeed(this);
        speed += ::sRiderSpinAccel;
        MR::setRailCoordSpeed(this, speed);
    }

    MR::startLevelSound(mRider, "SE_PM_LV_TORNADE_IN_WATER");

    if (updateRide()) {
        return;
    }

    if (MR::isStep(this, ::sStepRideSpin)) {
        setNerve(&NrvWaterRoad::WaterRoadNrvRideWait::sInstance);
    }
}

void WaterRoad::exeDemoStart() {
    MR::startLevelSound(this, "SE_OJ_LV_WATER_ROAD_APPEAR");
    MR::startAtmosphereLevelSE("SE_AT_LV_EARTHQUAKE", -1, -1);

    if (MR::isDemoPartLastStep("ウォーターロード出現")) {
        setNerve(&NrvWaterRoad::WaterRoadNrvDemoGrowUp::sInstance);
    }
}

void WaterRoad::exeDemoGrowUp() {
    if (MR::isFirstStep(this)) {
        MR::emitEffect(this, "Top");
        MR::startSound(this, "SE_OJ_WATER_ROAD_GROW_START");
        MR::shakeCameraStrong();
        MR::tryRumblePadVeryStrong(this, 0);
    }

    updateDemo(MR::calcDemoPartStepRate("ウォーターロード成長"));

    MR::startLevelSound(this, "SE_OJ_LV_WATER_ROAD_GROW");
    MR::startAtmosphereLevelSE("SE_AT_LV_EARTHQUAKE", -1, -1);

    if (MR::isDemoPartLastStep("ウォーターロード成長")) {
        MR::deleteEffect(this, "Top");
        MR::emitEffect(this, "End");
        setNerve(&NrvWaterRoad::WaterRoadNrvDemoWaitEnd::sInstance);
    }
}

void WaterRoad::exeDemoWaitEnd() {
    if (!MR::isDemoActive()) {
        MR::showPlayer();
        setNerve(&NrvWaterRoad::WaterRoadNrvWait::sInstance);
    }
}

void WaterRoad::exeDisappear() {
    mAlpha = MR::calcNerveValue(this, ::sDisappearStep, 255.0f, 0.0f);

    if (MR::isStep(this, ::sDisappearStep - 20)) {
        MR::tryDeleteEffect(this, "End");
    }

    if (MR::isStep(this, ::sDisappearStep)) {
        kill();
    }
}

void WaterRoad::movement() {
    // FIXME: float regswaps in repeat
    // https://decomp.me/scratch/6VOUU

    if (!MR::isValidMovement(this)) {
        return;
    }

    LiveActor::movement();

    mTexUV0.x = MR::repeat(mTexUV0.x + ::sTexSpeedU0, 0.0f, 1.0f);
    mTexUV0.y = MR::repeat(mTexUV0.y + ::sTexSpeedV0, 0.0f, 1.0f);
    mTexUV1.x = MR::repeat(mTexUV1.x + ::sTexSpeedU1, 0.0f, 1.0f);
    mTexUV1.y = MR::repeat(mTexUV1.y + ::sTexSpeedV1, 0.0f, 1.0f);
    mTexUV2.x = MR::repeat(mTexUV2.x + ::sTexSpeedU2, 0.0f, 1.0f);
    mTexUV2.y = MR::repeat(mTexUV2.y + ::sTexSpeedV2, 0.0f, 1.0f);

    MR::calcNearestRailPos(&mCamRailNearestPos, this, MR::getCamPos());
    MR::startSoundObjectLevel(mSoundObj, "SE_AT_LV_WATER_ROAD", -1);
    mSoundObj->process();
}

void WaterRoad::updateHitSensor(HitSensor* pSensor) {
    if (!isNerve(&NrvWaterRoad::WaterRoadNrvWait::sInstance)) {
        return;
    }

    MR::calcNearestRailPos(&pSensor->mPosition, this, *MR::getPlayerPos());

    // This doesn't work
    //TVec3f gravityDir;
    //MR::calcGravityVector(this, &gravityDir, nullptr, 0);
    //TVec3f railStart = MR::getRailPointPosStart(this);
    //TVec3f relativePos = pSensor->mPosition - railStart;
    //float currentDistanceDown = relativePos.dot(gravityDir);
    //float targetDistanceDown = -::sSensorOffsetY;

    //if (currentDistanceDown > targetDistanceDown) {
    //    float correctionAmount = targetDistanceDown - currentDistanceDown;
    //    pSensor->mPosition += gravityDir * correctionAmount;
    //}
    //OSReport("NearestRailPos: %ff %ff %ff\n", pSensor->mPosition.x, pSensor->mPosition.y, pSensor->mPosition.z);
}

bool WaterRoad::receiveOtherMsg(u32 msg, HitSensor* pSender, HitSensor* pReceiver) {
    if (MR::isMsgAutoRushBegin(msg)) {
        if (!isNerve(&NrvWaterRoad::WaterRoadNrvWait::sInstance)) {
            return false;
        }

        mRider = MR::getSensorHost(pSender);
        MR::moveCoordToNearestPos(this, mRider->mTranslation);
        MR::startMultiActorCameraTargetPlayer(this, mCameraInfo, "バインド中", -1);
        setNerve(&NrvWaterRoad::WaterRoadNrvRideStart::sInstance);
        return true;
    }

    if (MR::isMsgUpdateBaseMtx(msg)) {
        TVec3f front;
        mRiderMtx.getZDir(front);
        MR::makeMtxUpFrontPos(&mRiderMtx, MR::getRailDirection(this), front, MR::getRailPos(this));
        MR::setBaseTRMtx(mRider, mRiderMtx);
        return true;
    }

    return false;
}

bool WaterRoad::updateRide() {
    f32 speed = MR::getRailCoordSpeed(this);
    speed *= ::sRiderFrictionRate;
    MR::moveCoordAndFollowTrans(this, MR::clamp(speed, mMinSpeed, mMaxSpeed));

    if (mRider != nullptr) {
        MR::startLevelSound(mRider, "SE_OJ_LV_WATER_ROAD_BIND");
    }

    if (MR::isRailReachedGoal(this)) {
        MR::startSound(mRider, "SE_OJ_WATER_ROAD_BIND_END");
        MR::startSound(mRider, "SE_PV_JUMP_M");
        TVec3f dismount = TVec3f(MR::getRailDirection(this));
        MR::normalize(&dismount);
        dismount.scale(speed);
        MR::endBindAndPlayerJump(this, dismount, 0);
        MR::endMultiActorCamera(this, mCameraInfo, "バインド中", true, -1);
        mRider = nullptr;

        if (mAllowReuse)
            setNerve(&NrvWaterRoad::WaterRoadNrvWaitInvalid::sInstance);
        else
            setNerve(&NrvWaterRoad::WaterRoadNrvDisappear::sInstance);
        return true;
    }

    return false;
}

void WaterRoad::updateDemo(f32 demoRate) {
    MR::setRailCoord(this, MR::getEaseOutValue(demoRate, 0.0f, 1.0f, 1.0f) * MR::getRailTotalLength(this));

    mTranslation.set(MR::getRailPos(this));
    TVec3f front;
    mBaseMtx.getZDir(front);
    MR::makeMtxUpFront(&mBaseMtx, MR::getRailDirection(this), front);
    mBaseMtx.setTrans(mTranslation);
}

void WaterRoad::draw() const {
    if (!MR::isValidDraw(this)) {
        return;
    }

    if (isNerve(&NrvWaterRoad::WaterRoadNrvDemoStart::sInstance)) {
        return;
    }

    TPos3f mtx;
    mtx.identity();
    mtx[0][2] = mTexUV0.x;
    mtx[1][2] = mTexUV0.y;
    GXLoadTexMtxImm(mtx, 0x1E, GX_MTX2x4);
    mtx[0][2] = mTexUV1.x;
    mtx[1][2] = mTexUV1.y;
    GXLoadTexMtxImm(mtx, 0x21, GX_MTX2x4);

    mTexture->load(GX_TEXMAP0);
    JUTTexture screenTex(MR::getScreenResTIMG(), 0);
    screenTex.load(GX_TEXMAP2);
    mEnvMap->load(GX_TEXMAP3);

    if (isNerve(&NrvWaterRoad::WaterRoadNrvDemoGrowUp::sInstance) || isNerve(&NrvWaterRoad::WaterRoadNrvDemoWaitEnd::sInstance)) {
        mIndirect->load(GX_TEXMAP1);
        mtx[0][2] = mTexUV2.x;
        mtx[1][2] = mTexUV2.y;
        GXLoadTexMtxImm(mtx, 0x24, GX_MTX2x4);
        mModelInfoHigh->drawDirect(this);
        return;
    }

    TVec3f boundedPos;
    boundedPos.x = MR::clamp(MR::getPlayerPos()->x, mBoundingBox.mPointMin.x, mBoundingBox.mPointMax.x);
    boundedPos.y = MR::clamp(MR::getPlayerPos()->y, mBoundingBox.mPointMin.y, mBoundingBox.mPointMax.y);
    boundedPos.z = MR::clamp(MR::getPlayerPos()->z, mBoundingBox.mPointMin.z, mBoundingBox.mPointMax.z);

    if (PSVECDistance(*MR::getPlayerPos(), boundedPos) > ::sDistanceToLow) {
        mModelInfoLow->drawGD();
    } else {
        mIndirect->load(GX_TEXMAP1);
        mtx[0][2] = mTexUV2.x;
        mtx[1][2] = mTexUV2.y;
        GXLoadTexMtxImm(mtx, 0x24, GX_MTX2x4);
        if (isNerve(&NrvWaterRoad::WaterRoadNrvDisappear::sInstance))
            mModelInfoHigh->drawDirect(this);
        else
            mModelInfoHigh->drawGD();
    }
}
