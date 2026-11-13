#pragma once

#include "Game/LiveActor/LiveActor.h"
#include "Game/Util/JMapInfo.h"
#include "JSystem/JGeometry.h"

class JUTTexture;
class WaterRoadModelInfo;
class AudSoundObject;
class ActorCameraInfo;

class WaterRoad : public LiveActor {
public:
    WaterRoad(const char*);

    virtual void init(const JMapInfoIter& rIter);
    virtual void appear();
    virtual void movement();
    virtual void draw() const;
    virtual void updateHitSensor(HitSensor* pSensor);
    virtual bool receiveOtherMsg(u32 msg, HitSensor* pSender, HitSensor* pReceiver);

    void exeWait();
    void exeWaitInvalid();
    void exeRideStart();
    void exeRideWait();
    void exeRideSpin();
    void exeDemoStart();
    void exeDemoGrowUp();
    void exeDemoWaitEnd();
    void exeDisappear();

    bool updateRide();
    void updateDemo(f32);

    bool mAllowReuse;  // arg0, unused?
    WaterRoadModelInfo* mModelInfoHigh;
    WaterRoadModelInfo* mModelInfoLow;
    f32 mRadius;
    f32 mMinSpeed;
    f32 mMaxSpeed;
    u8 mAlpha;
    TVec3f mBaseUp;
    TPos3f mBaseMtx;
    LiveActor* mRider;
    TPos3f mRiderMtx;
    TVec2f mTexUV0;
    TVec2f mTexUV1;
    TVec2f mTexUV2;
    JUTTexture* mTexture;
    JUTTexture* mIndirect;
    JUTTexture* mEnvMap;
    TBox3f mBoundingBox;
    TVec3f mClippingCenter;
    TVec3f mCamRailNearestPos;
    AudSoundObject* mSoundObj;
    ActorCameraInfo* mCameraInfo;
};

class WaterRoadModelInfo {
public:
    WaterRoadModelInfo(WaterRoad*, bool);

    void initPoints(WaterRoad*);
    void initDisplayList();
    void sendGD() const;
    void drawGD() const;
    void drawDirect(const WaterRoad*) const;
    s32 calcDemoDrawPointNum(const WaterRoad*) const;
    void loadMaterialHigh(const WaterRoad*) const;
    void loadMaterialLow() const;

    s32 calcPointIndex(int i, int j) const {
        return j + i * mNumLoopPoints;
    }

    /* 0x00 */ bool mIsLow;
    /* 0x04 */ s32 mNumPoints;
    /* 0x08 */ s32 mNumLinePoints;
    /* 0x0C */ s32 mNumLoopPoints;
    /* 0x10 */ TVec3f* mPoints;
    /* 0x14 */ TVec3s* mNormals;
    /* 0x18 */ f32* mRailCoords;
    /* 0x1C */ u32 mDispListLength;
    /* 0x20 */ u8* mDispList;
};
