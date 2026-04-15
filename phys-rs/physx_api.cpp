#include "PxPhysicsAPI.h"
#include <cstdint>
#include "physx_generated.hpp"
#include "magicphysx_export.h"

PxDefaultAllocator gAllocator;
PxDefaultErrorCallback gErrorCallback;

struct FilterShaderCallbackInfo
{
    PxFilterObjectAttributes attributes0;
    PxFilterObjectAttributes attributes1;
    PxFilterData filterData0;
    PxFilterData filterData1;
    PxPairFlags *pairFlags;
    const void *constantBlock;
    PxU32 constantBlockSize;
};

typedef void (*CollisionCallback)(void *, PxContactPairHeader const *, PxContactPair const *, PxU32);
extern "C" typedef PxU16 (*SimulationShaderFilter)(FilterShaderCallbackInfo *);

struct FilterCallbackData {
    SimulationShaderFilter filter;
    bool call_default_filter_shader_first;
};

PxFilterFlags FilterShaderTrampoline(PxFilterObjectAttributes attributes0,
                                     PxFilterData filterData0,
                                     PxFilterObjectAttributes attributes1,
                                     PxFilterData filterData1,
                                     PxPairFlags &pairFlags,
                                     const void *constantBlock,
                                     PxU32 constantBlockSize)
{
    const FilterCallbackData *data = static_cast<const FilterCallbackData *>(constantBlock);

    if (data->call_default_filter_shader_first) {
        // Let the default handler set the pair flags, but ignore the collision filtering
        PxDefaultSimulationFilterShader(attributes0, filterData0, attributes1, filterData1, pairFlags, constantBlock,
                                        constantBlockSize);
    }

    // Get the filter shader from the constant block
    SimulationShaderFilter shaderfilter = data->filter;

    // This is a bit expensive since we're putting things on the stack but with LTO this should optimize OK,
    // and I was having issues with corrupted values when passing by value
    FilterShaderCallbackInfo info{attributes0, attributes1, filterData0, filterData1, &pairFlags, nullptr, 0};

    // We return a u16 since PxFilterFlags is a complex type and C++ wants it to be returned on the stack,
    // but Rust thinks it's simple due to the codegen and wants to return it in EAX.
    return PxFilterFlags{shaderfilter(&info)};
}

using CollisionCallback = void (*)(void *, PxContactPairHeader const *, PxContactPair const *, PxU32);
using TriggerCallback = void (*)(void *, PxTriggerPair const *, PxU32);
using ConstraintBreakCallback = void (*)(void *, PxConstraintInfo const *, PxU32);
using WakeSleepCallback = void (*)(void *, PxActor **const, PxU32, bool);
using AdvanceCallback = void (*)(void *, const PxRigidBody *const *, const PxTransform *const, PxU32);

struct SimulationEventCallbackInfo {
    CollisionCallback collisionCallback = nullptr;
    void *collisionUserData = nullptr;
    TriggerCallback triggerCallback = nullptr;
    void *triggerUserData = nullptr;
    ConstraintBreakCallback constraintBreakCallback = nullptr;
    void *constraintBreakUserData = nullptr;
    WakeSleepCallback wakeSleepCallback = nullptr;
    void *wakeSleepUserData = nullptr;
    AdvanceCallback advanceCallback = nullptr;
    void *advanceUserData = nullptr;
};

class SimulationEventTrampoline : public PxSimulationEventCallback
{
  public:
    SimulationEventTrampoline(const SimulationEventCallbackInfo *callbacks) : mCallbacks(*callbacks) {}

    void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs) override {
        if (mCallbacks.collisionCallback) {
            mCallbacks.collisionCallback(mCallbacks.collisionUserData, &pairHeader, pairs, nbPairs);
        }
    }

    void onTrigger(PxTriggerPair *pairs, PxU32 count) override {
        if (mCallbacks.triggerCallback) {
            mCallbacks.triggerCallback(mCallbacks.triggerUserData, pairs, count);
        }
    }

    void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) override {
        if (mCallbacks.constraintBreakCallback) {
            mCallbacks.constraintBreakCallback(mCallbacks.constraintBreakUserData, constraints, count);
        }
    }

    void onWake(PxActor **actors, PxU32 count) override {
        if (mCallbacks.wakeSleepCallback) {
            mCallbacks.wakeSleepCallback(mCallbacks.wakeSleepUserData, actors, count, true);
        }
    }
    void onSleep(PxActor **actors, PxU32 count) override {
        if (mCallbacks.wakeSleepCallback) {
            mCallbacks.wakeSleepCallback(mCallbacks.wakeSleepUserData, actors, count, false);
        }
    }

    void onAdvance(const PxRigidBody *const * bodyBuffer, const PxTransform *poseBuffer, const PxU32 count) override {
        if (mCallbacks.advanceCallback) {
            mCallbacks.advanceCallback(mCallbacks.advanceUserData, bodyBuffer, poseBuffer, count);
        }
    }

    SimulationEventCallbackInfo mCallbacks;
};

class RaycastFilterCallback : public PxQueryFilterCallback
{
  public:
    explicit RaycastFilterCallback(PxRigidActor *actor) : mActor(actor) {}

    PxRigidActor *mActor;

    virtual PxQueryHitType::Enum preFilter(const PxFilterData &, const PxShape *shape, const PxRigidActor *actor, PxHitFlags &)
    {
        if (mActor == actor)
        {
            return PxQueryHitType::eNONE;
        }
        else
        {
            return PxQueryHitType::eBLOCK;
        }
    }

    virtual PxQueryHitType::Enum postFilter(const PxFilterData &, const PxQueryHit &)
    {
        return PxQueryHitType::eNONE;
    }
};

typedef uint32_t (*RaycastHitCallback)(const PxRigidActor *actor, const PxFilterData *filterData, const PxShape *shape, uint32_t hitFlags, const void *userData);
typedef uint32_t (*PostFilterCallback)(const PxFilterData *filterData, const PxQueryHit* hit, const void *userData);

PxQueryHitType::Enum sanitize_hit_type(uint32_t hit_type) {
    switch (hit_type) {
        case PxQueryHitType::eNONE:
        case PxQueryHitType::eTOUCH:
        case PxQueryHitType::eBLOCK: return (PxQueryHitType::Enum)hit_type;
        default: return PxQueryHitType::eNONE;
    }
}

class RaycastFilterTrampoline : public PxQueryFilterCallback
{
  public:
    RaycastFilterTrampoline(RaycastHitCallback callback, const void *userdata)
        : mCallback(callback), mUserData(userdata) {}

    RaycastHitCallback mCallback;
    const void *mUserData;

    virtual PxQueryHitType::Enum preFilter(const PxFilterData &filterData, const PxShape *shape, const PxRigidActor *actor, PxHitFlags &hitFlags)
    {
        return sanitize_hit_type(mCallback(actor, &filterData, shape, (uint32_t)hitFlags, mUserData));
    }

    virtual PxQueryHitType::Enum postFilter(const PxFilterData &, const PxQueryHit &)
    {
        return PxQueryHitType::eNONE;
    }
};


class RaycastFilterPrePostTrampoline : public PxQueryFilterCallback
{
  public:
    RaycastFilterPrePostTrampoline(RaycastHitCallback preFilter, PostFilterCallback postFilter, const void *userdata)
        : mPreFilter(preFilter), mPostFilter(postFilter), mUserData(userdata) {}

    RaycastHitCallback mPreFilter;
    PostFilterCallback mPostFilter;
    
    const void *mUserData;

    virtual PxQueryHitType::Enum preFilter(const PxFilterData &filterData, const PxShape *shape, const PxRigidActor *actor, PxHitFlags &hitFlags)
    {
        return sanitize_hit_type(mPreFilter(actor, &filterData, shape, (uint32_t)hitFlags, mUserData));

    }

    virtual PxQueryHitType::Enum postFilter(const PxFilterData &filterData, const PxQueryHit &hit)
    {
        return sanitize_hit_type(mPostFilter(&filterData, &hit, mUserData));
    }
};

typedef PxAgain (*RaycastHitProcessTouchesCallback)(const PxRaycastHit *buffer, PxU32 nbHits, void *userdata);
typedef PxAgain (*SweepHitProcessTouchesCallback)(const PxSweepHit *buffer, PxU32 nbHits, void *userdata);
typedef PxAgain (*OverlapHitProcessTouchesCallback)(const PxOverlapHit *buffer, PxU32 nbHits, void *userdata);
typedef void (*HitFinalizeQueryCallback)(void *userdata);

class RaycastHitCallbackTrampoline : public PxRaycastCallback
{
  public:
    RaycastHitCallbackTrampoline(
        RaycastHitProcessTouchesCallback processTouchesCallback,
        HitFinalizeQueryCallback finalizeQueryCallback,
        PxRaycastHit *touchesBuffer,
        PxU32 numTouches,
        void *userdata)
        : PxRaycastCallback(touchesBuffer, numTouches),
          mProcessTouchesCallback(processTouchesCallback),
          mFinalizeQueryCallback(finalizeQueryCallback),
          mUserData(userdata) {}

    RaycastHitProcessTouchesCallback mProcessTouchesCallback;
    HitFinalizeQueryCallback mFinalizeQueryCallback;
    void *mUserData;

    PxAgain processTouches(const PxRaycastHit *buffer, PxU32 nbHits) override
    {
        return mProcessTouchesCallback(buffer, nbHits, mUserData);
    }

    void finalizeQuery() override
    {
        mFinalizeQueryCallback(mUserData);
    }
};

class SweepHitCallbackTrampoline : public PxSweepCallback
{
  public:
    SweepHitCallbackTrampoline(
        SweepHitProcessTouchesCallback processTouchesCallback,
        HitFinalizeQueryCallback finalizeQueryCallback,
        PxSweepHit *touchesBuffer,
        PxU32 numTouches,
        void *userdata)
        : PxSweepCallback(touchesBuffer, numTouches),
          mProcessTouchesCallback(processTouchesCallback),
          mFinalizeQueryCallback(finalizeQueryCallback),
          mUserData(userdata) {}

    SweepHitProcessTouchesCallback mProcessTouchesCallback;
    HitFinalizeQueryCallback mFinalizeQueryCallback;
    void *mUserData;

    PxAgain processTouches(const PxSweepHit *buffer, PxU32 nbHits) override
    {
        return mProcessTouchesCallback(buffer, nbHits, mUserData);
    }

    void finalizeQuery() override
    {
        mFinalizeQueryCallback(mUserData);
    }
};

class OverlapHitCallbackTrampoline : public PxOverlapCallback
{
  public:
    OverlapHitCallbackTrampoline(
        OverlapHitProcessTouchesCallback processTouchesCallback,
        HitFinalizeQueryCallback finalizeQueryCallback,
        PxOverlapHit *touchesBuffer,
        PxU32 numTouches,
        void *userdata)
        : PxOverlapCallback(touchesBuffer, numTouches),
          mProcessTouchesCallback(processTouchesCallback),
          mFinalizeQueryCallback(finalizeQueryCallback),
          mUserData(userdata) {}

    OverlapHitProcessTouchesCallback mProcessTouchesCallback;
    HitFinalizeQueryCallback mFinalizeQueryCallback;
    void *mUserData;

    PxAgain processTouches(const PxOverlapHit *buffer, PxU32 nbHits) override
    {
        return mProcessTouchesCallback(buffer, nbHits, mUserData);
    }

    void finalizeQuery() override
    {
        mFinalizeQueryCallback(mUserData);
    }
};

typedef void * (*AllocCallback)(uint64_t size, const char *typeName, const char *filename, int line, void *userdata);
typedef void (*DeallocCallback)(void *ptr, void *userdata);

class CustomAllocatorTrampoline : public PxAllocatorCallback {
public:
    CustomAllocatorTrampoline(AllocCallback allocCb, DeallocCallback deallocCb, void *userdata)
        : mAllocCallback(allocCb), mDeallocCallback(deallocCb), mUserData(userdata) {}

    void *allocate(size_t size, const char *typeName, const char *filename, int line)
    {
        return mAllocCallback((uint64_t)size, typeName, filename, line, mUserData);
    }

    virtual void deallocate(void* ptr)
    {
        mDeallocCallback(ptr, mUserData);
    }

private:
    AllocCallback mAllocCallback;
    DeallocCallback mDeallocCallback;
public:
    void *mUserData;
};

typedef void * (*ZoneStartCallback)(const char *typeName, bool detached, uint64_t context  , void *userdata);
typedef void  (*ZoneEndCallback)(void* profilerData, const char *typeName, bool detached, uint64_t context , void *userdata);

class CustomProfilerTrampoline : public PxProfilerCallback {
public:
    CustomProfilerTrampoline(ZoneStartCallback startCb, ZoneEndCallback endCb, void *userdata)
        : mStartCallback(startCb), mEndCallback(endCb), mUserData(userdata) {
    }

	virtual void* zoneStart(const char* eventName, bool detached, uint64_t contextId) override
	{
		return mStartCallback(eventName, detached, contextId, mUserData);
	}

	virtual void zoneEnd(void* profilerData, const char* eventName, bool detached, uint64_t contextId) override
	{
		return mEndCallback(profilerData, eventName, detached, contextId, mUserData);
	}

private:
    ZoneStartCallback mStartCallback;
    ZoneEndCallback mEndCallback;
public:
    void *mUserData;
};

using ErrorCallback = void (*)(int code, const char* message, const char* file, int line, void* userdata);

class ErrorTrampoline : public PxErrorCallback {
public:
    ErrorTrampoline(ErrorCallback errorCb, void* userdata)
        : mErrorCallback(errorCb), mUserdata(userdata)
	{}

    void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line) override {
        mErrorCallback(code, message, file, line, mUserdata);
    }

private:
    ErrorCallback mErrorCallback = nullptr;
    void* mUserdata = nullptr;
};

using AssertHandler = void (*)(const char* expr, const char* file, int line, bool* should_ignore, void* userdata);

class AssertTrampoline : public PxAssertHandler {
public:
    AssertTrampoline(AssertHandler onAssert, void* userdata)
        : mAssertHandler(onAssert), mUserdata(userdata)
	{}

	virtual void operator()(const char* exp, const char* file, int line, bool& ignore) override final {
        mAssertHandler(exp, file, line, &ignore, mUserdata);
    }

private:
    AssertHandler mAssertHandler = nullptr;
    void* mUserdata = nullptr;
};

extern "C"
{
    MAGICPHYSX_EXPORT PxFoundation *physx_create_foundation()
    {
        return PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
    }

    MAGICPHYSX_EXPORT PxFoundation *physx_create_foundation_with_alloc(PxAllocatorCallback *allocator)
    {
        return PxCreateFoundation(PX_PHYSICS_VERSION, *allocator, gErrorCallback);
    }

    // fixme[tolsson]: this might be iffy on Windows with DLLs if we have multiple packages
    // linking against the raw interface
    MAGICPHYSX_EXPORT PxAllocatorCallback* get_default_allocator()
    {
        return &gAllocator;
    }

    // fixme[tolsson]: this might be iffy on Windows with DLLs if we have multiple packages
    // linking against the raw interface
    MAGICPHYSX_EXPORT PxErrorCallback* get_default_error_callback()
    {
        return &gErrorCallback;
    }

    MAGICPHYSX_EXPORT PxPhysics *physx_create_physics(PxFoundation *foundation)
    {
        return PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale(), true, nullptr, nullptr);
    }

    MAGICPHYSX_EXPORT PxQueryFilterCallback *create_raycast_filter_callback(PxRigidActor *actor_to_ignore)
    {
        return new RaycastFilterCallback(actor_to_ignore);
    }

    MAGICPHYSX_EXPORT PxQueryFilterCallback *create_raycast_filter_callback_func(RaycastHitCallback callback, void *userData)
    {
        return new RaycastFilterTrampoline(callback, userData);
    }

    MAGICPHYSX_EXPORT PxQueryFilterCallback *create_pre_and_post_raycast_filter_callback_func(RaycastHitCallback preFilter, PostFilterCallback postFilter, void *userData)
    {
        return new RaycastFilterPrePostTrampoline(preFilter, postFilter, userData);
    }

    MAGICPHYSX_EXPORT PxRaycastCallback *create_raycast_buffer()
    {
        return new PxRaycastBuffer;
    }

    MAGICPHYSX_EXPORT PxSweepCallback *create_sweep_buffer()
    {
        return new PxSweepBuffer;
    }

    MAGICPHYSX_EXPORT PxOverlapCallback *create_overlap_buffer()
    {
        return new PxOverlapBuffer;
    }

    MAGICPHYSX_EXPORT PxRaycastCallback *create_raycast_callback(
        RaycastHitProcessTouchesCallback process_touches_callback,
        HitFinalizeQueryCallback finalize_query_callback,
        PxRaycastHit *touchesBuffer,
        PxU32 numTouches,
        void *userdata
    ) {
        return new RaycastHitCallbackTrampoline(
            process_touches_callback, finalize_query_callback, touchesBuffer, numTouches, userdata);
    }

    MAGICPHYSX_EXPORT void delete_raycast_callback(PxRaycastCallback *callback)
    {
        delete callback;
    }

    MAGICPHYSX_EXPORT void delete_sweep_callback(PxSweepCallback *callback)
    {
        delete callback;
    }

    MAGICPHYSX_EXPORT void delete_overlap_callback(PxOverlapCallback *callback)
    {
        delete callback;
    }

    MAGICPHYSX_EXPORT PxSweepCallback *create_sweep_callback(
        SweepHitProcessTouchesCallback process_touches_callback,
        HitFinalizeQueryCallback finalize_query_callback,
        PxSweepHit *touchesBuffer,
        PxU32 numTouches,
        void *userdata
    ) {
        return new SweepHitCallbackTrampoline(
            process_touches_callback, finalize_query_callback, touchesBuffer, numTouches, userdata
        );
    }

    MAGICPHYSX_EXPORT PxOverlapCallback *create_overlap_callback(
        OverlapHitProcessTouchesCallback process_touches_callback,
        HitFinalizeQueryCallback finalize_query_callback,
        PxOverlapHit *touchesBuffer,
        PxU32 numTouches,
        void *userdata
    ) {
        return new OverlapHitCallbackTrampoline(
            process_touches_callback, finalize_query_callback, touchesBuffer, numTouches, userdata
        );
    }

    MAGICPHYSX_EXPORT PxAllocatorCallback *create_alloc_callback(
        AllocCallback alloc_callback,
        DeallocCallback dealloc_callback,
        void *userdata
    ) {
        return new CustomAllocatorTrampoline(alloc_callback, dealloc_callback, userdata);
    }

    MAGICPHYSX_EXPORT void *get_alloc_callback_user_data(PxAllocatorCallback *allocator) {
        CustomAllocatorTrampoline *trampoline = static_cast<CustomAllocatorTrampoline *>(allocator);
        return trampoline->mUserData;
    }

	MAGICPHYSX_EXPORT PxProfilerCallback *create_profiler_callback(
        ZoneStartCallback zone_start_callback,
        ZoneEndCallback zone_end_callback,
        void *userdata
    ) {
        return new CustomProfilerTrampoline(zone_start_callback, zone_end_callback, userdata);
    }

    MAGICPHYSX_EXPORT PxErrorCallback *create_error_callback(
        ErrorCallback error_callback,
        void* userdata
    ) {
        return new ErrorTrampoline(error_callback, userdata);
    }


    MAGICPHYSX_EXPORT PxAssertHandler *create_assert_handler(
        AssertHandler on_assert,
        void* userdata
    ) {
        return new AssertTrampoline(on_assert, userdata);
    }

    MAGICPHYSX_EXPORT void *get_default_simulation_filter_shader()
    {
        return (void *)PxDefaultSimulationFilterShader;
    }

    MAGICPHYSX_EXPORT PxSimulationEventCallback *create_simulation_event_callbacks(const SimulationEventCallbackInfo *callbacks)
    {
        SimulationEventTrampoline *trampoline = new SimulationEventTrampoline(callbacks);
        return static_cast<PxSimulationEventCallback *>(trampoline);
    }

    MAGICPHYSX_EXPORT SimulationEventCallbackInfo *get_simulation_event_info(PxSimulationEventCallback *callback)
    {
        SimulationEventTrampoline *trampoline = static_cast<SimulationEventTrampoline *>(callback);
        return &trampoline->mCallbacks;
    }

    MAGICPHYSX_EXPORT void destroy_simulation_event_callbacks(PxSimulationEventCallback *callback)
    {
        SimulationEventTrampoline *trampoline = static_cast<SimulationEventTrampoline *>(callback);
        delete trampoline;
    }

    MAGICPHYSX_EXPORT void enable_custom_filter_shader(PxSceneDesc *desc, SimulationShaderFilter filter, uint32_t call_default_filter_shader_first)
    {
        /* Allocate memory dynamically instead of using static storage */
        FilterCallbackData* filterShaderData = (FilterCallbackData *)malloc(sizeof(FilterCallbackData));
        if (!filterShaderData)
        {
            return;
        }

        filterShaderData->filter = filter;
        filterShaderData->call_default_filter_shader_first = (call_default_filter_shader_first != 0);

        desc->filterShader = FilterShaderTrampoline;
        desc->filterShaderData = (void *)filterShaderData;
        desc->filterShaderDataSize = sizeof(FilterCallbackData);
    }

    MAGICPHYSX_EXPORT void free_custom_filter_shader(PxSceneDesc *desc)
    {
        if (desc->filterShaderData)
        {
            free((FilterCallbackData*)desc->filterShaderData);
            desc->filterShaderData = 0;
        }
    }

	// Not generated, used only for testing and examples!
    MAGICPHYSX_EXPORT void PxAssertHandler_opCall_mut(physx_PxErrorCallback_Pod* self__pod, char const* expr, char const* file, int32_t line, bool* ignore ) {
		physx::PxAssertHandler* self_ = reinterpret_cast<physx::PxAssertHandler*>(self__pod);
		(*self_)(expr, file, line, *ignore);
	};

    typedef void (*ControllerShapeHitCallback)(PxControllerShapeHit const *);
    typedef void (*ControllersHitCallback)(PxControllersHit const *);
    typedef void (*ControllerObstacleHitCallback)(PxControllerObstacleHit const *);

    struct ControllerCallbackInfo {
        ControllerShapeHitCallback controllerShapeHitCallback = nullptr;
        ControllersHitCallback controllersHitCallback = nullptr;
        ControllerObstacleHitCallback controllerObstacleHitCallback = nullptr;
    };

    class UserControllerHitReportTrampoline : public PxUserControllerHitReport
    {
      public:
        UserControllerHitReportTrampoline(const ControllerCallbackInfo *callbacks) : mCallbacks(*callbacks) {}

        void onShapeHit(const PxControllerShapeHit& hit) override {
            if (mCallbacks.controllerShapeHitCallback) {
                mCallbacks.controllerShapeHitCallback(&hit);
            }
        }

        void onControllerHit(const PxControllersHit& hit) override {
            if (mCallbacks.controllersHitCallback) {
                mCallbacks.controllersHitCallback(&hit);
            }
        }

        void onObstacleHit(const PxControllerObstacleHit& hit) override {
            if (mCallbacks.controllerObstacleHitCallback) {
                mCallbacks.controllerObstacleHitCallback(&hit);
            }
        }

        ControllerCallbackInfo mCallbacks;
    };

    MAGICPHYSX_EXPORT PxUserControllerHitReport *create_user_controller_hit_report(const ControllerCallbackInfo *callbacks)
    {
        UserControllerHitReportTrampoline *trampoline = new UserControllerHitReportTrampoline(callbacks);
        return static_cast<PxUserControllerHitReport *>(trampoline);
    }

    MAGICPHYSX_EXPORT void destroy_user_controller_hit_report(PxUserControllerHitReport *callback)
    {
        UserControllerHitReportTrampoline *trampoline = static_cast<UserControllerHitReportTrampoline *>(callback);
        delete trampoline;
    }
}

extern "C"
{
    typedef PxControllerBehaviorFlags(*ControllerBehaviorGetFlagsShapeCallback)(
        const PxController* sourceController, const PxShape* shape, const PxActor* actor);

    typedef PxControllerBehaviorFlags(*ControllerBehaviorGetFlagsControllerCallback)(
        const PxController* sourceController, const PxController* otherController);

    typedef PxControllerBehaviorFlags(*ControllerBehaviorGetFlagsObstacleCallback)(
        const PxController* sourceController, const PxObstacle* obstacle);

    struct ControllerBehaviorCallbackInfo
    {
        ControllerBehaviorGetFlagsShapeCallback      getBehaviorFlagsShapeCallback = nullptr;
        ControllerBehaviorGetFlagsControllerCallback  getBehaviorFlagsControllerCallback = nullptr;
        ControllerBehaviorGetFlagsObstacleCallback    getBehaviorFlagsObstacleCallback = nullptr;
        const PxController* sourceController = nullptr;
    };
}

class PxControllerBehaviorCallbackTrampoline final : public PxControllerBehaviorCallback
{
public:
    explicit PxControllerBehaviorCallbackTrampoline(const ControllerBehaviorCallbackInfo* info)
        : mInfo(*info) {}

    PxControllerBehaviorFlags getBehaviorFlags(const PxShape& shape, const PxActor& actor) override
    {
        if (!mInfo.getBehaviorFlagsShapeCallback) return PxControllerBehaviorFlags(0);
        return mInfo.getBehaviorFlagsShapeCallback(mInfo.sourceController, &shape, &actor);
    }

    PxControllerBehaviorFlags getBehaviorFlags(const PxController& controller) override
    {
        if (!mInfo.getBehaviorFlagsControllerCallback) return PxControllerBehaviorFlags(0);
        return mInfo.getBehaviorFlagsControllerCallback(mInfo.sourceController, &controller);
    }

    PxControllerBehaviorFlags getBehaviorFlags(const PxObstacle& obstacle) override
    {
        if (!mInfo.getBehaviorFlagsObstacleCallback) return PxControllerBehaviorFlags(0);
        return mInfo.getBehaviorFlagsObstacleCallback(mInfo.sourceController, &obstacle);
    }

    ControllerBehaviorCallbackInfo mInfo;
};

extern "C"
{
    MAGICPHYSX_EXPORT PxControllerBehaviorCallback* create_controller_behavior_callback(
        const ControllerBehaviorCallbackInfo* info)
    {
        return new PxControllerBehaviorCallbackTrampoline(info);
    }

    MAGICPHYSX_EXPORT void destroy_controller_behavior_callback(PxControllerBehaviorCallback* cb)
    {
        delete static_cast<PxControllerBehaviorCallbackTrampoline*>(cb);
    }
}

extern "C"
{
    typedef bool (*ControllerFilterFunction)(
        const PxController* controllerA, const PxController* controllerB, void* userData);

    struct ControllerFilterCallbackInfo
    {
        ControllerFilterFunction filterCallback = nullptr;
        void* userData = nullptr;
    };
}

class PxControllerFilterCallbackTrampoline final : public PxControllerFilterCallback
{
public:
    explicit PxControllerFilterCallbackTrampoline(const ControllerFilterCallbackInfo* info)
        : mInfo(*info) {}

    bool filter(const PxController& controllerA, const PxController& controllerB) override
    {
        if (!mInfo.filterCallback) return true;
        return mInfo.filterCallback(&controllerA, &controllerB, mInfo.userData);
    }

    ControllerFilterCallbackInfo mInfo;
};

extern "C"
{
    MAGICPHYSX_EXPORT PxControllerFilterCallback* create_controller_filter_callback(
        ControllerFilterFunction callback, void* userData)
    {
        ControllerFilterCallbackInfo info;
        info.filterCallback = callback;
        info.userData = userData;
        return new PxControllerFilterCallbackTrampoline(&info);
    }

    MAGICPHYSX_EXPORT void destroy_controller_filter_callback(PxControllerFilterCallback* cb)
    {
        delete static_cast<PxControllerFilterCallbackTrampoline*>(cb);
    }
}