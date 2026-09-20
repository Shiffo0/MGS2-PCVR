#include "dg_model_arm.h"
static SRWLOCK g_model_arm_lock=SRWLOCK_INIT;
static DG_MODEL_ARM g_model_arm;
void dg_bridge_model_arm_clear(void)
{
    AcquireSRWLockExclusive(&g_model_arm_lock);
    memset(&g_model_arm,0,sizeof g_model_arm);
    ReleaseSRWLockExclusive(&g_model_arm_lock);
}
void dg_bridge_model_arm_publish(const DG_MODEL_ARM *pose)
{
    AcquireSRWLockExclusive(&g_model_arm_lock);
    g_model_arm=*pose;
    ReleaseSRWLockExclusive(&g_model_arm_lock);
}
int dg_bridge_model_arm_snapshot(DG_MODEL_ARM *pose)
{
    ULONGLONG now=GetTickCount64();
    memset(pose,0,sizeof *pose);
    if(!TryAcquireSRWLockShared(&g_model_arm_lock))return 0;
    *pose=g_model_arm;
    ReleaseSRWLockShared(&g_model_arm_lock);
    if(!pose->valid || now<pose->ms || now-pose->ms>100) {
        memset(pose,0,sizeof *pose);return 0;
    }
    return 1;
}
