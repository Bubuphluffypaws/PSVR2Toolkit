#ifdef OPENVR_EXTENSIONS_AVAILABLE
#include "psvr2_openvr_driver/openvr_ex/openvr_ex.h"
#endif

#include "hmd_device_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "vr_settings.h"
#include "util.h"

#include <cstdint>

namespace psvr2_toolkit {

#ifdef OPENVR_EXTENSIONS_AVAILABLE
  void* g_pOpenVRExHandle = nullptr;
#endif

  vr::EVRInitError (*sie__psvr2__HmdDevice__Activate)(void *, uint32_t) = nullptr;
  vr::EVRInitError sie__psvr2__HmdDevice__ActivateHook(void *thisptr, uint32_t unObjectId) {
    vr::EVRInitError result = sie__psvr2__HmdDevice__Activate(thisptr, unObjectId);
    vr::PropertyContainerHandle_t ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);

    vr::VRProperties()->SetBoolProperty(ulPropertyContainer,
                                        vr::Prop_DriverProvidedChaperoneVisibility_Bool,
                                        !VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_CHAPERONE, SETTING_DISABLE_CHAPERONE_DEFAULT_VALUE));

    // This works perfectly fine on NVIDIA GPUs, but is broken on AMD GPUs newer than RDNA2/RX6000-series.
    vr::VRProperties()->SetBoolProperty(ulPropertyContainer,
                                        vr::Prop_Hmd_SupportsHDR10_Bool,
                                        VRSettings::GetBool(STEAMVR_SETTINGS_ENABLE_HDR10, SETTING_ENABLE_HDR10_DEFAULT_VALUE));
    if (VRSettings::GetBool(STEAMVR_SETTINGS_ENABLE_HDR10, SETTING_ENABLE_HDR10_DEFAULT_VALUE)) {
        Util::DriverLog("Enabling HDR10 on PSVR2...");
    }

    // Tell SteamVR to allow runtime framerate changes.
    // SteamVR does not allow this feature on AMD GPUs, so this is NVIDIA-only currently.
    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_DisplaySupportsRuntimeFramerateChange_Bool, true);

    // Tell SteamVR to allow night mode setting.
    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_DisplayAllowNightMode_Bool, true);

    // Tell SteamVR our dashboard scale, which matches Valve Index.
    vr::VRProperties()->SetFloatProperty(ulPropertyContainer, vr::Prop_DashboardScale_Float, .75f);

#ifdef OPENVR_EXTENSIONS_AVAILABLE
    psvr2_toolkit::openvr_ex::OnHmdActivate(ulPropertyContainer, &g_pOpenVRExHandle);
#endif

    return result;
  }

  void (*sie__psvr2__HmdDevice__Deactivate)(void *) = nullptr;
  void sie__psvr2__HmdDevice__DeactivateHook(void *thisptr) {
    sie__psvr2__HmdDevice__Deactivate(thisptr);

#ifdef OPENVR_EXTENSIONS_AVAILABLE
    if (g_pOpenVRExHandle) {
      psvr2_toolkit::openvr_ex::OnHmdDeactivate(&g_pOpenVRExHandle);
    }
#endif
  }

  void HmdDeviceHooks::UpdateGaze(void* pData, size_t dwSize)
  {
#ifdef OPENVR_EXTENSIONS_AVAILABLE
    if (g_pOpenVRExHandle) {
      psvr2_toolkit::openvr_ex::OnHmdUpdate(&g_pOpenVRExHandle, pData, dwSize);
    }
#endif
  }

  void HmdDeviceHooks::InstallHooks() {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

    // sie::psvr2::HmdDevice::Activate
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x19D1B0),
                         reinterpret_cast<void *>(sie__psvr2__HmdDevice__ActivateHook),
                         reinterpret_cast<void **>(&sie__psvr2__HmdDevice__Activate));

    // sie::psvr2::HmdDevice::Deactivate
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x19EBF0),
                         reinterpret_cast<void *>(sie__psvr2__HmdDevice__DeactivateHook),
                         reinterpret_cast<void **>(&sie__psvr2__HmdDevice__Deactivate));
  }

} // psvr2_toolkit
