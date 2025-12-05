#include <openvr_driver.h>
#include "driver_device.h"
#include <vector>
#include <memory>

// Definição da classe principal do driver (Server Provider)
class CServerDriver_LibaTracker : public vr::IServerTrackedDeviceProvider
{
public:
    virtual vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext);
    virtual void Cleanup();
    virtual const char* const* GetInterfaceVersions() { return vr::k_InterfaceVersions; }
    virtual void RunFrame();
    virtual bool ShouldBlockStandbyMode() { return false; }
    virtual void EnterStandby() {}
    virtual void LeaveStandby() {}

private:
    CDriverDevice* m_pTracker = nullptr;
};

// Variável global do driver
CServerDriver_LibaTracker g_serverDriverLiba;

// Implementação do Init
vr::EVRInitError CServerDriver_LibaTracker::Init(vr::IVRDriverContext* pDriverContext)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

    // 1. Cria o dispositivo (Tracker)
    // Passamos apenas o Serial Number, conforme definimos no novo driver_device.h
    m_pTracker = new CDriverDevice("LibaTracker_V6_001");

    // 2. Avisa o SteamVR que o dispositivo existe
    // Argumentos: Serial Number, Tipo de Device (Tracker), Ponteiro do Objeto
    vr::VRServerDriverHost()->TrackedDeviceAdded(
        "LibaTracker_V6_001", 
        vr::TrackedDeviceClass_GenericTracker, 
        m_pTracker
    );

    return vr::VRInitError_None;
}

// Implementação do Cleanup
void CServerDriver_LibaTracker::Cleanup()
{
    if (m_pTracker) {
        delete m_pTracker;
        m_pTracker = nullptr;
    }
}

// Implementação do RunFrame
void CServerDriver_LibaTracker::RunFrame()
{
    // Nosso tracker roda em uma thread separada (UDPListener),
    // então não precisamos fazer nada aqui a cada frame.
    // O SteamVR chama isso constantemente, deixe vazio ou adicione logs se precisar.
}

// --- Fábrica do Driver (Ponto de Entrada da DLL) ---
extern "C" __declspec(dllexport) void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode)
{
    if (0 == strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName))
    {
        return &g_serverDriverLiba;
    }

    if (pReturnCode)
        *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;

    return NULL;
}
