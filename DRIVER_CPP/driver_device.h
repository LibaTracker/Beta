// driver_device.h - Interface CORRIGIDA com OpenVR
#ifndef DRIVER_DEVICE_H
#define DRIVER_DEVICE_H

#include <string>
#include <mutex>
#include <openvr_driver.h>
#include "protocol.h"

/**
 * CDriverDevice - Implementa vr::ITrackedDeviceServerDriver
 * Representa um tracker LibaTracker V6 no SteamVR
 */
class CDriverDevice : public vr::ITrackedDeviceServerDriver {
public:
    // Construtor
    explicit CDriverDevice(const std::string& serialNumber);
    
    // Destrutor
    virtual ~CDriverDevice();

    // ==================== Interface vr::ITrackedDeviceServerDriver ====================
    
    /**
     * Ativa o dispositivo no SteamVR
     * @param unObjectId ID atribuído pelo SteamVR
     * @return Código de erro (VRInitError_None em sucesso)
     */
    virtual vr::EVRInitError Activate(uint32_t unObjectId) override;
    
    /**
     * Desativa o dispositivo
     */
    virtual void Deactivate() override;
    
    /**
     * Coloca dispositivo em standby
     */
    virtual void EnterStandby() override;
    
    /**
     * Retorna componente específico (opcional)
     */
    virtual void* GetComponent(const char* pchComponentNameAndVersion) override;
    
    /**
     * Debug request handler
     */
    virtual void DebugRequest(const char* pchRequest, 
                             char* pchResponseBuffer, 
                             uint32_t unResponseBufferSize) override;
    
    /**
     * Retorna pose atual do dispositivo
     */
    virtual vr::DriverPose_t GetPose() override;

    // ==================== Métodos específicos do LibaTracker ====================
    
    /**
     * Atualiza pose a partir de pacote UDP recebido
     * @param pkt Ponteiro para pacote validado
     */
    void UpdatePoseFromPacket(const VRTrackerPacket_V6* pkt);
    
    /**
     * Registra este device como receptor de callbacks UDP
     */
    void RegisterAsUDPReceiver();
    
    /**
     * Remove registro de callbacks UDP
     */
    void UnregisterAsUDPReceiver();
    
    /**
     * Retorna serial number
     */
    std::string GetSerialNumber() const { return m_serialNumber; }
    
    /**
     * Retorna índice no SteamVR
     */
    uint32_t GetTrackedDeviceIndex() const { return m_trackedDeviceIndex; }

private:
    // Dados do dispositivo
    std::string m_serialNumber;
    uint32_t m_trackedDeviceIndex;
    vr::PropertyContainerHandle_t m_propertyContainer;
    
    // Pose atual (thread-safe)
    vr::DriverPose_t m_currentPose;
    std::mutex m_poseMutex;
    
    // Não permitir cópia
    CDriverDevice(const CDriverDevice&) = delete;
    CDriverDevice& operator=(const CDriverDevice&) = delete;
};

#endif // DRIVER_DEVICE_H