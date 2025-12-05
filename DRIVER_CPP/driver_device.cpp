// driver_device.cpp - Implementação CORRIGIDA com interface OpenVR
#include "driver_device.h"
#include "protocol.h"
#include "udp_listener.h"
#include <openvr_driver.h>
#include <iostream>
#include <cmath>
#include <cstring>

// Conversão de VRTrackerPacket_V6 para vr::DriverPose_t
static vr::DriverPose_t convert_packet_to_pose(const VRTrackerPacket_V6* pkt) {
    vr::DriverPose_t pose = {0};
    
    // Timestamp
    pose.poseTimeOffset = 0.0; // Atualização em tempo real
    pose.poseIsValid = true;
    pose.deviceIsConnected = true;
    pose.result = vr::TrackingResult_Running_OK;
    
    // Orientação (quaternion)
    pose.qRotation.w = pkt->quat[0];
    pose.qRotation.x = pkt->quat[1];
    pose.qRotation.y = pkt->quat[2];
    pose.qRotation.z = pkt->quat[3];
    
    // Posição (metros)
    pose.vecPosition[0] = pkt->position[0];
    pose.vecPosition[1] = pkt->position[1];
    pose.vecPosition[2] = pkt->position[2];
    
    // Velocidade (m/s)
    pose.vecVelocity[0] = pkt->velocity[0];
    pose.vecVelocity[1] = pkt->velocity[1];
    pose.vecVelocity[2] = pkt->velocity[2];
    
    // Velocidade angular (rad/s)
    pose.vecAngularVelocity[0] = pkt->gyro[0];
    pose.vecAngularVelocity[1] = pkt->gyro[1];
    pose.vecAngularVelocity[2] = pkt->gyro[2];
    
    // Aceleração (m/s²)
    pose.vecAcceleration[0] = pkt->accel[0];
    pose.vecAcceleration[1] = pkt->accel[1];
    pose.vecAcceleration[2] = pkt->accel[2];
    
    // Aceleração angular (rad/s²) - calcular se necessário
    pose.vecAngularAcceleration[0] = 0.0;
    pose.vecAngularAcceleration[1] = 0.0;
    pose.vecAngularAcceleration[2] = 0.0;
    
    // Flags de atividade
    pose.willDriftInYaw = false; // Temos magnetômetro
    pose.shouldApplyHeadModel = false; // É tracker, não HMD
    
    return pose;
}

// ==================== IMPLEMENTAÇÃO CDriverDevice ====================

CDriverDevice::CDriverDevice(const std::string& serialNumber)
    : m_serialNumber(serialNumber)
    , m_trackedDeviceIndex(vr::k_unTrackedDeviceIndexInvalid)
{
    std::cout << "[CDriverDevice] Created device: " << m_serialNumber << std::endl;
}

CDriverDevice::~CDriverDevice() {
    std::cout << "[CDriverDevice] Destroyed device: " << m_serialNumber << std::endl;
}

// ==================== Interface vr::ITrackedDeviceServerDriver ====================

vr::EVRInitError CDriverDevice::Activate(uint32_t unObjectId) {
    m_trackedDeviceIndex = unObjectId;
    
    std::cout << "[CDriverDevice] Activate() called, index=" << unObjectId << std::endl;
    
    // Configurar propriedades do dispositivo
    vr::VRProperties()->SetStringProperty(m_propertyContainer, 
        vr::Prop_ModelNumber_String, "LibaTracker_V6");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, 
        vr::Prop_SerialNumber_String, m_serialNumber.c_str());
    vr::VRProperties()->SetStringProperty(m_propertyContainer, 
        vr::Prop_RenderModelName_String, "vr_tracker_waist");
    
    // Propriedades de rastreamento
    vr::VRProperties()->SetInt32Property(m_propertyContainer, 
        vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_GenericTracker);
    vr::VRProperties()->SetInt32Property(m_propertyContainer, 
        vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_Invalid);
    
    // Ícone no SteamVR
    vr::VRProperties()->SetStringProperty(m_propertyContainer, 
        vr::Prop_NamedIconPathDeviceOff_String, 
        "{libatracker}/icons/tracker_off.png");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, 
        vr::Prop_NamedIconPathDeviceSearching_String, 
        "{libatracker}/icons/tracker_searching.png");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, 
        vr::Prop_NamedIconPathDeviceReady_String, 
        "{libatracker}/icons/tracker_ready.png");
    
    return vr::VRInitError_None;
}

void CDriverDevice::Deactivate() {
    m_trackedDeviceIndex = vr::k_unTrackedDeviceIndexInvalid;
    std::cout << "[CDriverDevice] Deactivated" << std::endl;
}

void CDriverDevice::EnterStandby() {
    std::cout << "[CDriverDevice] Entering standby" << std::endl;
}

void* CDriverDevice::GetComponent(const char* pchComponentNameAndVersion) {
    // Não implementamos componentes adicionais
    return nullptr;
}

void CDriverDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, 
                                 uint32_t unResponseBufferSize) {
    if (unResponseBufferSize >= 1) {
        pchResponseBuffer[0] = 0;
    }
}

vr::DriverPose_t CDriverDevice::GetPose() {
    std::lock_guard<std::mutex> lock(m_poseMutex);
    return m_currentPose;
}

// ==================== MÉTODO INTERNO PARA ATUALIZAR POSE ====================

void CDriverDevice::UpdatePoseFromPacket(const VRTrackerPacket_V6* pkt) {
    if (!pkt) return;
    
    // Converter para formato OpenVR
    vr::DriverPose_t newPose = convert_packet_to_pose(pkt);
    
    // Atualizar pose interna (thread-safe)
    {
        std::lock_guard<std::mutex> lock(m_poseMutex);
        m_currentPose = newPose;
    }
    
    // Notificar SteamVR sobre a nova pose
    if (m_trackedDeviceIndex != vr::k_unTrackedDeviceIndexInvalid) {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(
            m_trackedDeviceIndex, 
            newPose, 
            sizeof(vr::DriverPose_t)
        );
    }
}

// ==================== CALLBACK UDP ====================

// Instância estática para receber callbacks
static CDriverDevice* g_deviceInstance = nullptr;

static void on_udp_packet_received(const VRTrackerPacket_V6* pkt, 
                                    const char* src_ip, 
                                    uint16_t src_port) {
    if (!pkt || !g_deviceInstance) return;
    
    // Debug opcional (comentar em produção)
    /*
    std::cout << "[UDP->Driver] seq=" << pkt->sequence 
              << " pos=(" << pkt->position[0] << "," 
              << pkt->position[1] << "," << pkt->position[2] << ")"
              << " from " << src_ip << ":" << src_port << std::endl;
    */
    
    // Atualizar pose do dispositivo
    g_deviceInstance->UpdatePoseFromPacket(pkt);
}

// ==================== REGISTRO DO CALLBACK ====================

void CDriverDevice::RegisterAsUDPReceiver() {
    g_deviceInstance = this;
    set_udp_payload_callback(on_udp_packet_received);
    std::cout << "[CDriverDevice] Registered as UDP receiver" << std::endl;
}

void CDriverDevice::UnregisterAsUDPReceiver() {
    set_udp_payload_callback(nullptr);
    g_deviceInstance = nullptr;
    std::cout << "[CDriverDevice] Unregistered from UDP" << std::endl;
}