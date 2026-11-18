// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024 ASFireWire Project
//
// SPro24DspProtocol.hpp - Focusrite Saffire Pro 24 DSP protocol implementation
// Reference: snd-firewire-ctl-services/protocols/dice/src/focusrite/spro24dsp.rs

#pragma once

#include "SaffireproCommon.hpp"
#include "SPro24DspControlModel.hpp"
#include "SPro24DspTypes.hpp"
#include "../Core/DICETypes.hpp"
#include "../Core/DiceClockChangeObserver.hpp"
#include "../TCAT/DICETcatProtocol.hpp"
#include "../../IDeviceProtocol.hpp"
#include "../../../Controls/IControlProvider.hpp"
#include "../../../../Scheduling/ITimerScheduler.hpp"
#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <vector>

namespace ASFW::IRM {
class IRMClient;
}

namespace ASFW::Audio::DICE::Focusrite {

// ============================================================================
// Device Identification
// ============================================================================

/// Focusrite vendor ID (OUI)
constexpr uint32_t kFocusriteVendorId = 0x00130e;

/// Saffire Pro 24 DSP model ID
constexpr uint32_t kSPro24DspModelId = 0x000008;

// ============================================================================
// SPro24DspProtocol
// ============================================================================

/// Protocol handler for Focusrite Saffire Pro 24 DSP
/// 
/// This class provides async-callback-based access to device parameters.
/// All operations are asynchronous since they involve FireWire transactions.
class SPro24DspProtocol : public Audio::IDeviceProtocol,
                          public ::ASFW::Audio::Controls::IControlProvider,
                          private DiceClockChangeObserver {
public:
    /// Callback types for async operations
    using InitCallback = std::function<void(IOReturn)>;
    using VoidCallback = std::function<void(IOReturn)>;
    template<typename T> using ResultCallback = std::function<void(IOReturn, T)>;
    
    /// Construct protocol handler
    /// @param busOps     FireWire bus operations port
    /// @param busInfo    FireWire bus info port
    /// @param route      Current registry-issued device route
    /// @param waitClock  Paces the bring-up's waits (CLOCK_ACCEPTED, clock
    ///                   lock, source-lock confirm). Deliberately required: a
    ///                   missing time source made every wait that needed a
    ///                   second poll fail at once, which is how this protocol
    ///                   ran until the S0 golden traces exposed it.
    SPro24DspProtocol(Protocols::Ports::FireWireBusOps& busOps,
                      Protocols::Ports::FireWireBusInfo& busInfo,
                      Discovery::DeviceRegistry& routeRegistry,
                      const Discovery::DeviceRouteToken& route,
                      ::ASFW::IRM::IRMClient* irmClient,
                      DiceWaitClock& waitClock,
                      DiceNotificationRouter* notifications,
                      Scheduling::ITimerScheduler* preparationScheduler = nullptr);
    
    /// Initialize protocol (generic DICE init is delegated to the TCAT core)
    IOReturn Initialize() override;
    
    /// Shutdown protocol
    IOReturn Shutdown() override;
    
    /// Get device name
    const char* GetName() const override { return "Focusrite Saffire Pro 24 DSP"; }
    Audio::FamilyDriver* AsFamilyDriver() noexcept override { return tcat_.AsFamilyDriver(); }
    
    /// Device has DSP effects
    bool HasDsp() const override { return true; }

    // ========================================================================
    // Device Controls (IControlProvider). Controller (Default) queue only:
    // every step is an asynchronous transaction, never a blocking DiceDeviceIo.
    // ========================================================================
    ::ASFW::Audio::Controls::IControlProvider* Controls() noexcept override { return this; }
    void Prepare(std::function<void(IOReturn)> done) override;
    std::optional<::ASFW::Audio::Controls::ControlSnapshot> CopySnapshot() const override;
    void SetParameter(AudioModel::ParameterId id, const AudioModel::ParameterValue& value,
                      Completion done) override;
    void SetRoute(AudioModel::NodeId router, AudioModel::PortId output,
                  std::optional<AudioModel::PortId> input, Completion done) override;

    bool GetRuntimeAudioStreamCaps(AudioStreamRuntimeCaps& outCaps) const override;
    std::shared_ptr<const std::vector<Runtime::RateFormation>> RateFormations() const override {
        return tcat_.RateFormations();
    }
    void ReadRateObservation(std::function<void(IOReturn, RateHardwareObservation)> callback) override {
        tcat_.ReadRateObservation(std::move(callback));
    }
    bool CopyDiceReceiverLock(Diagnostics::DiceLockSnapshot& out) const override {
        return tcat_.CopyDiceReceiverLock(out);
    }
    IOReturn RequestDiceReceiverDump(bool fifoEvents) override {
        return tcat_.RequestDiceReceiverDump(fifoEvents);
    }
    bool CopyDiceReceiverDump(Diagnostics::DiceReceiverDumpSnapshot& out) const override {
        return tcat_.CopyDiceReceiverDump(out);
    }
    void CancelPendingPreparation() override { tcat_.CancelPendingPreparation(); }
    void EnsureRuntimeStreamGeometry(std::function<void(IOReturn)> callback) override {
        tcat_.EnsureRuntimeStreamGeometry(std::move(callback));
    }
    
    IOReturn StopDuplex() override;
    void UpdateRuntimeContext(const Discovery::DeviceRouteToken& route,
                              std::shared_ptr<ASFW::AVC::IAvcUnit> avcUnit) override;
    
    // ========================================================================
    // Async Initialization
    // ========================================================================
    
    /// Initialize protocol asynchronously
    void InitializeAsync(InitCallback callback);
    
    // ========================================================================
    // DSP Control (Async)
    // ========================================================================
    
    /// Enable/disable DSP
    void EnableDsp(bool enable, VoidCallback callback);
    
    /// Get effect general parameters
    void GetEffectParams(ResultCallback<EffectGeneralParams> callback);
    
    /// Set effect general parameters
    void SetEffectParams(const EffectGeneralParams& params, VoidCallback callback);
    
    /// Get compressor state
    void GetCompressorState(ResultCallback<CompressorState> callback);
    
    /// Set compressor state
    void SetCompressorState(const CompressorState& state, VoidCallback callback);
    
    /// Get reverb state
    void GetReverbState(ResultCallback<ReverbState> callback);
    
    /// Set reverb state
    void SetReverbState(const ReverbState& state, VoidCallback callback);
    
    // ========================================================================
    // Input/Output Control (Async)
    // ========================================================================
    
    /// Get input parameters
    void GetInputParams(ResultCallback<InputParams> callback);
    
    /// Set input parameters
    void SetInputParams(const InputParams& params, VoidCallback callback);
    
    /// Get output group state
    void GetOutputGroupState(ResultCallback<OutputGroupState> callback);
    
    /// Set output group state
    void SetOutputGroupState(const OutputGroupState& state, VoidCallback callback);

private:
    // DiceClockChangeObserver: keep clock changes away from a DSP that is not up.
    void BeforeClockSelectWrite(DiceDeviceIo& io) noexcept override;

    TCAT::DICETcatProtocol tcat_;
    ExtensionSections extensionSections_{};
    uint32_t appSectionBase_{0};
    uint32_t commandSectionBase_{0};
    uint32_t routerSectionBase_{0};
    uint32_t currentConfigBase_{0};
    bool extensionsLoaded_{false};
    
    /// Send software notice to commit changes
    void SendSwNotice(SwNotice notice, VoidCallback callback);
    void EnsureExtensionsLoaded(VoidCallback callback);
    void ReadAppQuad(uint32_t offset, std::function<void(IOReturn, uint32_t)> callback);
    void WriteAppQuad(uint32_t offset, uint32_t value, VoidCallback callback);

    void HandleExtensionSectionsRead(IOReturn status,
                                     ExtensionSections sections,
                                     InitCallback callback);
    
    /// Read from application section
    void ReadAppSection(uint32_t offset, size_t size, DICEReadCallback callback);
    
    /// Write to application section
    void WriteAppSection(uint32_t offset, const uint8_t* data, size_t size, DICEWriteCallback callback);

    // --- Output group / input params writes: read-modify-write, changed quadlets only.
    /// Writes the quadlets of `after` that differ from `before`, then the notices
    /// focusrite.rs sends: DimMute if 0x00-0x07 changed, OutputSrc if 0x08-0x33 did.
    void WriteOutputGroupDiff(const OutputGroupState& before, const OutputGroupState& after,
                              VoidCallback callback);
    void WriteInputParamsDiff(const InputParams& before, const InputParams& after, VoidCallback callback);

    // --- TCAT extension router and command section (absolute DICE offsets).
    void ReadRouterTable(SPro24DspRouter::RateMode mode,
                         std::function<void(IOReturn, std::vector<uint32_t>)> callback);
    /// Writes `table` to the router section and loads it into `mode`'s current config.
    void LoadRouterTable(SPro24DspRouter::RateMode mode, std::vector<uint32_t> table,
                         VoidCallback callback);
    /// Runs one extension command: waits for the section to be idle, writes the
    /// opcode, waits for EXECUTE to clear and returns the command's result word.
    void RunExtensionCommand(uint32_t opcode, uint32_t timeoutMs,
                             std::function<void(IOReturn, uint32_t)> callback);
    /// Polls the opcode register (every 10 ms on the driver timer) until EXECUTE clears.
    void AwaitExtensionCommandIdle(uint32_t pollsLeft, std::function<void(IOReturn)> callback);
    [[nodiscard]] size_t RouterCapacity() const noexcept;

    // --- Control provider state (mirrors OrpheusProtocol's queueing discipline).
    void ReadControlsState(VoidCallback done);
    bool CommitControls();
    void FinishControlWrite(AudioModel::ParameterId id, IOReturn status, Completion done);
    void QueueControl(std::function<void(SPro24DspProtocol&)> op, Completion refuse);
    void ReleaseControl();
    void ResetControls();
    [[nodiscard]] std::shared_ptr<SPro24DspProtocol> SharedSelf();

    static constexpr size_t kMaxControlWaiters = 16;
    Scheduling::ITimerScheduler* timerScheduler_{nullptr};
    SPro24DspControlsState controls_{};
    std::optional<::ASFW::Audio::Controls::ControlSnapshot> controlSnapshot_;
    std::deque<std::function<void()>> controlWaiters_;
    bool controlsReady_{false};
    bool controlBusy_{false};
    uint64_t controlsEpoch_{0};
    uint64_t controlRouteEpoch_{0};
};

} // namespace ASFW::Audio::DICE::Focusrite
