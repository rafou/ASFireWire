// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024 ASFireWire Project
//
// SPro24DspProtocol.cpp - Focusrite Saffire Pro 24 DSP protocol implementation

#include "SPro24DspProtocol.hpp"
#include "SPro24DspRouting.hpp"
#include "../Core/DiceDeviceIo.hpp"
#include "../../../Controls/ControlNotifications.hpp"
#include "../../../Model/Topology/Validate.hpp"
#include "../../../../Common/CallbackUtils.hpp"
#include "../../../../Common/WireFormat.hpp"
#include "../../../../Logging/Logging.hpp"
#include <DriverKit/IOLib.h>
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <type_traits>
#include <utility>
#include <variant>

namespace ASFW::Audio::DICE::Focusrite {

// ============================================================================
// Wire Format Helpers
// ============================================================================

namespace {

// Before a clock change: how long to wait for a DSP still coming up.
constexpr uint32_t kDspReadyTimeoutMs = 5000;
constexpr uint32_t kDspPollMs = 50;

// DICE-relative offset of the application section, or nothing when the device
// exposes no TCAT extension sections.
[[nodiscard]] std::optional<uint32_t> ApplicationBase(DiceDeviceIo& io) {
    const auto sections = io.ReadExtensionSections();
    if (!sections) {
        return std::nullopt;
    }
    const uint32_t base = ExtensionAbsoluteOffset(sections->application);
    if (base == kDICEExtensionOffset || sections->application.size <= Offsets::kDspStatus) {
        return std::nullopt;
    }
    return base;
}

// Polls the DSP status until running and not busy. A read that fails counts as
// "not yet": the device answered nothing for a while during a DSP restart.
// Elapsed time comes from the clock: a read that times out costs far more than
// one poll step.
[[nodiscard]] bool AwaitDspReady(DiceDeviceIo& io, uint32_t appBase, uint32_t timeoutMs,
                                 uint32_t& waitedMs, uint32_t& lastStatus) {
    const uint64_t startMs = io.Clock().NowMs();
    for (;;) {
        const auto status = io.ReadQuad(appBase + Offsets::kDspStatus);
        waitedMs = static_cast<uint32_t>(io.Clock().NowMs() - startMs);
        if (status) {
            lastStatus = *status;
            if ((*status & DspStatus::kRunning) != 0 && (*status & DspStatus::kBusy) == 0) {
                return true;
            }
        }
        if (waitedMs >= timeoutMs) {
            return false;
        }
        io.Clock().SleepMs(kDspPollMs);
    }
}

} // anonymous namespace

// ============================================================================
// SPro24DspProtocol Implementation
// ============================================================================

SPro24DspProtocol::SPro24DspProtocol(Protocols::Ports::FireWireBusOps& busOps,
                                     Protocols::Ports::FireWireBusInfo& busInfo,
                                     Discovery::DeviceRegistry& routeRegistry,
                                     const Discovery::DeviceRouteToken& route,
                                     IRM::IRMClient* irmClient,
                                     DiceWaitClock& waitClock,
                                     DiceNotificationRouter* notifications,
                                     Scheduling::ITimerScheduler* preparationScheduler)
    : tcat_(busOps, busInfo, routeRegistry, route, irmClient, waitClock, notifications,
            TCAT::DICETcatRuntimePolicy{
                // PEEK verified in Pro24DSPFirmware: frmwload_cmd_test_action
                // 0x6A384, sub-command 2 (TCD2210, TCAT loader mailbox).
                .verifiedLoaderPeek = true,
                .clockChangeObserver = this,
                .waitForInitialDsp = true,
            }, preparationScheduler)
    , timerScheduler_(preparationScheduler)
{
    ASFW_LOG(DICE, "SPro24DspProtocol created for GUID=0x%016llx node=0x%04x", route.guid,
             route.nodeId);
}

// A clock change while the DSP was still applying the previous one left it
// running with digital zero on capture 1/2 until a DoDspReset (hardware,
// 2026-10-11). MixControl itself sends DSP messages only while the DSP reports
// running and not busy (MaybeSendDspMsg, IsDspRunning/IsDeviceBusy). Wait for
// that before the clock write. Nothing to do on a device without the
// extension sections (the wire simulator has none).
void SPro24DspProtocol::BeforeClockSelectWrite(DiceDeviceIo& io) noexcept {
    const auto appBase = ApplicationBase(io);
    if (!appBase) {
        ASFW_LOG(DICE, "[SPro24Dsp] DSP state unreadable before the clock change; not waiting");
        return;
    }
    uint32_t waitedMs = 0;
    uint32_t status = 0;
    if (!AwaitDspReady(io, *appBase, kDspReadyTimeoutMs, waitedMs, status)) {
        ASFW_LOG_ERROR(DICE, "[SPro24Dsp] DSP not ready after %u ms (status=0x%08x); changing the "
                       "clock anyway", waitedMs, status);
    } else if (waitedMs > 0) {
        ASFW_LOG(DICE, "[SPro24Dsp] DSP ready after %u ms; changing the clock", waitedMs);
    }
}

bool SPro24DspProtocol::GetRuntimeAudioStreamCaps(AudioStreamRuntimeCaps& outCaps) const {
    return tcat_.GetRuntimeAudioStreamCaps(outCaps);
}

IOReturn SPro24DspProtocol::Initialize() {
    InitializeAsync([](IOReturn status) {
        if (status != kIOReturnSuccess) {
            ASFW_LOG(DICE, "SPro24DspProtocol initialization failed: 0x%x", status);
        }
    });
    return kIOReturnSuccess;
}

void SPro24DspProtocol::InitializeAsync(InitCallback callback) {
    ASFW_LOG(DICE, "SPro24DspProtocol::InitializeAsync defers generic DICE discovery to TCAT runtime");
    callback(tcat_.Initialize());
}

void SPro24DspProtocol::HandleExtensionSectionsRead(IOReturn status,
                                                    ExtensionSections sections,
                                                    InitCallback callback) {
    if (status != kIOReturnSuccess) {
        ASFW_LOG(DICE, "Failed to read Focusrite extension sections: 0x%x", status);
        callback(status);
        return;
    }

    extensionSections_ = sections;
    appSectionBase_ = ASFW::Audio::DICE::ExtensionAbsoluteOffset(extensionSections_.application);
    commandSectionBase_ = ASFW::Audio::DICE::ExtensionAbsoluteOffset(extensionSections_.command);
    routerSectionBase_ = ASFW::Audio::DICE::ExtensionAbsoluteOffset(extensionSections_.router);
    currentConfigBase_ = ASFW::Audio::DICE::ExtensionAbsoluteOffset(extensionSections_.currentConfig);

    if (appSectionBase_ == kDICEExtensionOffset ||
        commandSectionBase_ == kDICEExtensionOffset ||
        routerSectionBase_ == kDICEExtensionOffset ||
        currentConfigBase_ == kDICEExtensionOffset) {
        ASFW_LOG(DICE, "SPro24DspProtocol: missing required TCAT extension section(s)");
        callback(kIOReturnNotFound);
        return;
    }

    extensionsLoaded_ = true;
    ASFW_LOG(DICE,
             "SPro24DspProtocol: loaded Focusrite extension bases app=0x%08x cmd=0x%08x router=0x%08x current=0x%08x",
             appSectionBase_,
             commandSectionBase_,
             routerSectionBase_,
             currentConfigBase_);
    callback(kIOReturnSuccess);
}

void SPro24DspProtocol::EnsureExtensionsLoaded(VoidCallback callback) {
    if (extensionsLoaded_) {
        callback(kIOReturnSuccess);
        return;
    }

    tcat_.Transaction().ReadExtensionSections(
        [this, callback = std::move(callback)](IOReturn status, ExtensionSections sections) mutable {
            HandleExtensionSectionsRead(status, sections, std::move(callback));
        });
}

IOReturn SPro24DspProtocol::Shutdown() {
    ASFW_LOG(DICE, "SPro24DspProtocol::Shutdown");
    ResetControls();
    extensionSections_ = {};
    appSectionBase_ = 0;
    commandSectionBase_ = 0;
    routerSectionBase_ = 0;
    currentConfigBase_ = 0;
    extensionsLoaded_ = false;
    return tcat_.Shutdown();
}


IOReturn SPro24DspProtocol::StopDuplex() {
    return tcat_.StopDuplex();
}

void SPro24DspProtocol::UpdateRuntimeContext(const Discovery::DeviceRouteToken& route,
                                             std::shared_ptr<ASFW::AVC::IAvcUnit> avcUnit) {
    tcat_.UpdateRuntimeContext(route, std::move(avcUnit));
    // A reset or rebind invalidates everything read back: re-read on the next Prepare.
    if (controlsReady_ && route.routeEpoch != controlRouteEpoch_) {
        ResetControls();
    }
}

void SPro24DspProtocol::ReadAppQuad(uint32_t offset,
                                    std::function<void(IOReturn, uint32_t)> callback) {
    EnsureExtensionsLoaded([this, offset, callback = std::move(callback)](IOReturn status) mutable {
        if (status != kIOReturnSuccess) {
            callback(status, 0);
            return;
        }

        (void)tcat_.IO().ReadQuadBE(MakeDICEAddress(appSectionBase_ + offset),
                              [callback = std::move(callback)](Async::AsyncStatus transportStatus, uint32_t value) mutable {
                                  callback(Protocols::Ports::MapAsyncStatusToIOReturn(transportStatus), value);
                              });
    });
}

void SPro24DspProtocol::WriteAppQuad(uint32_t offset, uint32_t value, VoidCallback callback) {
    EnsureExtensionsLoaded([this, offset, value, callback = std::move(callback)](IOReturn status) mutable {
        if (status != kIOReturnSuccess) {
            callback(status);
            return;
        }

        (void)tcat_.IO().WriteQuadBE(MakeDICEAddress(appSectionBase_ + offset),
                               value,
                               [callback = std::move(callback)](Async::AsyncStatus transportStatus) mutable {
                                   callback(Protocols::Ports::MapAsyncStatusToIOReturn(transportStatus));
                               });
    });
}

void SPro24DspProtocol::ReadAppSection(uint32_t offset, size_t size, DICEReadCallback callback) {
    auto callbackState = Common::ShareCallback(std::move(callback));
    EnsureExtensionsLoaded([this, offset, size, callbackState](IOReturn status) mutable {
        if (status != kIOReturnSuccess) {
            Common::InvokeSharedCallback(callbackState, status, nullptr, size_t{0});
            return;
        }

        (void)tcat_.IO().ReadBlock(MakeDICEAddress(appSectionBase_ + offset),
                             static_cast<uint32_t>(size),
                             [callbackState](Async::AsyncStatus transportStatus, std::span<const uint8_t> payload) {
                                 const bool hasPayload = transportStatus == Async::AsyncStatus::kSuccess ||
                                                         transportStatus == Async::AsyncStatus::kShortRead;
                                 Common::InvokeSharedCallback(callbackState,
                                                             Protocols::Ports::MapAsyncStatusToIOReturn(transportStatus),
                                                             hasPayload ? payload.data() : nullptr,
                                                             hasPayload ? payload.size() : size_t{0});
                             });
    });
}

void SPro24DspProtocol::WriteAppSection(uint32_t offset,
                                        const uint8_t* data,
                                        size_t size,
                                        DICEWriteCallback callback) {
    auto callbackState = Common::ShareCallback(std::move(callback));
    EnsureExtensionsLoaded([this, offset, data, size, callbackState](IOReturn status) mutable {
        if (status != kIOReturnSuccess) {
            Common::InvokeSharedCallback(callbackState, status);
            return;
        }

        (void)tcat_.IO().WriteBlock(MakeDICEAddress(appSectionBase_ + offset),
                              std::span<const uint8_t>(data, size),
                              [callbackState](Async::AsyncStatus transportStatus) {
                                  Common::InvokeSharedCallback(callbackState,
                                                              Protocols::Ports::MapAsyncStatusToIOReturn(transportStatus));
                              });
    });
}

void SPro24DspProtocol::SendSwNotice(SwNotice notice, VoidCallback callback) {
    WriteAppQuad(kSwNoticeOffset, static_cast<uint32_t>(notice), std::move(callback));
}

void SPro24DspProtocol::EnableDsp(bool enable, VoidCallback callback) {
    uint32_t value = enable ? 1 : 0;
    WriteAppQuad(kDspEnableOffset, value, [this, callback](IOReturn status) {
        if (status != kIOReturnSuccess) {
            callback(status);
            return;
        }
        SendSwNotice(SwNotice::DspChanged, callback);
    });
}

void SPro24DspProtocol::GetEffectParams(ResultCallback<EffectGeneralParams> callback) {
    ReadAppQuad(kEffectGeneralOffset, [callback](IOReturn status, uint32_t value) {
        if (status != kIOReturnSuccess) {
            callback(status, {});
            return;
        }
        uint8_t data[4];
        ASFW::FW::WriteBE32(data, value);
        callback(kIOReturnSuccess, EffectGeneralParams::Deserialize(data));
    });
}

void SPro24DspProtocol::SetEffectParams(const EffectGeneralParams& params, VoidCallback callback) {
    uint8_t data[4];
    params.Serialize(data);
    const uint32_t value = ASFW::FW::ReadBE32(data);
    
    WriteAppQuad(kEffectGeneralOffset, value, [this, callback](IOReturn status) {
        if (status != kIOReturnSuccess) {
            callback(status);
            return;
        }
        SendSwNotice(SwNotice::EffectChanged, callback);
    });
}

void SPro24DspProtocol::GetCompressorState(ResultCallback<CompressorState> callback) {
    ReadAppSection(kCoefOffset + CoefBlock::kCompressor * kCoefBlockSize, 2 * kCoefBlockSize,
                   [callback](IOReturn status, const uint8_t* data, size_t /*size*/) {
        if (status != kIOReturnSuccess) {
            callback(status, {});
            return;
        }
        callback(kIOReturnSuccess, CompressorState::Deserialize(data));
    });
}

void SPro24DspProtocol::SetCompressorState(const CompressorState& state, VoidCallback callback) {
    // Note: Need to allocate buffer that outlives async call
    auto buffer = std::make_shared<std::array<uint8_t, 2 * kCoefBlockSize>>();
    state.Serialize(buffer->data());

    WriteAppSection(kCoefOffset + CoefBlock::kCompressor * kCoefBlockSize, buffer->data(), buffer->size(),
                    [this, callback, buffer](IOReturn status) {
        if (status != kIOReturnSuccess) {
            callback(status);
            return;
        }
        // Per Linux reference (spro24dsp.rs lines 770-771): send BOTH
        // CompCh0 and CompCh1 SW notices after compressor state write.
        SendSwNotice(SwNotice::CompCh0, [this, callback](IOReturn s1) {
            if (s1 != kIOReturnSuccess) {
                callback(s1);
                return;
            }
            SendSwNotice(SwNotice::CompCh1, callback);
        });
    });
}

void SPro24DspProtocol::GetReverbState(ResultCallback<ReverbState> callback) {
    ReadAppSection(kCoefOffset + CoefBlock::kReverb * kCoefBlockSize, kCoefBlockSize,
                   [callback](IOReturn status, const uint8_t* data, size_t /*size*/) {
        if (status != kIOReturnSuccess) {
            callback(status, {});
            return;
        }
        callback(kIOReturnSuccess, ReverbState::Deserialize(data));
    });
}

void SPro24DspProtocol::SetReverbState(const ReverbState& state, VoidCallback callback) {
    auto buffer = std::make_shared<std::array<uint8_t, kCoefBlockSize>>();
    state.Serialize(buffer->data());

    WriteAppSection(kCoefOffset + CoefBlock::kReverb * kCoefBlockSize, buffer->data(), buffer->size(),
                    [this, callback, buffer](IOReturn status) {
        if (status != kIOReturnSuccess) {
            callback(status);
            return;
        }
        // Per Linux reference: REVERB_SW_NOTICE = 0x1A
        SendSwNotice(SwNotice::Reverb, callback);
    });
}

// ============================================================================
// Input params and output group: read-modify-write of changed quadlets only
// ============================================================================

namespace {

using QuadWrites = std::vector<std::pair<uint32_t, uint32_t>>; // app-section offset, value

// Quadlets of `after` that differ from `before`, as app-section writes.
template <size_t N>
[[nodiscard]] QuadWrites ChangedQuadlets(const std::array<uint8_t, N>& before,
                                         const std::array<uint8_t, N>& after,
                                         uint32_t sectionOffset) {
    QuadWrites writes;
    for (size_t pos = 0; pos + 4 <= N; pos += 4) {
        if (!std::equal(before.begin() + pos, before.begin() + pos + 4, after.begin() + pos)) {
            writes.emplace_back(sectionOffset + static_cast<uint32_t>(pos), ASFW::FW::ReadBE32(after.data() + pos));
        }
    }
    return writes;
}

[[nodiscard]] bool AnyIn(const QuadWrites& writes, uint32_t from, uint32_t to) {
    return std::ranges::any_of(writes, [&](const auto& w) { return w.first >= from && w.first < to; });
}

// Command section polling: MixControl gives up on a command after 400 ms
// (kEAP_CMD_TIMEOUT); storing to flash is given longer. Polls every 10 ms.
constexpr uint32_t kCommandPollMs = 10;
constexpr uint32_t kRouterCommandTimeoutMs = 400;
constexpr uint32_t kFlashCommandTimeoutMs = 5000;
// TCAT extension command "store all configurations to flash" (cmd_section.rs
// STORE_FLASH_CONFIG_VALUE); takes no rate-mode flag.
constexpr uint32_t kStoreConfigToFlash = 0x00000005;
constexpr size_t kRouterTableCapacityLimit = 128;

[[nodiscard]] uint32_t RateFlag(SPro24DspRouter::RateMode mode) {
    return mode == SPro24DspRouter::RateMode::Low ? ExtensionCommandOpcode::kRateLow
                                                  : ExtensionCommandOpcode::kRateMiddle;
}

[[nodiscard]] const char* ModeName(SPro24DspRouter::RateMode mode) {
    return mode == SPro24DspRouter::RateMode::Low ? "low" : "mid";
}

} // namespace

void SPro24DspProtocol::GetInputParams(ResultCallback<InputParams> callback) {
    ReadAppSection(kInputOffset, kInputParamsSize,
                   [callback](IOReturn status, const uint8_t* data, size_t size) {
        if (status != kIOReturnSuccess) {
            callback(status, {});
            return;
        }
        if (size < kInputParamsSize) {
            callback(kIOReturnUnderrun, {});
            return;
        }
        callback(kIOReturnSuccess, InputParams::Deserialize(data));
    });
}

void SPro24DspProtocol::SetInputParams(const InputParams& params, VoidCallback callback) {
    GetInputParams([this, params, callback = std::move(callback)](IOReturn status, InputParams current) mutable {
        if (status != kIOReturnSuccess) {
            callback(status);
            return;
        }
        WriteInputParamsDiff(current, params, std::move(callback));
    });
}

void SPro24DspProtocol::GetOutputGroupState(ResultCallback<OutputGroupState> callback) {
    ReadAppSection(kOutputGroupOffset, kOutputGroupStateSize,
                   [callback](IOReturn status, const uint8_t* data, size_t size) {
        if (status != kIOReturnSuccess) {
            callback(status, {});
            return;
        }
        if (size < kOutputGroupStateSize) {
            callback(kIOReturnUnderrun, {});
            return;
        }
        callback(kIOReturnSuccess, OutputGroupState::Deserialize(data));
    });
}

// Was a whole-block write from a zero-filled image: it cleared the undecoded
// ranges (0x34-0x47, 0x4C-0x4F) and always sent both notices. Read the block
// first and write only what changed, as focusrite.rs does.
void SPro24DspProtocol::SetOutputGroupState(const OutputGroupState& state, VoidCallback callback) {
    GetOutputGroupState([this, state, callback = std::move(callback)](IOReturn status, OutputGroupState current) mutable {
        if (status != kIOReturnSuccess) {
            callback(status);
            return;
        }
        WriteOutputGroupDiff(current, state, std::move(callback));
    });
}

namespace {

// Writes `writes` one quadlet at a time into the application section, then calls `done`.
void WriteQuadsInOrder(Protocols::Ports::ProtocolRegisterIO& io, uint32_t appBase,
                       std::shared_ptr<const QuadWrites> writes, size_t index,
                       std::function<void(IOReturn)> done) {
    if (index >= writes->size()) {
        done(kIOReturnSuccess);
        return;
    }
    const auto [offset, value] = (*writes)[index];
    (void)io.WriteQuadBE(MakeDICEAddress(appBase + offset), value,
                         [&io, appBase, writes, index, done = std::move(done)](Async::AsyncStatus status) mutable {
        const IOReturn result = Protocols::Ports::MapAsyncStatusToIOReturn(status);
        if (result != kIOReturnSuccess) {
            done(result);
            return;
        }
        WriteQuadsInOrder(io, appBase, std::move(writes), index + 1, std::move(done));
    });
}

} // namespace

void SPro24DspProtocol::WriteInputParamsDiff(const InputParams& before, const InputParams& after,
                                             VoidCallback callback) {
    std::array<uint8_t, kInputParamsSize> oldRaw{};
    std::array<uint8_t, kInputParamsSize> newRaw{};
    before.Serialize(oldRaw.data());
    after.Serialize(newRaw.data());
    auto writes = std::make_shared<const QuadWrites>(ChangedQuadlets(oldRaw, newRaw, kInputOffset));
    if (writes->empty()) {
        callback(kIOReturnSuccess);
        return;
    }
    EnsureExtensionsLoaded([this, writes, callback = std::move(callback)](IOReturn status) mutable {
        if (status != kIOReturnSuccess) {
            callback(status);
            return;
        }
        WriteQuadsInOrder(tcat_.IO(), appSectionBase_, writes, 0,
                          [this, callback = std::move(callback)](IOReturn written) mutable {
            if (written != kIOReturnSuccess) {
                callback(written);
                return;
            }
            SendSwNotice(SwNotice::InputParams, std::move(callback));
        });
    });
}

void SPro24DspProtocol::WriteOutputGroupDiff(const OutputGroupState& before, const OutputGroupState& after,
                                             VoidCallback callback) {
    std::array<uint8_t, kOutputGroupStateSize> oldRaw{};
    std::array<uint8_t, kOutputGroupStateSize> newRaw{};
    before.Serialize(oldRaw.data());
    after.Serialize(newRaw.data());
    auto writes = std::make_shared<const QuadWrites>(ChangedQuadlets(oldRaw, newRaw, kOutputGroupOffset));
    if (writes->empty()) {
        callback(kIOReturnSuccess);
        return;
    }
    const bool dimMute = AnyIn(*writes, kOutputGroupOffset, kOutputGroupOffset + 0x08);
    const bool source = AnyIn(*writes, kOutputGroupOffset + 0x08, kOutputGroupOffset + 0x34);
    EnsureExtensionsLoaded([this, writes, dimMute, source, callback = std::move(callback)](IOReturn status) mutable {
        if (status != kIOReturnSuccess) {
            callback(status);
            return;
        }
        WriteQuadsInOrder(tcat_.IO(), appSectionBase_, writes, 0,
                          [this, dimMute, source, callback = std::move(callback)](IOReturn written) mutable {
            if (written != kIOReturnSuccess) {
                callback(written);
                return;
            }
            auto sendSource = [this, source, callback = std::move(callback)](IOReturn noticed) mutable {
                if (noticed != kIOReturnSuccess || !source) {
                    callback(noticed);
                    return;
                }
                SendSwNotice(SwNotice::OutputSrc, std::move(callback));
            };
            if (dimMute) {
                SendSwNotice(SwNotice::DimMute, std::move(sendSource));
            } else {
                sendSource(kIOReturnSuccess);
            }
        });
    });
}

// ============================================================================
// TCAT router and extension commands (asynchronous, controller queue)
// ============================================================================

size_t SPro24DspProtocol::RouterCapacity() const noexcept {
    const size_t section = extensionSections_.router.size >= 8 ? extensionSections_.router.size / 4U - 1U : 0U;
    return std::min(section, kRouterTableCapacityLimit);
}

void SPro24DspProtocol::ReadRouterTable(SPro24DspRouter::RateMode mode,
                                        std::function<void(IOReturn, std::vector<uint32_t>)> callback) {
    EnsureExtensionsLoaded([this, mode, callback = std::move(callback)](IOReturn status) mutable {
        if (status != kIOReturnSuccess) {
            callback(status, {});
            return;
        }
        const uint32_t modeOffset = mode == SPro24DspRouter::RateMode::Low ? CurrentConfigOffset::kLowRouter
                                                                          : CurrentConfigOffset::kMiddleRouter;
        if (modeOffset + 4U > extensionSections_.currentConfig.size) {
            callback(kIOReturnSuccess, {}); // no table kept for this rate mode
            return;
        }
        const uint32_t base = ExtensionAbsoluteOffset(extensionSections_.currentConfig, modeOffset);
        (void)tcat_.IO().ReadQuadBE(MakeDICEAddress(base),
            [this, base, mode, callback = std::move(callback)](Async::AsyncStatus quadStatus, uint32_t count) mutable {
            const IOReturn result = Protocols::Ports::MapAsyncStatusToIOReturn(quadStatus);
            if (result != kIOReturnSuccess) {
                callback(result, {});
                return;
            }
            if (count == 0) {
                callback(kIOReturnSuccess, {});
                return;
            }
            if (count > RouterCapacity()) {
                ASFW_LOG(DICE, "[SPro24Dsp] %{public}s router table claims %u entries (capacity %zu)",
                         ModeName(mode), count, RouterCapacity());
                callback(kIOReturnDeviceError, {});
                return;
            }
            (void)tcat_.IO().ReadBlock(MakeDICEAddress(base + 4U), count * 4U,
                [count, callback = std::move(callback)](Async::AsyncStatus blockStatus,
                                                        std::span<const uint8_t> payload) mutable {
                const IOReturn read = Protocols::Ports::MapAsyncStatusToIOReturn(blockStatus);
                if (read != kIOReturnSuccess || payload.size() < count * 4U) {
                    callback(read != kIOReturnSuccess ? read : kIOReturnUnderrun, {});
                    return;
                }
                std::vector<uint32_t> table(count);
                for (uint32_t i = 0; i < count; ++i) {
                    // Keep destination and source; the high half is a live peak meter.
                    table[i] = ASFW::FW::ReadBE32(payload.data() + i * 4U) & 0xFFFFU;
                }
                callback(kIOReturnSuccess, std::move(table));
            });
        });
    });
}

void SPro24DspProtocol::AwaitExtensionCommandIdle(uint32_t pollsLeft, std::function<void(IOReturn)> callback) {
    auto self = SharedSelf();
    if (!self) {
        callback(kIOReturnAborted);
        return;
    }
    (void)tcat_.IO().ReadQuadBE(MakeDICEAddress(commandSectionBase_ + ExtensionCommandOffset::kOpcode),
        [self, pollsLeft, callback = std::move(callback)](Async::AsyncStatus status, uint32_t opcode) mutable {
        const IOReturn result = Protocols::Ports::MapAsyncStatusToIOReturn(status);
        if (result != kIOReturnSuccess) {
            callback(result);
            return;
        }
        if ((opcode & ExtensionCommandOpcode::kExecute) == 0) {
            callback(kIOReturnSuccess);
            return;
        }
        if (pollsLeft == 0) {
            callback(kIOReturnTimeout);
            return;
        }
        auto again = [self, pollsLeft, callback = std::move(callback)]() mutable {
            self->AwaitExtensionCommandIdle(pollsLeft - 1, std::move(callback));
        };
        if (self->timerScheduler_ == nullptr) {
            again();
            return;
        }
        auto shared = std::make_shared<decltype(again)>(std::move(again));
        const auto token = self->timerScheduler_->ScheduleAfter(uint64_t{kCommandPollMs} * 1'000'000ULL,
                                                                [shared] { (*shared)(); });
        if (token == Scheduling::kInvalidTimerToken) {
            (*shared)(); // no timer: poll again at once rather than hang
        }
    });
}

void SPro24DspProtocol::RunExtensionCommand(uint32_t opcode, uint32_t timeoutMs,
                                            std::function<void(IOReturn, uint32_t)> callback) {
    auto self = SharedSelf();
    if (!self) {
        callback(kIOReturnAborted, 0);
        return;
    }
    const uint32_t polls = std::max<uint32_t>(1, timeoutMs / kCommandPollMs);
    EnsureExtensionsLoaded([self, opcode, polls, callback = std::move(callback)](IOReturn loaded) mutable {
        if (loaded != kIOReturnSuccess) {
            callback(loaded, 0);
            return;
        }
        // Another command (the driver's own bring-up, or MixControl's) may still run.
        self->AwaitExtensionCommandIdle(polls, [self, opcode, polls, callback = std::move(callback)](IOReturn idle) mutable {
            if (idle != kIOReturnSuccess) {
                callback(idle, 0);
                return;
            }
            const uint32_t commandBase = self->commandSectionBase_;
            (void)self->tcat_.IO().WriteQuadBE(MakeDICEAddress(commandBase + ExtensionCommandOffset::kOpcode), opcode,
                [self, commandBase, polls, callback = std::move(callback)](Async::AsyncStatus status) mutable {
                const IOReturn written = Protocols::Ports::MapAsyncStatusToIOReturn(status);
                if (written != kIOReturnSuccess) {
                    callback(written, 0);
                    return;
                }
                self->AwaitExtensionCommandIdle(polls, [self, commandBase, callback = std::move(callback)](IOReturn done) mutable {
                    if (done != kIOReturnSuccess) {
                        callback(done, 0);
                        return;
                    }
                    (void)self->tcat_.IO().ReadQuadBE(MakeDICEAddress(commandBase + ExtensionCommandOffset::kReturn),
                        [callback = std::move(callback)](Async::AsyncStatus readStatus, uint32_t value) mutable {
                        callback(Protocols::Ports::MapAsyncStatusToIOReturn(readStatus), value);
                    });
                });
            });
        });
    });
}

void SPro24DspProtocol::LoadRouterTable(SPro24DspRouter::RateMode mode, std::vector<uint32_t> table,
                                        VoidCallback callback) {
    auto self = SharedSelf();
    if (!self) {
        callback(kIOReturnAborted);
        return;
    }
    auto image = std::make_shared<std::vector<uint8_t>>(4U + table.size() * 4U);
    ASFW::FW::WriteBE32(image->data(), static_cast<uint32_t>(table.size()));
    for (size_t i = 0; i < table.size(); ++i) {
        ASFW::FW::WriteBE32(image->data() + 4U + i * 4U, table[i]);
    }
    // The router section is the command's input: never rewrite it under a running command.
    AwaitExtensionCommandIdle(kRouterCommandTimeoutMs / kCommandPollMs,
        [self, mode, image, callback = std::move(callback)](IOReturn idle) mutable {
        if (idle != kIOReturnSuccess) {
            callback(idle);
            return;
        }
        (void)self->tcat_.IO().WriteBlock(MakeDICEAddress(self->routerSectionBase_), std::span<const uint8_t>(*image),
            [self, mode, image, callback = std::move(callback)](Async::AsyncStatus status) mutable {
            const IOReturn written = Protocols::Ports::MapAsyncStatusToIOReturn(status);
            if (written != kIOReturnSuccess) {
                callback(written);
                return;
            }
            const uint32_t opcode = ExtensionCommandOpcode::kExecute | RateFlag(mode) |
                                    ExtensionCommandOpcode::kLoadRouter;
            self->RunExtensionCommand(opcode, kRouterCommandTimeoutMs,
                [mode, opcode, entries = image->size() / 4U - 1U, callback = std::move(callback)](IOReturn status, uint32_t result) mutable {
                ASFW_LOG(DICE, "[SPro24Dsp] router %{public}s: %zu entries loaded kr=0x%x (opcode=0x%08x return=0x%08x)",
                         ModeName(mode), entries, status, opcode, result);
                callback(status);
            });
        });
    });
}

// ============================================================================
// Device Controls (IControlProvider)
// ============================================================================

std::shared_ptr<SPro24DspProtocol> SPro24DspProtocol::SharedSelf() {
    return std::static_pointer_cast<SPro24DspProtocol>(weak_from_this().lock());
}

void SPro24DspProtocol::ReadControlsState(VoidCallback done) {
    namespace R = SPro24DspRouter;
    auto self = SharedSelf();
    if (!self) {
        done(kIOReturnNotReady);
        return;
    }
    auto next = std::make_shared<SPro24DspControlsState>();
    AudioStreamRuntimeCaps caps{};
    if (tcat_.GetRuntimeAudioStreamCaps(caps)) {
        next->liveMode = R::RateModeFor(caps.sampleRateHz);
    }
    GetInputParams([self, next, done = std::move(done)](IOReturn status, InputParams inputs) mutable {
        if (status != kIOReturnSuccess) { done(status); return; }
        next->inputs = inputs;
        self->GetOutputGroupState([self, next, done = std::move(done)](IOReturn status, OutputGroupState outputs) mutable {
            if (status != kIOReturnSuccess) { done(status); return; }
            next->outputs = outputs;
            self->ReadRouterTable(R::RateMode::Low,
                [self, next, done = std::move(done)](IOReturn status, std::vector<uint32_t> low) mutable {
                if (status != kIOReturnSuccess) { done(status); return; }
                next->routers[static_cast<size_t>(R::RateMode::Low)] = std::move(low);
                self->ReadRouterTable(R::RateMode::Middle,
                    [self, next, done = std::move(done)](IOReturn status, std::vector<uint32_t> middle) mutable {
                    if (status != kIOReturnSuccess) { done(status); return; }
                    next->routers[static_cast<size_t>(R::RateMode::Middle)] = std::move(middle);
                    self->controls_ = std::move(*next);
                    done(kIOReturnSuccess);
                });
            });
        });
    });
}

void SPro24DspProtocol::Prepare(std::function<void(IOReturn)> done) {
    if (controlsReady_) {
        // A rate change rebuilds the Core Audio graph: show the live mode's table.
        AudioStreamRuntimeCaps caps{};
        if (tcat_.GetRuntimeAudioStreamCaps(caps)) {
            controls_.liveMode = SPro24DspRouter::RateModeFor(caps.sampleRateHz);
        }
        done(CommitControls() ? kIOReturnSuccess : kIOReturnError);
        return;
    }
    if (controlBusy_) {
        done(kIOReturnNotReady);
        return;
    }
    auto self = SharedSelf();
    if (!self) {
        done(kIOReturnNotReady);
        return;
    }
    controlBusy_ = true;
    const auto epoch = controlsEpoch_;
    ReadControlsState([self, epoch, done = std::move(done)](IOReturn status) {
        if (epoch != self->controlsEpoch_) {
            done(kIOReturnAborted);
            return;
        }
        if (status == kIOReturnSuccess) {
            self->controlsReady_ = true;
            self->controlRouteEpoch_ = self->tcat_.IO().Route().routeEpoch;
            if (!self->CommitControls()) {
                self->controlsReady_ = false;
                status = kIOReturnError;
            }
        }
        ASFW_LOG(DICE, "[SPro24Dsp] controls prepared kr=0x%x low=%zu mid=%zu entries", status,
                 self->controls_.routers[0].size(), self->controls_.routers[1].size());
        self->ReleaseControl();
        done(status);
    });
}

std::optional<::ASFW::Audio::Controls::ControlSnapshot> SPro24DspProtocol::CopySnapshot() const {
    return controlSnapshot_;
}

void SPro24DspProtocol::SetParameter(AudioModel::ParameterId id, const AudioModel::ParameterValue& value,
                                     Completion done) {
    namespace K = SPro24DspControl;
    if (!controlsReady_) {
        done(kIOReturnNotReady, false);
        return;
    }
    if (controlBusy_) {
        QueueControl([id, value, done](SPro24DspProtocol& self) { self.SetParameter(id, value, done); }, done);
        return;
    }
    auto self = SharedSelf();
    if (!self) {
        done(kIOReturnNotReady, false);
        return;
    }
    const uint32_t raw = id.value;
    const auto epoch = controlsEpoch_;
    auto finish = [self, epoch, id, done](IOReturn status) {
        if (epoch != self->controlsEpoch_) {
            done(kIOReturnAborted, false);
            return;
        }
        self->FinishControlWrite(id, status, done);
    };

    const bool instrument = raw >= K::InputInstrument && raw < K::InputInstrument + 2;
    const bool highGain = raw >= K::InputHighGain && raw < K::InputHighGain + 2;
    if (instrument || highGain) {
        const auto* choice = std::get_if<int64_t>(&value);
        if (choice == nullptr || (*choice != 0 && *choice != 1)) {
            done(kIOReturnBadArgument, false);
            return;
        }
        controlBusy_ = true;
        GetInputParams([self, raw, instrument, on = *choice == 1, finish](IOReturn status, InputParams current) {
            if (status != kIOReturnSuccess) {
                finish(status);
                return;
            }
            auto next = current;
            if (instrument) {
                next.micLevels[raw - K::InputInstrument] = on ? MicInputLevel::Instrument : MicInputLevel::Line;
            } else {
                next.lineLevels[raw - K::InputHighGain] = on ? LineInputLevel::High : LineInputLevel::Low;
            }
            self->WriteInputParamsDiff(current, next, finish);
        });
        return;
    }

    const bool volume = raw >= K::OutputVolume && raw < K::OutputVolume + 6;
    const bool outputMute = raw >= K::OutputMute && raw < K::OutputMute + 6;
    if (raw == K::MonitorMute || raw == K::MonitorDim || volume || outputMute) {
        const auto* flag = std::get_if<bool>(&value);
        const auto* level = std::get_if<double>(&value);
        if ((volume && (level == nullptr || !(*level >= kOutputVolumeMinDb && *level <= 0.0))) ||
            (!volume && flag == nullptr)) {
            done(kIOReturnBadArgument, false);
            return;
        }
        const bool on = flag != nullptr && *flag;
        const auto volumeValue = level != nullptr ? OutputVolumeFromDb(*level) : int8_t{0};
        controlBusy_ = true;
        GetOutputGroupState([self, raw, volume, outputMute, on, volumeValue, finish](IOReturn status,
                                                                                   OutputGroupState current) {
            if (status != kIOReturnSuccess) {
                finish(status);
                return;
            }
            auto next = current;
            if (raw == K::MonitorMute) next.muteEnabled = on;
            else if (raw == K::MonitorDim) next.dimEnabled = on;
            else if (volume) next.volumes[raw - K::OutputVolume] = volumeValue;
            else if (outputMute) next.volMutes[raw - K::OutputMute] = on;
            self->WriteOutputGroupDiff(current, next, finish);
        });
        return;
    }

    if (raw == K::SaveToDevice) {
        if (!std::holds_alternative<AudioModel::ActionTrigger>(value)) {
            done(kIOReturnBadArgument, false);
            return;
        }
        controlBusy_ = true;
        RunExtensionCommand(ExtensionCommandOpcode::kExecute | kStoreConfigToFlash, kFlashCommandTimeoutMs,
            [self, epoch, done](IOReturn status, uint32_t result) {
            ASFW_LOG(DICE, "[SPro24Dsp] store to flash kr=0x%x return=0x%08x", status, result);
            if (epoch != self->controlsEpoch_) {
                done(kIOReturnAborted, false);
                return;
            }
            self->ReleaseControl();
            done(status, AudioModel::ActionTrigger{});
        });
        return;
    }
    done(kIOReturnUnsupported, false);
}

void SPro24DspProtocol::SetRoute(AudioModel::NodeId router, AudioModel::PortId output,
                                 std::optional<AudioModel::PortId> input, Completion done) {
    namespace K = SPro24DspControl;
    namespace R = SPro24DspRouter;
    if (!controlsReady_) {
        done(kIOReturnNotReady, false);
        return;
    }
    if (controlBusy_) {
        QueueControl([router, output, input, done](SPro24DspProtocol& self) {
            self.SetRoute(router, output, input, done);
        }, done);
        return;
    }
    if (router.value < K::OutputRouter || router.value >= K::OutputRouter + R::kDestinations.size()) {
        done(kIOReturnUnsupported, false);
        return;
    }
    const size_t dst = router.value - K::OutputRouter;
    auto sources = std::make_shared<const std::vector<R::Source>>(R::Sources());
    size_t src = 0; // "Off" when no input is given
    if (input) {
        const uint32_t first = SPro24DspRouterInputPort(dst, 0);
        if (input->value < first || input->value >= first + sources->size()) {
            done(kIOReturnBadArgument, false);
            return;
        }
        src = input->value - first;
    }
    if (output.value != SPro24DspRouterOutputPort(dst)) {
        done(kIOReturnBadArgument, false);
        return;
    }
    auto self = SharedSelf();
    if (!self) {
        done(kIOReturnNotReady, false);
        return;
    }
    controlBusy_ = true;
    const auto epoch = controlsEpoch_;

    // Every rate mode keeps its own table; change the output in each mode the
    // device has a program for, translated to that mode's channel numbers. A
    // source the mode does not carry (ADAT 5-8 at 88.2/96 kHz) mutes the output there.
    auto applyMode = std::make_shared<std::function<void(size_t)>>();
    *applyMode = [self, epoch, dst, src, sources, router, done, applyMode](size_t m) {
        if (epoch != self->controlsEpoch_) {
            done(kIOReturnAborted, false);
            return;
        }
        if (m >= R::kRateModeCount) {
            // Confirm from the device, not from what was sent.
            self->ReadControlsState([self, epoch, router, done](IOReturn status) {
                if (epoch != self->controlsEpoch_) {
                    done(kIOReturnAborted, false);
                    return;
                }
                const bool committed = status == kIOReturnSuccess && self->CommitControls();
                self->ReleaseControl();
                if (!committed) {
                    done(status != kIOReturnSuccess ? status : kIOReturnError, false);
                    return;
                }
                const auto state = self->controlSnapshot_->state.routers.find(router);
                if (state == self->controlSnapshot_->state.routers.end() || state->second.activeBundles.empty()) {
                    done(kIOReturnError, false);
                    return;
                }
                const auto dstIndex = router.value - K::OutputRouter;
                done(kIOReturnSuccess,
                     int64_t(SPro24DspRouterInputPort(dstIndex, state->second.activeBundles.front().value - 1)));
            });
            return;
        }
        const auto mode = static_cast<R::RateMode>(m);
        const auto& table = self->controls_.routers[m];
        const uint8_t destination = R::kDestinations[dst].endpoint[m];
        const uint8_t source = (*sources)[src].endpoint[m].value_or(R::kMuteSource);
        if (table.empty()) {
            (*applyMode)(m + 1); // the device keeps no program for this rate mode
            return;
        }
        auto edited = R::WithRoute(table, destination, source, self->RouterCapacity());
        if (!edited) {
            ASFW_LOG(DICE, "[SPro24Dsp] router %{public}s full; route not added", ModeName(mode));
            self->FinishControlWrite(AudioModel::ParameterId{0}, kIOReturnNoSpace, done);
            return;
        }
        if (*edited == table) {
            (*applyMode)(m + 1);
            return;
        }
        self->LoadRouterTable(mode, std::move(*edited), [self, done, applyMode, m](IOReturn status) {
            if (status != kIOReturnSuccess) {
                self->FinishControlWrite(AudioModel::ParameterId{0}, status, done);
                return;
            }
            (*applyMode)(m + 1);
        });
    };
    (*applyMode)(0);
}

void SPro24DspProtocol::FinishControlWrite(AudioModel::ParameterId id, IOReturn status, Completion done) {
    auto self = SharedSelf();
    if (!self) {
        done(kIOReturnAborted, false);
        return;
    }
    const auto epoch = controlsEpoch_;
    // Re-read whatever the write touched, success or not: the snapshot shows the device.
    ReadControlsState([self, epoch, id, status, done](IOReturn readStatus) {
        if (epoch != self->controlsEpoch_) {
            done(kIOReturnAborted, false);
            return;
        }
        const bool committed = readStatus == kIOReturnSuccess && self->CommitControls();
        self->ReleaseControl();
        if (status != kIOReturnSuccess) {
            done(status, false);
            return;
        }
        if (!committed) {
            done(readStatus != kIOReturnSuccess ? readStatus : kIOReturnError, false);
            return;
        }
        const auto found = self->controlSnapshot_->state.parameters.find(id);
        if (found == self->controlSnapshot_->state.parameters.end()) {
            done(kIOReturnError, false);
            return;
        }
        done(kIOReturnSuccess, found->second.value);
    });
}

bool SPro24DspProtocol::CommitControls() {
    namespace AM = ASFW::AudioModel;
    namespace C = ::ASFW::Audio::Controls;
    const auto route = tcat_.IO().Route();
    auto next = BuildSPro24DspControls(controls_, route.routeEpoch,
                                       controlSnapshot_ ? controlSnapshot_->sequence : 0);
    if (!AM::validate(next.topology) || !AM::validateState(next.topology, next.state)) {
        ASFW_LOG(DICE, "[SPro24Dsp] control model failed validation");
        return false;
    }
    if (!controlSnapshot_ || controlSnapshot_->topology.revision != next.topology.revision) {
        const bool rebound = controlSnapshot_.has_value();
        if (rebound) next.sequence = C::ControlNotifications::Shared().NextSequence();
        controlSnapshot_ = std::move(next);
        if (rebound) {
            C::ControlNotifications::Shared().Publish({route.guid, controlSnapshot_->topology.revision,
                                                       controlSnapshot_->sequence, 0, 4, 0});
        }
        return true;
    }
    bool metadata = false;
    for (const auto& [id, state] : next.state.routers) {
        const auto old = controlSnapshot_->state.routers.find(id);
        if (old == controlSnapshot_->state.routers.end() || old->second.activeBundles != state.activeBundles) {
            metadata = true;
        }
    }
    std::vector<C::ParameterChanged> events;
    for (const auto& [id, state] : next.state.parameters) {
        const auto old = controlSnapshot_->state.parameters.find(id);
        if (old == controlSnapshot_->state.parameters.end()) {
            metadata = true;
            continue;
        }
        if (old->second.domainOverride != state.domainOverride || old->second.applicable != state.applicable) {
            metadata = true;
        }
        if (old->second.value == state.value) continue;
        C::ParameterChanged event{route.guid, next.topology.revision, 0, id.value, 0, 0};
        std::visit([&](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, bool>) { event.kind = 0; event.bits = v; }
            else if constexpr (std::is_same_v<T, double>) { event.kind = 1; event.bits = std::bit_cast<uint64_t>(v); }
            else if constexpr (std::is_same_v<T, int64_t>) { event.kind = 2; event.bits = static_cast<uint64_t>(v); }
        }, state.value);
        events.push_back(event);
    }
    if (metadata) events = {{route.guid, next.topology.revision, 0, 0, 4, 0}};
    for (auto& event : events) event.sequence = next.sequence = C::ControlNotifications::Shared().NextSequence();
    controlSnapshot_ = std::move(next);
    for (const auto& event : events) C::ControlNotifications::Shared().Publish(event);
    return true;
}

void SPro24DspProtocol::QueueControl(std::function<void(SPro24DspProtocol&)> op, Completion refuse) {
    if (controlWaiters_.size() >= kMaxControlWaiters) {
        refuse(kIOReturnBusy, false);
        return;
    }
    controlWaiters_.push_back([weak = weak_from_this(), op = std::move(op), refuse = std::move(refuse)] {
        const auto locked = weak.lock();
        if (!locked) {
            refuse(kIOReturnAborted, false);
            return;
        }
        op(*std::static_pointer_cast<SPro24DspProtocol>(locked));
    });
}

void SPro24DspProtocol::ReleaseControl() {
    controlBusy_ = false;
    if (controlWaiters_.empty()) return;
    auto next = std::move(controlWaiters_.front());
    controlWaiters_.pop_front();
    next();
}

void SPro24DspProtocol::ResetControls() {
    ++controlsEpoch_;
    controlsReady_ = false;
    controlBusy_ = false;
    controlSnapshot_.reset();
    auto waiters = std::move(controlWaiters_);
    controlWaiters_.clear();
    for (auto& waiter : waiters) waiter(); // each re-enters and is refused as not ready
}

} // namespace ASFW::Audio::DICE::Focusrite
