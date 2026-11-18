// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 ASFireWire Project
//
// SPro24DspControlModel.hpp - Device Controls model for the Saffire Pro 24 DSP.
//
// Publishes what Focusrite MixControl's main panel sets, from state read back
// from the device: the input levels, the output group (monitor section), and
// the source of each physical output in the TCAT router.
//
// Wire facts are cross-checked with snd-firewire-ctl-services
// (protocols/dice/src/focusrite/spro24dsp.rs: router INPUTS/OUTPUTS/FIXED and
// the 88.2/96 kHz notes; focusrite.rs: output group and input params) and with
// router tables read back from a Pro 24 DSP after MixControl stored them
// (2026-11-18). Pure and allocation-only: no I/O, host-testable.

#pragma once

#include "SaffireproCommon.hpp"
#include "../../../Controls/IControlProvider.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ASFW::Audio::DICE::Focusrite {

namespace SPro24DspControl {
inline constexpr uint32_t InputInstrument = 0x53500010; // + ch (0..1): inputs 1-2
inline constexpr uint32_t InputHighGain   = 0x53500020; // + ch (0..1): inputs 3-4
inline constexpr uint32_t MonitorMute     = 0x53500100;
inline constexpr uint32_t MonitorDim      = 0x53500101;
inline constexpr uint32_t OutputVolume    = 0x53500110; // + output (0..5)
inline constexpr uint32_t OutputMute      = 0x53500120; // + output (0..5)
inline constexpr uint32_t SaveToDevice    = 0x53500300;
inline constexpr uint32_t OutputRouter    = 0x53501000; // + destination index (router node id)
} // namespace SPro24DspControl

namespace SPro24DspRouter {

// TCAT router entry: destination in bits 7:0, source in bits 15:8, peak meter
// in bits 31:16. Each endpoint byte is (block << 4) | channel.
[[nodiscard]] constexpr uint8_t Endpoint(uint8_t block, uint8_t channel) noexcept {
    return static_cast<uint8_t>((block << 4) | (channel & 0x0F));
}
inline constexpr uint8_t kDstAes = 0x0;
inline constexpr uint8_t kDstIns0 = 0x4;
inline constexpr uint8_t kSrcAes = 0x0;
inline constexpr uint8_t kSrcAdat = 0x1;
inline constexpr uint8_t kSrcMixer = 0x2;
inline constexpr uint8_t kSrcIns0 = 0x4;
inline constexpr uint8_t kSrcAvs0 = 0xB;
inline constexpr uint8_t kSrcMute = 0xF;
inline constexpr uint8_t kMuteSource = Endpoint(kSrcMute, 0);

[[nodiscard]] constexpr uint8_t EntryDestination(uint32_t entry) noexcept { return entry & 0xFFU; }
[[nodiscard]] constexpr uint8_t EntrySource(uint32_t entry) noexcept { return (entry >> 8) & 0xFFU; }
[[nodiscard]] constexpr uint32_t MakeEntry(uint8_t destination, uint8_t source) noexcept {
    return uint32_t{destination} | (uint32_t{source} << 8);
}

/// The router tables the device keeps, one per rate mode (current-config section).
enum class RateMode : uint8_t { Low = 0, Middle = 1 };
inline constexpr size_t kRateModeCount = 2;

[[nodiscard]] constexpr RateMode RateModeFor(uint32_t sampleRateHz) noexcept {
    return sampleRateHz >= 88200 ? RateMode::Middle : RateMode::Low;
}

/// A physical output. At 88.2/96 kHz the channel strip and reverb move to
/// Ins0:4-7 (spro24dsp.rs), so analog outputs 5/6 move to Ins0:8/9 (device
/// table stored by MixControl).
struct Destination {
    const char* name;
    std::array<uint8_t, kRateModeCount> endpoint;
};

inline constexpr std::array<Destination, 8> kDestinations{{
    {"Monitor Out 1", {Endpoint(kDstIns0, 0), Endpoint(kDstIns0, 0)}},
    {"Monitor Out 2", {Endpoint(kDstIns0, 1), Endpoint(kDstIns0, 1)}},
    {"Line Out 3 (Headphones 1 L)", {Endpoint(kDstIns0, 2), Endpoint(kDstIns0, 2)}},
    {"Line Out 4 (Headphones 1 R)", {Endpoint(kDstIns0, 3), Endpoint(kDstIns0, 3)}},
    {"Line Out 5 (Headphones 2 L)", {Endpoint(kDstIns0, 4), Endpoint(kDstIns0, 8)}},
    {"Line Out 6 (Headphones 2 R)", {Endpoint(kDstIns0, 5), Endpoint(kDstIns0, 9)}},
    {"S/PDIF Out 1", {Endpoint(kDstAes, 6), Endpoint(kDstAes, 6)}},
    {"S/PDIF Out 2", {Endpoint(kDstAes, 7), Endpoint(kDstAes, 7)}},
}};

/// A router source; nullopt in a rate mode where the device does not carry it.
struct Source {
    std::string name;
    std::array<std::optional<uint8_t>, kRateModeCount> endpoint;
};

/// Every source MixControl offers for an output, "Off" first.
inline std::vector<Source> Sources() {
    std::vector<Source> s;
    auto both = [&](std::string name, uint8_t endpoint) {
        s.push_back({std::move(name), {endpoint, endpoint}});
    };
    both("Off", kMuteSource);
    for (uint8_t ch = 0; ch < 8; ++ch) both("DAW " + std::to_string(ch + 1), Endpoint(kSrcAvs0, ch));
    // Inputs 1/2 are the mic/instrument combos (Ins0:2/3); 3/4 the line inputs (Ins0:0/1).
    both("Analog In 1", Endpoint(kSrcIns0, 2));
    both("Analog In 2", Endpoint(kSrcIns0, 3));
    both("Analog In 3", Endpoint(kSrcIns0, 0));
    both("Analog In 4", Endpoint(kSrcIns0, 1));
    s.push_back({"FX 1 (channel strip)", {Endpoint(kSrcIns0, 8), Endpoint(kSrcIns0, 4)}});
    s.push_back({"FX 2 (channel strip)", {Endpoint(kSrcIns0, 9), Endpoint(kSrcIns0, 5)}});
    s.push_back({"Reverb L", {Endpoint(kSrcIns0, 14), Endpoint(kSrcIns0, 6)}});
    s.push_back({"Reverb R", {Endpoint(kSrcIns0, 15), Endpoint(kSrcIns0, 7)}});
    both("S/PDIF In 1", Endpoint(kSrcAes, 6));
    both("S/PDIF In 2", Endpoint(kSrcAes, 7));
    both("Optical S/PDIF In 1", Endpoint(kSrcAes, 4));
    both("Optical S/PDIF In 2", Endpoint(kSrcAes, 5));
    // The optical port carries 8 ADAT channels at 44.1/48 kHz, 4 at 88.2/96 kHz.
    for (uint8_t ch = 0; ch < 8; ++ch) {
        s.push_back({"ADAT " + std::to_string(ch + 1),
                     {Endpoint(kSrcAdat, ch),
                      ch < 4 ? std::optional<uint8_t>{Endpoint(kSrcAdat, ch)} : std::nullopt}});
    }
    for (uint8_t ch = 0; ch < 16; ++ch) both("Mixer Out " + std::to_string(ch + 1), Endpoint(kSrcMixer, ch));
    return s;
}

/// Source index driving `destination` in `table`, or nullopt when the device
/// uses a source this catalog does not name. An absent entry is "Off".
[[nodiscard]] inline std::optional<size_t> SourceIndexFor(const std::vector<uint32_t>& table,
                                                          const std::vector<Source>& sources,
                                                          RateMode mode,
                                                          uint8_t destination) {
    const auto entry = std::ranges::find(table, destination, EntryDestination);
    const uint8_t source = entry == table.end() ? kMuteSource : EntrySource(*entry);
    const auto m = static_cast<size_t>(mode);
    const auto found = std::ranges::find_if(sources, [&](const Source& s) { return s.endpoint[m] == source; });
    if (found == sources.end()) return std::nullopt;
    return static_cast<size_t>(found - sources.begin());
}

/// The table with `destination` driven by `source`. Every other entry and the
/// order are kept: the first four feed the front-panel meters (spro24dsp.rs
/// FIXED), and routes this model does not name (mixer inputs, capture) stay.
/// An absent destination is appended unless it would only be muted.
/// Returns nullopt when an append would exceed `capacity`.
[[nodiscard]] inline std::optional<std::vector<uint32_t>> WithRoute(std::vector<uint32_t> table,
                                                                    uint8_t destination,
                                                                    uint8_t source,
                                                                    size_t capacity) {
    bool found = false;
    for (auto& entry : table) {
        if (EntryDestination(entry) != destination) continue;
        entry = MakeEntry(destination, source);
        found = true;
    }
    if (!found && source != kMuteSource) {
        if (table.size() >= capacity) return std::nullopt;
        table.push_back(MakeEntry(destination, source));
    }
    return table;
}

} // namespace SPro24DspRouter

/// What the provider has read back from the device.
struct SPro24DspControlsState {
    InputParams inputs{};
    OutputGroupState outputs{};
    std::array<std::vector<uint32_t>, SPro24DspRouter::kRateModeCount> routers{};
    SPro24DspRouter::RateMode liveMode{SPro24DspRouter::RateMode::Low};
};

/// Port id of source `source` on output router `destination`: each router owns
/// its own input ports, numbered from a block per router.
[[nodiscard]] constexpr uint32_t SPro24DspRouterInputPort(size_t destination, size_t source) noexcept {
    return 1000U + static_cast<uint32_t>(destination) * 100U + static_cast<uint32_t>(source);
}
[[nodiscard]] constexpr uint32_t SPro24DspRouterOutputPort(size_t destination) noexcept {
    return 900U + static_cast<uint32_t>(destination);
}

inline Controls::ControlSnapshot BuildSPro24DspControls(const SPro24DspControlsState& native,
                                                        uint64_t revision,
                                                        uint64_t sequence) {
    namespace AM = ASFW::AudioModel;
    namespace R = SPro24DspRouter;
    Controls::ControlSnapshot result;
    result.family = "focusrite-saffire-pro24dsp";
    result.topology.revision = result.state.topologyRevision = revision;
    result.sequence = sequence;
    auto& t = result.topology;
    auto& s = result.state;

    auto node = [&](uint32_t id, std::string name, AM::NodeBody body) {
        t.nodes.push_back({AM::NodeId{id}, std::move(name), std::move(body)});
    };
    auto port = [&](uint32_t id, uint32_t owner, AM::PortDirection direction, std::string name,
                    AM::SignalIdentity signal = {}) {
        t.ports.push_back({AM::PortId{id}, AM::NodeId{owner}, direction, 1, std::move(name), signal});
        return AM::PortId{id};
    };
    auto param = [&](uint32_t id, AM::ParameterTarget target, AM::ParameterSemantic semantic,
                     AM::ParameterDomain domain, std::string name, AM::ParameterValue value) {
        t.parameters.push_back({AM::ParameterId{id}, target, semantic, std::move(domain), std::move(name)});
        s.parameters.emplace(AM::ParameterId{id}, AM::ParameterState{std::move(value)});
    };

    // --- Inputs: no fixed links to the host streams, so nothing here becomes a
    // Core Audio control (CoreAudioProjection places only linked parameters).
    const AM::EnumDomain instrumentLevels{{{0, "Line"}, {1, "Instrument"}}};
    const AM::EnumDomain lineGains{{{0, "Lo Gain (+16 dBu)"}, {1, "Hi Gain (-10 dBV)"}}};
    for (uint32_t ch = 0; ch < 4; ++ch) {
        const uint32_t nodeId = 10 + ch;
        node(nodeId, "Analog Input " + std::to_string(ch + 1), AM::EndpointNode{AM::EndpointKind::Physical});
        if (ch < 2) {
            const auto jack = port(20 + ch, nodeId, AM::PortDirection::Output, "Combo",
                                   {AM::SignalKind::AnalogInstrument, ch + 1});
            param(SPro24DspControl::InputInstrument + ch, jack, AM::ParameterSemantic::NominalLevel,
                  instrumentLevels, "Input " + std::to_string(ch + 1) + " Level",
                  int64_t(native.inputs.micLevels[ch] == MicInputLevel::Instrument ? 1 : 0));
        } else {
            const auto line = port(20 + ch, nodeId, AM::PortDirection::Output, "Line",
                                   {AM::SignalKind::AnalogLine, ch + 1});
            param(SPro24DspControl::InputHighGain + (ch - 2), line, AM::ParameterSemantic::NominalLevel,
                  lineGains, "Input " + std::to_string(ch + 1) + " Gain",
                  int64_t(native.inputs.lineLevels[ch - 2] == LineInputLevel::High ? 1 : 0));
        }
    }

    // --- Output group (MixControl's monitor section): one channel per analog
    // output, after its router selector and before the jack. Master mute/dim act
    // on the whole node; volume and mute on each channel's port, so Device
    // Controls titles them by output. Nothing links to the host streams.
    AM::ProcessorNode monitor;
    monitor.channelPreserving = true;
    node(51, "Analog Outputs", AM::EndpointNode{AM::EndpointKind::Physical});
    for (uint32_t out = 0; out < 6; ++out) {
        const std::string name = SPro24DspRouter::kDestinations[out].name;
        monitor.inputs.push_back(port(30 + out, 50, AM::PortDirection::Input, name));
        monitor.outputs.push_back(port(40 + out, 50, AM::PortDirection::Output, name));
        const auto jack = port(60 + out, 51, AM::PortDirection::Input, name, {AM::SignalKind::AnalogLine, out + 1});
        t.fixedLinks.push_back({AM::PortId{SPro24DspRouterOutputPort(out)}, AM::PortId{30 + out}});
        t.fixedLinks.push_back({AM::PortId{40 + out}, jack});
    }
    node(52, "S/PDIF Outputs", AM::EndpointNode{AM::EndpointKind::Physical});
    for (uint32_t ch = 0; ch < 2; ++ch) {
        const auto jack = port(66 + ch, 52, AM::PortDirection::Input,
                               SPro24DspRouter::kDestinations[6 + ch].name, {AM::SignalKind::SpdifCoaxial, ch + 1});
        t.fixedLinks.push_back({AM::PortId{SPro24DspRouterOutputPort(6 + ch)}, jack});
    }
    const auto monitorOutputs = monitor.outputs;
    node(50, "Monitor", std::move(monitor));
    param(SPro24DspControl::MonitorMute, AM::NodeId{50}, AM::ParameterSemantic::Mute, AM::BooleanDomain{},
          "Mute", native.outputs.muteEnabled);
    param(SPro24DspControl::MonitorDim, AM::NodeId{50}, AM::ParameterSemantic::Dim, AM::BooleanDomain{},
          "Dim", native.outputs.dimEnabled);
    for (uint32_t out = 0; out < 6; ++out) {
        param(SPro24DspControl::OutputVolume + out, monitorOutputs[out], AM::ParameterSemantic::Level,
              AM::ScalarDomain{double(OutputGroupState::kVolMin), double(OutputGroupState::kVolMax), 1,
                               AM::ScalarUnit::Generic},
              "Output " + std::to_string(out + 1) + " Volume", double(native.outputs.volumes[out]));
        param(SPro24DspControl::OutputMute + out, monitorOutputs[out], AM::ParameterSemantic::Mute,
              AM::BooleanDomain{}, "Output " + std::to_string(out + 1) + " Mute", native.outputs.volMutes[out]);
    }

    // --- Device actions.
    node(60, "Device", AM::EndpointNode{AM::EndpointKind::Physical});
    param(SPro24DspControl::SaveToDevice, AM::NodeId{60}, AM::ParameterSemantic::Unknown, AM::ActionDomain{},
          "Save Settings to Device", AM::ActionTrigger{});

    // --- One source selector per physical output, from the live rate mode's table.
    const auto sources = R::Sources();
    const auto mode = static_cast<size_t>(native.liveMode);
    for (size_t dst = 0; dst < R::kDestinations.size(); ++dst) {
        const uint32_t routerId = SPro24DspControl::OutputRouter + static_cast<uint32_t>(dst);
        AM::RouterNode router;
        const auto output = port(SPro24DspRouterOutputPort(dst), routerId, AM::PortDirection::Output,
                                 R::kDestinations[dst].name);
        router.outputs.push_back(output);
        for (size_t src = 0; src < sources.size(); ++src) {
            const auto input = port(SPro24DspRouterInputPort(dst, src), routerId, AM::PortDirection::Input,
                                    sources[src].name);
            router.inputs.push_back(input);
            router.legalBundles.push_back({AM::RouteBundleId{static_cast<uint32_t>(src + 1)}, {{input, output}}});
        }
        router.constraints = {.maxActiveBundles = 1, .maxSourcesPerOutput = 1};
        router.writable = true;
        AM::RouterState state;
        if (const auto active = R::SourceIndexFor(native.routers[mode], sources, native.liveMode,
                                                  R::kDestinations[dst].endpoint[mode])) {
            state.activeBundles.push_back(AM::RouteBundleId{static_cast<uint32_t>(*active + 1)});
        }
        s.routers.emplace(AM::NodeId{routerId}, std::move(state));
        node(routerId, R::kDestinations[dst].name, std::move(router));
    }
    return result;
}

} // namespace ASFW::Audio::DICE::Focusrite
