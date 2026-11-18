// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 ASFireWire Project
//
// Device Controls model for the Focusrite Saffire Pro 24 DSP. The router
// tables are the ones read back from a Pro 24 DSP on 2026-11-18, before and
// after Focusrite MixControl stored "DAW n -> output n" (ASFW DICE Report).

#include "Audio/Protocols/DICE/Focusrite/SPro24DspControlModel.hpp"
#include "Audio/Controls/CoreAudioProjection.hpp"
#include "Audio/Model/Topology/Validate.hpp"
#include "Common/WireFormat.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <string>
#include <vector>

namespace {

using namespace ASFW::Audio::DICE::Focusrite;
namespace R = SPro24DspRouter;
namespace C = ASFW::Audio::Controls;
namespace AM = ASFW::AudioModel;

constexpr uint32_t E(uint8_t dst, uint8_t src) { return R::MakeEntry(dst, src); }

// Low rate (32-48 kHz) table after MixControl, 48 entries, in device order.
std::vector<uint32_t> LowAfterMixControl() {
    std::vector<uint32_t> t{
        E(0x48, 0x42), E(0x49, 0x43), E(0xB2, 0x40), E(0xB3, 0x41), // metering (FIXED)
        E(0xB4, 0x06), E(0xB5, 0x07), E(0xB6, 0x04), E(0xB7, 0x05),
    };
    for (uint8_t ch = 8; ch <= 13; ++ch) t.push_back(E(uint8_t(0xB0 | ch), 0xF0));
    for (uint8_t ch = 0; ch < 6; ++ch) t.push_back(E(uint8_t(0x40 | ch), uint8_t(0xB0 | ch))); // Ins0:n <- Avs0:n
    t.insert(t.end(), {E(0x06, 0xF0), E(0x07, 0xF0), E(0xBE, 0xF0), E(0xBF, 0xF0),
                       E(0x20, 0x42), E(0x21, 0x43), E(0x22, 0x40), E(0x23, 0x41)});
    for (uint8_t ch = 4; ch <= 13; ++ch) t.push_back(E(uint8_t(0x20 | ch), 0xF0));
    t.insert(t.end(), {E(0x2E, 0xB0), E(0x2F, 0xB1), E(0x30, 0x4E), E(0x31, 0x4F),
                       E(0xB0, 0x48), E(0xB1, 0x49), E(0x4E, 0x28), E(0x4F, 0x29),
                       E(0xF0, 0x20), E(0xF0, 0x21)});
    return t;
}

// The same table before MixControl: output 1 fed by mixer output 1, output 3 muted.
std::vector<uint32_t> LowBeforeMixControl() {
    auto t = LowAfterMixControl();
    for (auto& e : t) {
        if (R::EntryDestination(e) == 0x40) e = E(0x40, 0x20);
        if (R::EntryDestination(e) == 0x42) e = E(0x42, 0xF0);
    }
    return t;
}

// Middle rate (88.2-96 kHz) entries that differ in channel numbers.
std::vector<uint32_t> MiddleAfterMixControl() {
    return {E(0x44, 0x42), E(0x45, 0x43), E(0xB2, 0x40), E(0xB3, 0x41),
            E(0x40, 0xB0), E(0x41, 0xB1), E(0x42, 0xB2), E(0x43, 0xB3),
            E(0x48, 0xB4), E(0x49, 0xB5), E(0x46, 0x28), E(0x47, 0x29)};
}

SPro24DspControlsState MakeState(std::vector<uint32_t> low, std::vector<uint32_t> middle,
                                 R::RateMode live = R::RateMode::Low) {
    SPro24DspControlsState s;
    s.routers[0] = std::move(low);
    s.routers[1] = std::move(middle);
    s.liveMode = live;
    return s;
}

std::string ActiveSourceName(const C::ControlSnapshot& model, size_t destination) {
    const AM::NodeId router{SPro24DspControl::OutputRouter + static_cast<uint32_t>(destination)};
    const auto state = model.state.routers.find(router);
    if (state == model.state.routers.end() || state->second.activeBundles.empty()) return "<none>";
    const auto index = state->second.activeBundles.front().value - 1;
    return R::Sources().at(index).name;
}

const AM::ParameterState& Param(const C::ControlSnapshot& model, uint32_t id) {
    return model.state.parameters.at(AM::ParameterId{id});
}

TEST(SPro24DspControlModelTests, TopologyAndStateValidate) {
    const auto model = BuildSPro24DspControls(MakeState(LowAfterMixControl(), MiddleAfterMixControl()), 7, 1);
    ASSERT_TRUE(AM::validate(model.topology));
    ASSERT_TRUE(AM::validateState(model.topology, model.state));
    const auto routers = std::ranges::count_if(model.topology.nodes, [](const AM::Node& n) {
        const auto* r = std::get_if<AM::RouterNode>(&n.body);
        return r != nullptr && r->writable && r->outputs.size() == 1;
    });
    EXPECT_EQ(routers, 8);
}

TEST(SPro24DspControlModelTests, ActiveSourcesFollowTheStoredLowRateTable) {
    const auto model = BuildSPro24DspControls(MakeState(LowAfterMixControl(), MiddleAfterMixControl()), 1, 1);
    EXPECT_EQ(ActiveSourceName(model, 0), "DAW 1");
    EXPECT_EQ(ActiveSourceName(model, 1), "DAW 2");
    EXPECT_EQ(ActiveSourceName(model, 2), "DAW 3");
    EXPECT_EQ(ActiveSourceName(model, 5), "DAW 6");
    EXPECT_EQ(ActiveSourceName(model, 6), "Off"); // S/PDIF out <- Mute
}

TEST(SPro24DspControlModelTests, TableBeforeMixControlShowsTheMixerAndTheMutedHeadphone) {
    const auto model = BuildSPro24DspControls(MakeState(LowBeforeMixControl(), {}), 1, 1);
    EXPECT_EQ(ActiveSourceName(model, 0), "Mixer Out 1");
    EXPECT_EQ(ActiveSourceName(model, 1), "DAW 2");
    EXPECT_EQ(ActiveSourceName(model, 2), "Off");
    EXPECT_EQ(ActiveSourceName(model, 3), "DAW 4");
}

TEST(SPro24DspControlModelTests, MiddleRateReadsOutputsFiveAndSixFromTheirOwnChannels) {
    const auto model = BuildSPro24DspControls(
        MakeState(LowAfterMixControl(), MiddleAfterMixControl(), R::RateMode::Middle), 1, 1);
    EXPECT_EQ(ActiveSourceName(model, 4), "DAW 5"); // Ins0:8 <- Avs0:4 at 88.2/96 kHz
    EXPECT_EQ(ActiveSourceName(model, 5), "DAW 6");
    EXPECT_EQ(R::kDestinations[4].endpoint[1], 0x48);
    EXPECT_EQ(R::RateModeFor(96000), R::RateMode::Middle);
    EXPECT_EQ(R::RateModeFor(44100), R::RateMode::Low);
}

TEST(SPro24DspControlModelTests, UnknownSourceLeavesNoActiveBundle) {
    auto low = LowAfterMixControl();
    for (auto& e : low) if (R::EntryDestination(e) == 0x41) e = E(0x41, 0x5A); // Ins1:10, not offered
    const auto model = BuildSPro24DspControls(MakeState(std::move(low), {}), 1, 1);
    EXPECT_EQ(ActiveSourceName(model, 1), "<none>");
    EXPECT_TRUE(AM::validateState(model.topology, model.state));
}

TEST(SPro24DspControlModelTests, SourceCatalogMatchesTheReference) {
    const auto sources = R::Sources();
    ASSERT_EQ(sources.size(), 45u);
    EXPECT_EQ(sources.front().name, "Off");
    const auto adat5 = std::ranges::find(sources, std::string("ADAT 5"), &R::Source::name);
    ASSERT_NE(adat5, sources.end());
    EXPECT_TRUE(adat5->endpoint[0].has_value());
    EXPECT_FALSE(adat5->endpoint[1].has_value()); // 4 ADAT channels at 88.2/96 kHz
    const auto in1 = std::ranges::find(sources, std::string("Analog In 1"), &R::Source::name);
    EXPECT_EQ(in1->endpoint[0], uint8_t{0x42}); // Mic input 1 is Ins0:2 (spro24dsp.rs)
}

TEST(SPro24DspControlModelTests, WithRouteReplacesInPlaceAndKeepsTheMeteringEntries) {
    const auto before = LowBeforeMixControl();
    const auto edited = R::WithRoute(before, 0x42, 0xB2, 128); // Line Out 3 <- DAW 3
    ASSERT_TRUE(edited.has_value());
    ASSERT_EQ(edited->size(), before.size());
    EXPECT_TRUE(std::equal(before.begin(), before.begin() + 4, edited->begin()));
    size_t changed = 0;
    for (size_t i = 0; i < before.size(); ++i) changed += before[i] != (*edited)[i];
    EXPECT_EQ(changed, 1u);
    const auto model = BuildSPro24DspControls(MakeState(*edited, {}), 1, 1);
    EXPECT_EQ(ActiveSourceName(model, 2), "DAW 3");
}

TEST(SPro24DspControlModelTests, WithRouteAppendsAMissingOutputWithinCapacity) {
    std::vector<uint32_t> table{E(0x48, 0x42), E(0x49, 0x43), E(0xB2, 0x40), E(0xB3, 0x41)};
    const auto appended = R::WithRoute(table, 0x07, 0xB7, 128); // S/PDIF Out 2 <- DAW 8
    ASSERT_TRUE(appended.has_value());
    ASSERT_EQ(appended->size(), 5u);
    EXPECT_EQ(appended->back(), E(0x07, 0xB7));
    EXPECT_FALSE(R::WithRoute(table, 0x07, 0xB7, table.size()).has_value());
    const auto muted = R::WithRoute(table, 0x07, R::kMuteSource, table.size());
    ASSERT_TRUE(muted.has_value());
    EXPECT_EQ(*muted, table); // muting an output nothing drives changes nothing
}

TEST(SPro24DspControlModelTests, ParametersReflectTheReadBackState) {
    auto state = MakeState(LowAfterMixControl(), {});
    state.inputs.micLevels[0] = MicInputLevel::Instrument;
    state.inputs.lineLevels[1] = LineInputLevel::High;
    state.outputs.dimEnabled = true;
    state.outputs.volumes[3] = 100;
    state.outputs.volMutes[5] = true;
    const auto model = BuildSPro24DspControls(state, 1, 1);
    EXPECT_EQ(Param(model, SPro24DspControl::InputInstrument).value, AM::ParameterValue{int64_t{1}});
    EXPECT_EQ(Param(model, SPro24DspControl::InputInstrument + 1).value, AM::ParameterValue{int64_t{0}});
    EXPECT_EQ(Param(model, SPro24DspControl::InputHighGain + 1).value, AM::ParameterValue{int64_t{1}});
    EXPECT_EQ(Param(model, SPro24DspControl::MonitorDim).value, AM::ParameterValue{true});
    EXPECT_EQ(Param(model, SPro24DspControl::MonitorMute).value, AM::ParameterValue{false});
    EXPECT_EQ(Param(model, SPro24DspControl::OutputVolume + 3).value, AM::ParameterValue{100.0});
    EXPECT_EQ(Param(model, SPro24DspControl::OutputMute + 5).value, AM::ParameterValue{true});
}

// Volume and mute act on their output's channel (Device Controls titles them by
// it); the selector of the same output feeds that channel.
TEST(SPro24DspControlModelTests, PerOutputControlsSitOnTheirOutputChannel) {
    const auto model = BuildSPro24DspControls(MakeState(LowAfterMixControl(), {}), 1, 1);
    const auto& params = model.topology.parameters;
    const auto volume = std::ranges::find(params, AM::ParameterId{SPro24DspControl::OutputVolume + 2}, &AM::Parameter::id);
    const auto mute = std::ranges::find(params, AM::ParameterId{SPro24DspControl::OutputMute + 2}, &AM::Parameter::id);
    ASSERT_NE(volume, params.end());
    ASSERT_NE(mute, params.end());
    ASSERT_TRUE(std::holds_alternative<AM::PortId>(volume->target));
    EXPECT_EQ(volume->target, mute->target);
    const auto channel = std::get<AM::PortId>(volume->target);
    const auto port = std::ranges::find(model.topology.ports, channel, &AM::Port::id);
    ASSERT_NE(port, model.topology.ports.end());
    EXPECT_EQ(port->name, "Line Out 3 (Headphones 1 L)");
    const AM::PortId selector{SPro24DspRouterOutputPort(2)};
    EXPECT_TRUE(std::ranges::any_of(model.topology.fixedLinks, [&](const AM::FixedLink& l) { return l.source == selector; }));
    const auto master = std::ranges::find(params, AM::ParameterId{SPro24DspControl::MonitorMute}, &AM::Parameter::id);
    EXPECT_TRUE(std::holds_alternative<AM::NodeId>(master->target));
}

TEST(SPro24DspControlModelTests, NothingBecomesACoreAudioControl) {
    const auto model = BuildSPro24DspControls(MakeState(LowAfterMixControl(), MiddleAfterMixControl()), 1, 1);
    EXPECT_TRUE(C::ProjectCoreAudioControls(model.topology, model.state).empty());
    EXPECT_TRUE(C::ProjectInputTypes(model.topology, model.state).empty());
}

// The output-group write sends only changed quadlets (focusrite.rs): one volume
// changes one packed pair quadlet, never the knob value at 0x48.
TEST(SPro24DspControlModelTests, OneVolumeChangesOnlyItsPackedQuadlet) {
    OutputGroupState before;
    before.volumes = {127, 127, 90, 90, 80, 80};
    before.hwKnobValue = 42;
    auto after = before;
    after.volumes[3] = 60;
    std::array<uint8_t, kOutputGroupStateSize> a{};
    std::array<uint8_t, kOutputGroupStateSize> b{};
    before.Serialize(a.data());
    after.Serialize(b.data());
    std::vector<size_t> changed;
    for (size_t pos = 0; pos < kOutputGroupStateSize; pos += 4) {
        if (!std::equal(a.begin() + pos, a.begin() + pos + 4, b.begin() + pos)) changed.push_back(pos);
    }
    EXPECT_EQ(changed, (std::vector<size_t>{0x0C}));
}

} // namespace
