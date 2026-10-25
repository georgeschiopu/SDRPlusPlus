#pragma once
#include "../demod.h"
#include "adsb_map_data.h"
#include "adsb_tiles.h"
#include <core.h>
#include <memory>
#include <dsp/processor.h>
#include <gui/style.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <algorithm>

namespace demod {
namespace adsb {

using Clock = std::chrono::steady_clock;
static constexpr double PI = 3.14159265358979323846;

struct Aircraft {
    uint32_t icao = 0;
    std::string callsign;
    int altitudeFt = 0;
    double lat = 0.0;
    double lon = 0.0;
    bool hasPos = false;
    double speedKt = 0.0;
    double heading = 0.0;
    int verticalRateFpm = 0;
    int category = 0;
    uint64_t messages = 0;

    uint32_t cprLatEven = 0;
    uint32_t cprLonEven = 0;
    uint32_t cprLatOdd = 0;
    uint32_t cprLonOdd = 0;
    bool hasEven = false;
    bool hasOdd = false;
    Clock::time_point lastEven;
    Clock::time_point lastOdd;
    Clock::time_point lastSeen;
    Clock::time_point lastPos;
};

struct State {
    std::mutex mtx;
    std::map<uint32_t, Aircraft> aircraft;
    uint64_t messages = 0;
    uint64_t decoded = 0;
    Clock::time_point lastCleanup = Clock::now();
};

inline uint32_t getBits(const uint8_t* data, int start, int len) {
    uint32_t val = 0;
    for (int i = 0; i < len; i++) {
        int bitIdx = start + i;
        int bit = (data[bitIdx >> 3] >> (7 - (bitIdx & 7))) & 1;
        val = (val << 1) | bit;
    }
    return val;
}

inline uint32_t checksum(const uint8_t* msg) {
    uint32_t rem = 0;
    for (int i = 0; i < 112; i++) {
        int bit = (msg[i >> 3] >> (7 - (i & 7))) & 1;
        rem = (rem << 1) | bit;
        if (rem & 0x1000000) { rem ^= 0x1FFF409; }
    }
    return rem & 0xFFFFFF;
}

inline int cprNL(double lat) {
    lat = std::fabs(lat);
    if (lat >= 87.0) { return 1; }
    double c = std::cos(lat * PI / 180.0);
    double a = 1.0 - std::cos(PI / 30.0);
    double x = 1.0 - a / (c * c);
    if (x <= -1.0) { return 1; }
    if (x >= 1.0) { return 59; }
    return (int)std::floor(2.0 * PI / std::acos(x));
}

inline bool decodeCPR(const Aircraft& a, bool useEven, double& lat, double& lon) {
    if (!a.hasEven || !a.hasOdd) { return false; }
    double cprLatEven = a.cprLatEven / 131072.0;
    double cprLonEven = a.cprLonEven / 131072.0;
    double cprLatOdd = a.cprLatOdd / 131072.0;
    double cprLonOdd = a.cprLonOdd / 131072.0;

    double j = std::floor(59.0 * cprLatEven - 60.0 * cprLatOdd + 0.5);
    double latEven = 6.0 * (std::fmod(j, 60.0) + cprLatEven);
    double latOdd = (360.0 / 59.0) * (std::fmod(j, 59.0) + cprLatOdd);
    if (latEven >= 270.0) { latEven -= 360.0; }
    if (latOdd >= 270.0) { latOdd -= 360.0; }
    if (cprNL(latEven) != cprNL(latOdd)) { return false; }

    double useLat = useEven ? latEven : latOdd;
    int ni = std::max(cprNL(useLat) - (useEven ? 0 : 1), 1);
    double m = std::floor(cprLonEven * (cprNL(useLat) - 1) - cprLonOdd * cprNL(useLat) + 0.5);
    double useLon = (360.0 / ni) * (std::fmod(m, ni) + (useEven ? cprLonEven : cprLonOdd));
    if (useLon > 180.0) { useLon -= 360.0; }
    else if (useLon < -180.0) { useLon += 360.0; }

    lat = useLat;
    lon = useLon;
    return true;
}

inline char callsignChar(int c) {
    if (c >= 1 && c <= 26) { return (char)('A' + c - 1); }
    if (c >= 48 && c <= 57) { return (char)('0' + c - 48); }
    return ' ';
}

class Processor : public dsp::Processor<dsp::complex_t, dsp::stereo_t> {
public:
    Processor(State* state) : state(state) {
        pending.reserve(2 * 1024 * 1024);
        mag.reserve(2 * 1024 * 1024);
    }

    void setOutputRate(double rate) { outputRate = rate; }

    int run() override {
        int count = _in->read();
        if (count < 0) { return -1; }

        pending.insert(pending.end(), _in->readBuf, _in->readBuf + count);
        size_t total = pending.size();
        if (total > 256) {
            size_t scanEnd = total - 256;
            mag.resize(total);
            for (size_t i = 0; i < total; i++) {
                float re = pending[i].re;
                float im = pending[i].im;
                mag[i] = std::sqrt(re * re + im * im);
            }

            size_t n = 0;
            while (n < scanEnd) {
                noiseFloor = noiseFloor * 0.9999f + mag[n] * 0.0001f;
                if (isPreamble((int)n)) {
                    uint8_t msg[14];
                    memset(msg, 0, sizeof(msg));
                    int base = (int)n + 16;
                    for (int b = 0; b < 112; b++) {
                        if (mag[base + 2 * b] > mag[base + 2 * b + 1]) {
                            msg[b >> 3] |= (uint8_t)(0x80 >> (b & 7));
                        }
                    }
                    handleMessage(msg);
                    n += 16;
                }
                else {
                    n++;
                }
            }

            pending.erase(pending.begin(), pending.begin() + scanEnd);
        }

        _in->flush();

        outAccum += (double)count * outputRate / 2000000.0;
        int outCount = (int)std::floor(outAccum);
        outAccum -= outCount;
        if (outCount > 0) {
            for (int i = 0; i < outCount; i++) {
                out.writeBuf[i].l = 0.0f;
                out.writeBuf[i].r = 0.0f;
            }
            if (!out.swap(outCount)) { return -1; }
        }

        return count;
    }

private:
    bool isPreamble(int n) {
        float p = mag[n] + mag[n + 2] + mag[n + 7] + mag[n + 9];
        float q = mag[n + 1] + mag[n + 3] + mag[n + 4] + mag[n + 5] + mag[n + 6] +
                  mag[n + 8] + mag[n + 10] + mag[n + 11] + mag[n + 12] +
                  mag[n + 13] + mag[n + 14] + mag[n + 15];
        return (p > 1.08f * q) && (p > 4.0f * std::max(noiseFloor, 1e-5f));
    }

    void handleMessage(const uint8_t* msg) {
        uint32_t df = getBits(msg, 0, 5);
        if (df != 17 && df != 18) { return; }
        if (checksum(msg) != 0) { return; }

        uint32_t icao = getBits(msg, 8, 24);
        if (icao == 0) { return; }

        int tc = getBits(msg, 32, 5);
        Clock::time_point now = Clock::now();

        std::lock_guard<std::mutex> lck(state->mtx);
        state->messages++;

        Aircraft& a = state->aircraft[icao];
        a.icao = icao;
        a.lastSeen = now;
        a.messages++;

        if (tc >= 1 && tc <= 4) {
            std::string cs;
            for (int i = 0; i < 8; i++) {
                cs += callsignChar(getBits(msg, 40 + i * 6, 6));
            }
            while (!cs.empty() && cs.back() == ' ') { cs.pop_back(); }
            if (!cs.empty()) { a.callsign = cs; }
            a.category = getBits(msg, 37, 3);
        }
        else if ((tc >= 9 && tc <= 18) || (tc >= 20 && tc <= 22)) {
            uint32_t altRaw = getBits(msg, 40, 12);
            if (tc >= 9 && tc <= 18) {
                if (altRaw & 0x10) {
                    int alt = (int)(((altRaw & 0x0FE0) >> 1) | (altRaw & 0x000F));
                    a.altitudeFt = alt * 25 - 1000;
                }
            }
            else {
                a.altitudeFt = (int)std::lround(altRaw * 3.28084);
            }

            bool odd = getBits(msg, 53, 1) != 0;
            uint32_t latCpr = getBits(msg, 54, 17);
            uint32_t lonCpr = getBits(msg, 71, 17);
            if (!odd) {
                a.cprLatEven = latCpr;
                a.cprLonEven = lonCpr;
                a.hasEven = true;
                a.lastEven = now;
            }
            else {
                a.cprLatOdd = latCpr;
                a.cprLonOdd = lonCpr;
                a.hasOdd = true;
                a.lastOdd = now;
            }

            bool recent = a.hasEven && a.hasOdd &&
                          std::chrono::duration_cast<std::chrono::seconds>(now - a.lastEven).count() < 10 &&
                          std::chrono::duration_cast<std::chrono::seconds>(now - a.lastOdd).count() < 10;
            if (recent) {
                bool useEven = (a.lastEven >= a.lastOdd);
                double lat, lon;
                if (decodeCPR(a, useEven, lat, lon)) {
                    a.lat = lat;
                    a.lon = lon;
                    a.hasPos = true;
                    a.lastPos = now;
                }
            }
        }
        else if (tc == 19) {
            int subtype = getBits(msg, 37, 3);
            if (subtype == 1 || subtype == 2) {
                int sEw = getBits(msg, 45, 1);
                int vEw = getBits(msg, 46, 10);
                int sNs = getBits(msg, 56, 1);
                int vNs = getBits(msg, 57, 10);
                double ve = (double)(vEw - 1) * (sEw ? -1.0 : 1.0);
                double vn = (double)(vNs - 1) * (sNs ? -1.0 : 1.0);
                if (subtype == 2) { ve *= 4.0; vn *= 4.0; }
                a.speedKt = std::sqrt(ve * ve + vn * vn);
                double hdg = std::atan2(ve, vn) * 180.0 / PI;
                if (hdg < 0.0) { hdg += 360.0; }
                a.heading = hdg;

                int sVr = getBits(msg, 68, 1);
                int vr = getBits(msg, 69, 9);
                if (vr > 0) {
                    a.verticalRateFpm = (sVr ? -1 : 1) * (vr - 1) * 64;
                }
            }
        }

        state->decoded++;

        if (std::chrono::duration_cast<std::chrono::seconds>(now - state->lastCleanup).count() >= 5) {
            state->lastCleanup = now;
            for (auto it = state->aircraft.begin(); it != state->aircraft.end();) {
                if (std::chrono::duration_cast<std::chrono::seconds>(now - it->second.lastSeen).count() > 60) {
                    it = state->aircraft.erase(it);
                }
                else {
                    ++it;
                }
            }
        }
    }

    State* state;
    std::vector<dsp::complex_t> pending;
    std::vector<float> mag;
    double outAccum = 0.0;
    double outputRate = 48000.0;
    float noiseFloor = 0.001f;
};

} // namespace adsb

class ADSB : public Demodulator {
public:
    ADSB() {}
    ADSB(std::string name, ConfigManager* config, dsp::stream<dsp::complex_t>* input, double bandwidth, double audioSR) {
        init(name, config, input, bandwidth, audioSR);
    }

    ~ADSB() { stop(); }

    void init(std::string name, ConfigManager* config, dsp::stream<dsp::complex_t>* input, double bandwidth, double audioSR) {
        this->name = name;
        this->_config = config;
        audioSampleRate = audioSR;
        processor = std::make_unique<adsb::Processor>(&state);
        processor->init(input);
        processor->setOutputRate(audioSampleRate);

        bool changed = false;
        _config->acquire();
        json& c = config->conf[name][getName()];
        if (!c.contains("rxLat")) { c["rxLat"] = 53.4808; changed = true; }
        if (!c.contains("rxLon")) { c["rxLon"] = -2.2426; changed = true; }
        if (!c.contains("mapSpan")) { c["mapSpan"] = 6.0; changed = true; }
        if (!c.contains("autoFit")) { c["autoFit"] = true; changed = true; }
        if (!c.contains("onlineMap")) { c["onlineMap"] = true; changed = true; }
        rxLat = c["rxLat"];
        rxLon = c["rxLon"];
        mapSpan = c["mapSpan"];
        autoFit = c["autoFit"];
        onlineMap = c["onlineMap"];
        _config->release(changed);

        std::string cacheDir = (std::string)core::args["root"] + "/adsb_tile_cache_osm";
        tileCache = std::make_unique<adsbmap::TileCache>(cacheDir, "https://tile.openstreetmap.org/{z}/{x}/{y}.png");
        tileCache->setEnabled(onlineMap);
    }

    void start() { if (processor) { processor->start(); } }
    void stop() { if (processor) { processor->stop(); } }

    void setInput(dsp::stream<dsp::complex_t>* input) {
        if (processor) { processor->setInput(input); }
    }

    void AFSampRateChanged(double newSR) {
        audioSampleRate = newSR;
        if (processor) { processor->setOutputRate(newSR); }
    }

    void showMenu() {
        float menuWidth = ImGui::GetContentRegionAvail().x;

        if (ImGui::Button(("Tune to 1090 MHz##_adsb_tune_" + name).c_str(), ImVec2(menuWidth, 0))) {
            gui::waterfall.setCenterFrequency(1090000000.0);
            gui::waterfall.centerFreqMoved = true;
        }

        double center = gui::waterfall.getCenterFrequency();
        if (std::fabs(center - 1090000000.0) > 500000.0) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.65f, 0.2f, 1.0f));
            ImGui::TextWrapped("Tune to 1090 MHz and set the source sample rate to 2.4 MS/s (or higher).");
            ImGui::PopStyleColor();
        }

        bool changed = false;
        ImGui::LeftLabel("Receiver Lat");
        ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
        if (ImGui::InputDouble(("##adsb_lat_" + name).c_str(), &rxLat, 0.0001, 0.01, "%.5f")) { changed = true; }
        ImGui::LeftLabel("Receiver Lon");
        ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
        if (ImGui::InputDouble(("##adsb_lon_" + name).c_str(), &rxLon, 0.0001, 0.01, "%.5f")) { changed = true; }
        if (ImGui::Checkbox(("Auto-fit##adsb_fit_" + name).c_str(), &autoFit)) { changed = true; }
        if (ImGui::Checkbox(("Online map tiles##adsb_online_" + name).c_str(), &onlineMap)) {
            if (tileCache) { tileCache->setEnabled(onlineMap); }
            changed = true;
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
        if (ImGui::SliderFloat(("##adsb_span_" + name).c_str(), &mapSpan, 1.0f, 40.0f, "%.1f deg")) { changed = true; }

        if (changed) {
            _config->acquire();
            json& c = _config->conf[name][getName()];
            c["rxLat"] = rxLat;
            c["rxLon"] = rxLon;
            c["mapSpan"] = mapSpan;
            c["autoFit"] = autoFit;
            c["onlineMap"] = onlineMap;
            _config->release(true);
        }

        drawMap();
    }

    void setBandwidth(double bandwidth) {}
    double getIFSampleRate() { return 2000000.0; }
    double getAFSampleRate() { return audioSampleRate; }
    double getDefaultBandwidth() { return 2000000.0; }
    double getMinBandwidth() { return 2000000.0; }
    double getMaxBandwidth() { return 2000000.0; }
    bool getBandwidthLocked() { return true; }
    double getDefaultSnapInterval() { return 100000.0; }
    int getVFOReference() { return ImGui::WaterfallVFO::REF_CENTER; }
    bool getDeempAllowed() { return false; }
    bool getPostProcEnabled() { return false; }
    int getDefaultDeemphasisMode() { return DEEMP_MODE_NONE; }
    bool getFMIFNRAllowed() { return false; }
    bool getNBAllowed() { return false; }
    bool getHighPassAllowed() { return false; }
    bool getSquelchAllowed() { return false; }
    dsp::stream<dsp::stereo_t>* getOutput() { return processor ? &processor->out : nullptr; }
    const char* getName() { return "ADS-B"; }

private:
    static double distanceKm(double lat1, double lon1, double lat2, double lon2) {
        double dLat = (lat2 - lat1) * adsb::PI / 180.0;
        double dLon = (lon2 - lon1) * adsb::PI / 180.0;
        double mLat = (lat1 + lat2) * 0.5 * adsb::PI / 180.0;
        double x = dLon * std::cos(mLat);
        return std::sqrt(x * x + dLat * dLat) * 6371.0;
    }

    void drawMap() {
        float mapW = ImGui::GetContentRegionAvail().x;
        float mapH = 230.0f * style::uiScale;
        ImGui::InvisibleButton(("##adsb_map_" + name).c_str(), ImVec2(mapW, mapH));
        ImVec2 p = ImGui::GetItemRectMin();
        ImVec2 sz = ImGui::GetItemRectSize();
        ImDrawList* dl = ImGui::GetWindowDrawList();

        dl->AddRectFilled(p, ImVec2(p.x + sz.x, p.y + sz.y), IM_COL32(8, 16, 28, 255));

        std::vector<adsb::Aircraft> list;
        uint64_t totalMsgs = 0;
        uint64_t decoded = 0;
        {
            std::lock_guard<std::mutex> lck(state.mtx);
            totalMsgs = state.messages;
            decoded = state.decoded;
            adsb::Clock::time_point now = adsb::Clock::now();
            for (auto& [icao, a] : state.aircraft) {
                if (std::chrono::duration_cast<std::chrono::seconds>(now - a.lastSeen).count() <= 60) {
                    list.push_back(a);
                }
            }
        }

        std::sort(list.begin(), list.end(), [&](const adsb::Aircraft& a, const adsb::Aircraft& b) {
            if (a.hasPos != b.hasPos) { return a.hasPos; }
            return distanceKm(rxLat, rxLon, a.lat, a.lon) < distanceKm(rxLat, rxLon, b.lat, b.lon);
        });

        double centerLat = rxLat;
        double centerLon = rxLon;
        double lonSpan = mapSpan;
        if (autoFit) {
            bool found = false;
            double minLat = 1e9, maxLat = -1e9, minLon = 1e9, maxLon = -1e9;
            for (auto& a : list) {
                if (!a.hasPos) { continue; }
                minLat = std::min(minLat, a.lat); maxLat = std::max(maxLat, a.lat);
                minLon = std::min(minLon, a.lon); maxLon = std::max(maxLon, a.lon);
                found = true;
            }
            if (found) {
                centerLon = (minLon + maxLon) * 0.5;
                double y0 = adsbmap::mercY(minLat);
                double y1 = adsbmap::mercY(maxLat);
                centerLat = adsbmap::latFromMercY((y0 + y1) * 0.5);
                double xNeeded = std::max((maxLon - minLon) * 1.35, 0.02) * adsbmap::MAP_PI / 180.0;
                double yNeeded = std::max(y1 - y0, 0.01) * 1.35;
                lonSpan = std::max(xNeeded, yNeeded * ((double)sz.x / (double)sz.y)) * 180.0 / adsbmap::MAP_PI;
            }
        }
        lonSpan = std::clamp(lonSpan, 0.05, 120.0);

        adsbmap::MercatorView view;
        view.set(centerLat, centerLon, lonSpan, p, sz);

        ImGui::PushClipRect(p, ImVec2(p.x + sz.x, p.y + sz.y), true);

        double gridStep = (lonSpan > 20.0) ? 5.0 : (lonSpan > 8.0) ? 2.0 : (lonSpan > 3.0) ? 1.0 : 0.5;
        for (double lon = std::ceil(view.lonMin / gridStep) * gridStep; lon <= view.lonMax; lon += gridStep) {
            float x = view.projectX(lon);
            dl->AddLine(ImVec2(x, p.y), ImVec2(x, p.y + sz.y), IM_COL32(60, 90, 120, 90));
        }
        for (double lat = std::ceil(view.latMin / gridStep) * gridStep; lat <= view.latMax; lat += gridStep) {
            float y = view.projectY(lat);
            dl->AddLine(ImVec2(p.x, y), ImVec2(p.x + sz.x, y), IM_COL32(60, 90, 120, 90));
        }

        for (int i = 0; i < adsbBorderCount; i++) {
            const AdsbMapLine& line = adsbBorder[i];
            for (int j = 1; j < line.count; j++) {
                double lon1 = line.pts[(j - 1) * 2] / 100.0;
                double lat1 = line.pts[(j - 1) * 2 + 1] / 100.0;
                double lon2 = line.pts[j * 2] / 100.0;
                double lat2 = line.pts[j * 2 + 1] / 100.0;
                dl->AddLine(view.project(lon1, lat1), view.project(lon2, lat2), IM_COL32(80, 105, 125, 255));
            }
        }
        for (int i = 0; i < adsbCoastCount; i++) {
            const AdsbMapLine& line = adsbCoast[i];
            for (int j = 1; j < line.count; j++) {
                double lon1 = line.pts[(j - 1) * 2] / 100.0;
                double lat1 = line.pts[(j - 1) * 2 + 1] / 100.0;
                double lon2 = line.pts[j * 2] / 100.0;
                double lat2 = line.pts[j * 2 + 1] / 100.0;
                dl->AddLine(view.project(lon1, lat1), view.project(lon2, lat2), IM_COL32(95, 165, 210, 255));
            }
        }

        int tilesDrawn = 0;
        if (tileCache) {
            tileCache->update();
            tilesDrawn = adsbmap::drawTiles(dl, view, *tileCache);
            if (tilesDrawn > 0) {
                // Dim the bright OSM tiles so they fit the dark SDR++ UI.
                dl->AddRectFilled(p, ImVec2(p.x + sz.x, p.y + sz.y), IM_COL32(8, 14, 24, 150));
            }
        }

        float rxX = view.projectX(rxLon);
        float rxY = view.projectY(rxLat);
        auto drawRing = [&](double km, ImU32 col) {
            double d = km / 6371.0;
            double lat1 = rxLat * adsbmap::MAP_PI / 180.0;
            double lon1 = rxLon * adsbmap::MAP_PI / 180.0;
            ImVec2 prev;
            for (int i = 0; i <= 96; i++) {
                double brg = 2.0 * adsbmap::MAP_PI * i / 96.0;
                double lat2 = std::asin(std::sin(lat1) * std::cos(d) + std::cos(lat1) * std::sin(d) * std::cos(brg));
                double lon2 = lon1 + std::atan2(std::sin(brg) * std::sin(d) * std::cos(lat1), std::cos(d) - std::sin(lat1) * std::sin(lat2));
                ImVec2 pt = view.project(lon2 * 180.0 / adsbmap::MAP_PI, lat2 * 180.0 / adsbmap::MAP_PI);
                if (i > 0) { dl->AddLine(prev, pt, col, 1.0f); }
                prev = pt;
            }
        };
        drawRing(50.0, IM_COL32(120, 170, 100, 100));
        drawRing(100.0, IM_COL32(120, 170, 100, 80));
        drawRing(200.0, IM_COL32(120, 170, 100, 60));

        dl->AddCircleFilled(ImVec2(rxX, rxY), 3.0f * style::uiScale, IM_COL32(255, 220, 0, 255));
        dl->AddText(ImVec2(rxX + 5.0f, rxY - ImGui::GetFontSize()), IM_COL32(255, 220, 0, 255), "QTH");

        int hovered = -1;
        float hoverDist = 12.0f * style::uiScale;
        ImVec2 mouse = ImGui::GetIO().MousePos;
        for (int i = 0; i < (int)list.size(); i++) {
            adsb::Aircraft& a = list[i];
            if (!a.hasPos) { continue; }
            ImVec2 pos = view.project(a.lon, a.lat);
            float x = pos.x;
            float y = pos.y;
            if (x < p.x - 20 || x > p.x + sz.x + 20 || y < p.y - 20 || y > p.y + sz.y + 20) { continue; }

            ImU32 col = IM_COL32(0, 255, 120, 255);
            if (a.altitudeFt >= 30000) { col = IM_COL32(255, 255, 255, 255); }
            else if (a.altitudeFt >= 15000) { col = IM_COL32(0, 220, 255, 255); }
            else if (a.altitudeFt >= 5000) { col = IM_COL32(120, 255, 120, 255); }
            else { col = IM_COL32(255, 180, 60, 255); }

            float ang = (float)(a.heading * adsb::PI / 180.0);
            float dx = std::sin(ang);
            float dy = -std::cos(ang);
            float pxv = -dy;
            float pyv = dx;
            float s = 13.0f * style::uiScale;
            float h = s * 0.5f;
            auto P = [&](float fwd, float side) {
                return ImVec2(x + dx * fwd + pxv * side, y + dy * fwd + pyv * side);
            };
            auto uv0 = ImVec2(0, 0), uv1 = ImVec2(1, 0), uv2 = ImVec2(1, 1), uv3 = ImVec2(0, 1);
            unsigned int icon = adsbmap::planeIconTexture();
            if (icon) {
                ImVec2 p1 = P(h, -h);
                ImVec2 p2 = P(h, h);
                ImVec2 p3 = P(-h, h);
                ImVec2 p4 = P(-h, -h);
                ImVec2 off(0.8f * style::uiScale, 0.8f * style::uiScale);
                ImVec2 s1(p1.x + off.x, p1.y + off.y);
                ImVec2 s2(p2.x + off.x, p2.y + off.y);
                ImVec2 s3(p3.x + off.x, p3.y + off.y);
                ImVec2 s4(p4.x + off.x, p4.y + off.y);
                dl->AddImageQuad((ImTextureID)(uintptr_t)icon, s1, s2, s3, s4,
                                 uv0, uv1, uv2, uv3, IM_COL32(0, 0, 0, 150));
                dl->AddImageQuad((ImTextureID)(uintptr_t)icon, p1, p2, p3, p4,
                                 uv0, uv1, uv2, uv3, col);
            }
            else {
                dl->AddCircleFilled(ImVec2(x, y), 3.0f * style::uiScale, col, 10);
            }

            std::string label = a.callsign.empty() ? std::string() : a.callsign;
            if (!label.empty()) {
                ImFont* font = ImGui::GetFont();
                float fs = ImGui::GetFontSize() * 0.72f;
                ImVec2 tpos(x + 7.0f, y - fs * 0.7f);
                ImVec2 tsz = font->CalcTextSizeA(fs, 10000.0f, 0.0f, label.c_str());
                dl->AddRectFilled(ImVec2(tpos.x - 2.0f, tpos.y - 1.0f),
                                  ImVec2(tpos.x + tsz.x + 2.0f, tpos.y + tsz.y + 1.0f),
                                  IM_COL32(0, 0, 0, 150), 2.0f);
                dl->AddText(font, fs, tpos, col, label.c_str());
            }

            float ddx = mouse.x - x;
            float ddy = mouse.y - y;
            float d = std::sqrt(ddx * ddx + ddy * ddy);
            if (d < hoverDist) { hoverDist = d; hovered = i; }
        }

        dl->AddText(ImVec2(p.x + 6.0f, p.y + 4.0f), IM_COL32(180, 200, 220, 255), "ADS-B / 1090 MHz");
        const char* attr = (tilesDrawn > 0) ? "© OpenStreetMap contributors" : "Offline map: Natural Earth";
        dl->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 0.65f,
                    ImVec2(p.x + 6.0f, p.y + sz.y - ImGui::GetFontSize() * 0.65f - 3.0f),
                    IM_COL32(120, 145, 170, 255), attr);
        ImGui::PopClipRect();

        if (hovered >= 0) {
            adsb::Aircraft& a = list[hovered];
            ImGui::BeginTooltip();
            if (!a.callsign.empty()) { ImGui::Text("Flight: %s", a.callsign.c_str()); }
            ImGui::Text("ICAO: %06X", a.icao);
            ImGui::Text("Altitude: %d ft", a.altitudeFt);
            ImGui::Text("Speed: %.0f kt", a.speedKt);
            ImGui::Text("Heading: %.0f deg", a.heading);
            ImGui::Text("Distance: %.1f km", distanceKm(rxLat, rxLon, a.lat, a.lon));
            ImGui::Text("Position: %.5f, %.5f", a.lat, a.lon);
            ImGui::EndTooltip();
        }

        ImGui::Text("Aircraft: %d   Messages: %llu   Decoded: %llu",
                    (int)list.size(), (unsigned long long)totalMsgs, (unsigned long long)decoded);

        if (ImGui::BeginTable(("##adsb_table_" + name).c_str(), 4,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Flight", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Alt", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("Spd", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("Dist", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableHeadersRow();
            for (auto& a : list) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                if (!a.callsign.empty()) { ImGui::TextUnformatted(a.callsign.c_str()); }
                else { ImGui::Text("%06X", a.icao); }
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%d", a.altitudeFt);
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.0f", a.speedKt);
                ImGui::TableSetColumnIndex(3);
                if (a.hasPos) { ImGui::Text("%.0f", distanceKm(rxLat, rxLon, a.lat, a.lon)); }
                else { ImGui::TextUnformatted("-"); }
            }
            ImGui::EndTable();
        }
    }

    std::string name;
    ConfigManager* _config = nullptr;
    adsb::State state;
    std::unique_ptr<adsb::Processor> processor;
    std::unique_ptr<adsbmap::TileCache> tileCache;
    double audioSampleRate = 48000.0;
    double rxLat = 53.4808;
    double rxLon = -2.2426;
    float mapSpan = 6.0f;
    bool autoFit = true;
    bool onlineMap = true;
};

} // namespace demod
