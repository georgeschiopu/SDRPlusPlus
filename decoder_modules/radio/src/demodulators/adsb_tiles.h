#pragma once
#include <imgui.h>
#include <string>
#include <map>
#include <memory>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace adsbmap {
    static constexpr double MAP_PI = 3.14159265358979323846;
    static constexpr int TILE_SIZE = 256;

    inline double mercX(double lon) { return lon * MAP_PI / 180.0; }
    inline double mercY(double lat) {
        lat = std::max(-85.05112878, std::min(85.05112878, lat));
        return std::log(std::tan(MAP_PI / 4.0 + lat * MAP_PI / 360.0));
    }
    inline double latFromMercY(double y) {
        return (2.0 * std::atan(std::exp(y)) - MAP_PI / 2.0) * 180.0 / MAP_PI;
    }

    struct MercatorView {
        ImVec2 pos;
        ImVec2 size;
        double xMin = 0.0, xMax = 0.0;
        double yMin = 0.0, yMax = 0.0;
        double lonMin = 0.0, lonMax = 0.0;
        double latMin = 0.0, latMax = 0.0;

        void set(double centerLat, double centerLon, double lonSpanDeg, ImVec2 p, ImVec2 sz) {
            pos = p;
            size = sz;
            double xc = mercX(centerLon);
            double xs = lonSpanDeg * MAP_PI / 180.0;
            xMin = xc - xs * 0.5;
            xMax = xc + xs * 0.5;
            double yc = mercY(centerLat);
            double ys = xs * ((double)sz.y / (double)sz.x);
            yMin = yc - ys * 0.5;
            yMax = yc + ys * 0.5;
            lonMin = xMin * 180.0 / MAP_PI;
            lonMax = xMax * 180.0 / MAP_PI;
            latMin = latFromMercY(yMin);
            latMax = latFromMercY(yMax);
        }

        inline ImVec2 project(double lon, double lat) const {
            float x = pos.x + (float)((mercX(lon) - xMin) / (xMax - xMin) * size.x);
            float y = pos.y + (float)((yMax - mercY(lat)) / (yMax - yMin) * size.y);
            return ImVec2(x, y);
        }

        inline float projectX(double lon) const {
            return pos.x + (float)((mercX(lon) - xMin) / (xMax - xMin) * size.x);
        }

        inline float projectY(double lat) const {
            return pos.y + (float)((yMax - mercY(lat)) / (yMax - yMin) * size.y);
        }

        inline double pixelsPerMeterAtLat(double lat) const {
            double worldPx = TILE_SIZE * std::pow(2.0, 13.0);
            double yScale = 1.0 / std::cos(lat * MAP_PI / 180.0);
            double mercPerPx = (yMax - yMin) / size.y;
            return 1.0 / (mercPerPx * 6378137.0 * yScale);
        }
    };

    class TileCache {
    public:
        TileCache(const std::string& cacheDir, const std::string& urlTemplate);
        ~TileCache();

        void setEnabled(bool enabled);
        bool isEnabled() const { return enabled; }

        void update();
        unsigned int getTile(int z, int x, int y);

    private:
        struct Key {
            int z, x, y;
            bool operator<(const Key& o) const {
                if (z != o.z) { return z < o.z; }
                if (x != o.x) { return x < o.x; }
                return y < o.y;
            }
        };
        struct Entry {
            unsigned int tex = 0;
            bool requested = false;
            bool failed = false;
            std::chrono::steady_clock::time_point retryAt;
        };

        void worker();
        std::vector<unsigned char> loadTile(const Key& key);
        std::vector<unsigned char> readCached(const Key& key);
        void writeCached(const Key& key, const std::vector<unsigned char>& data);

        std::string cacheDir;
        std::string urlTemplate;
        bool enabled = true;

        std::map<Key, Entry> entries;
        std::map<Key, std::vector<unsigned char>> completed;
        std::mutex mtx;
        std::condition_variable cv;
        std::queue<Key> queue;
        std::thread workerThread;
        bool stop = false;
    };
}

namespace adsbmap {
    int drawTiles(ImDrawList* dl, const MercatorView& view, TileCache& cache);
    unsigned int planeIconTexture();
}
