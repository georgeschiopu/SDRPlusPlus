#include "adsb_tiles.h"
#include "adsb_plane_icon.h"
#include <utils/opengl_include_code.h>
#include <utils/flog.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#ifdef ADSB_HAVE_CURL
#include <curl/curl.h>
#endif

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_STATIC
#include <imgui/stb_image.h>

namespace adsbmap {

TileCache::TileCache(const std::string& cacheDir, const std::string& urlTemplate)
    : cacheDir(cacheDir), urlTemplate(urlTemplate) {
    std::error_code ec;
    std::filesystem::create_directories(cacheDir, ec);
#ifdef ADSB_HAVE_CURL
    static std::once_flag curlInitFlag;
    std::call_once(curlInitFlag, []() { curl_global_init(CURL_GLOBAL_DEFAULT); });
#endif
    workerThread = std::thread(&TileCache::worker, this);
}

TileCache::~TileCache() {
    {
        std::lock_guard<std::mutex> lck(mtx);
        stop = true;
    }
    cv.notify_all();
    if (workerThread.joinable()) { workerThread.join(); }

    // Do not delete GL textures here: the demodulator can be destroyed from a
    // non-render thread (e.g. the rigctl server), where there is no current GL
    // context. Textures are reclaimed when the GL context is destroyed.
}

void TileCache::setEnabled(bool enabled) {
    std::lock_guard<std::mutex> lck(mtx);
    this->enabled = enabled;
    if (enabled) {
        for (auto& [key, entry] : entries) {
            if (!entry.tex) {
                entry.failed = false;
                entry.requested = false;
            }
        }
    }
}

unsigned int TileCache::getTile(int z, int x, int y) {
    Key key{z, x, y};
    std::lock_guard<std::mutex> lck(mtx);
    auto it = entries.find(key);
    if (it != entries.end()) {
        if (it->second.tex) { return it->second.tex; }
        if (!enabled || it->second.requested) { return 0; }
        if (it->second.failed && std::chrono::steady_clock::now() < it->second.retryAt) { return 0; }
        it->second.requested = true;
        it->second.failed = false;
        queue.push(key);
        cv.notify_one();
        return 0;
    }

    Entry& entry = entries[key];
    if (!enabled) { return 0; }
    entry.requested = true;
    queue.push(key);
    cv.notify_one();
    return 0;
}

void TileCache::update() {
    std::map<Key, std::vector<unsigned char>> done;
    {
        std::lock_guard<std::mutex> lck(mtx);
        done.swap(completed);
    }

    for (auto& [key, data] : done) {
        Entry& entry = entries[key];
        entry.requested = false;
        if (data.empty()) {
            entry.failed = true;
            entry.retryAt = std::chrono::steady_clock::now() + std::chrono::minutes(5);
            continue;
        }

        int w = 0, h = 0, n = 0;
        stbi_uc* pixels = stbi_load_from_memory(data.data(), (int)data.size(), &w, &h, &n, 4);
        if (!pixels) {
            entry.failed = true;
            entry.retryAt = std::chrono::steady_clock::now() + std::chrono::minutes(5);
            continue;
        }

        GLuint tex = 0;
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
        stbi_image_free(pixels);
        entry.tex = tex;
        entry.failed = false;
    }
}

void TileCache::worker() {
#ifdef ADSB_HAVE_CURL
    CURL* curl = curl_easy_init();
#endif
    while (true) {
        Key key;
        {
            std::unique_lock<std::mutex> lck(mtx);
            cv.wait(lck, [this]() { return stop || !queue.empty(); });
            if (stop && queue.empty()) { break; }
            key = queue.front();
            queue.pop();
            if (!enabled) { continue; }
        }

        std::vector<unsigned char> data = readCached(key);
        if (data.empty()) {
#ifdef ADSB_HAVE_CURL
            if (curl) {
                std::string url = urlTemplate;
                auto replaceAll = [](std::string& str, const std::string& from, const std::string& to) {
                    size_t pos = 0;
                    while ((pos = str.find(from, pos)) != std::string::npos) {
                        str.replace(pos, from.length(), to);
                        pos += to.length();
                    }
                };
                replaceAll(url, "{z}", std::to_string(key.z));
                replaceAll(url, "{x}", std::to_string(key.x));
                replaceAll(url, "{y}", std::to_string(key.y));
                data.clear();
                curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
                curl_easy_setopt(curl, CURLOPT_USERAGENT, "SDRPlusPlus-ADSB/1.0 (+https://github.com/AlexandreRouma/SDRPlusPlus)");
                curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
                curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);
                curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
                curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
                curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, +[](char* ptr, size_t size, size_t nmemb, void* userdata) {
                    auto* vec = (std::vector<unsigned char>*)userdata;
                    vec->insert(vec->end(), (unsigned char*)ptr, (unsigned char*)ptr + size * nmemb);
                    return size * nmemb;
                });
                curl_easy_setopt(curl, CURLOPT_WRITEDATA, &data);
                CURLcode res = curl_easy_perform(curl);
                long httpCode = 0;
                curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
                if (res != CURLE_OK || httpCode != 200) { data.clear(); }
                else { writeCached(key, data); }
            }
#endif
        }

        {
            std::lock_guard<std::mutex> lck(mtx);
            completed[key] = std::move(data);
        }
    }
#ifdef ADSB_HAVE_CURL
    if (curl) { curl_easy_cleanup(curl); }
#endif
}

std::vector<unsigned char> TileCache::readCached(const Key& key) {
    std::filesystem::path path = std::filesystem::path(cacheDir) /
                                 std::to_string(key.z) /
                                 std::to_string(key.x) /
                                 (std::to_string(key.y) + ".png");
    std::ifstream fs(path, std::ios::binary);
    if (!fs.is_open()) { return {}; }
    fs.seekg(0, std::ios::end);
    std::streamoff len = fs.tellg();
    if (len <= 0) { return {}; }
    fs.seekg(0, std::ios::beg);
    std::vector<unsigned char> data((size_t)len);
    fs.read((char*)data.data(), len);
    return data;
}

void TileCache::writeCached(const Key& key, const std::vector<unsigned char>& data) {
    if (data.empty()) { return; }
    std::filesystem::path dir = std::filesystem::path(cacheDir) /
                                std::to_string(key.z) /
                                std::to_string(key.x);
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    std::ofstream fs(dir / (std::to_string(key.y) + ".png"), std::ios::binary);
    fs.write((const char*)data.data(), (std::streamsize)data.size());
}

int drawTiles(ImDrawList* dl, const MercatorView& view, TileCache& cache) {
    if (!cache.isEnabled()) { return 0; }

    double lonSpan = view.lonMax - view.lonMin;
    if (lonSpan <= 0.0) { return 0; }
    int z = (int)std::lround(std::log2(360.0 * view.size.x / (TILE_SIZE * lonSpan)));
    z = std::max(2, std::min(13, z));
    int n = 1 << z;

    double x0 = (view.xMin + MAP_PI) / (2.0 * MAP_PI) * n;
    double x1 = (view.xMax + MAP_PI) / (2.0 * MAP_PI) * n;
    double y0 = (1.0 - view.yMax / MAP_PI) * 0.5 * n;
    double y1 = (1.0 - view.yMin / MAP_PI) * 0.5 * n;

    int tx0 = (int)std::floor(x0);
    int tx1 = (int)std::floor(x1);
    int ty0 = std::max(0, (int)std::floor(y0));
    int ty1 = std::min(n - 1, (int)std::floor(y1));

    int drawn = 0;
    for (int ty = ty0; ty <= ty1; ty++) {
        for (int tx = tx0; tx <= tx1; tx++) {
            int wrappedX = ((tx % n) + n) % n;
            unsigned int tex = cache.getTile(z, wrappedX, ty);
            if (!tex) { continue; }

            double lonW = (double)tx / n * 360.0 - 180.0;
            double lonE = (double)(tx + 1) / n * 360.0 - 180.0;
            double mercN = MAP_PI * (1.0 - 2.0 * ty / n);
            double mercS = MAP_PI * (1.0 - 2.0 * (ty + 1) / n);

            ImVec2 tl = view.project(lonW, latFromMercY(mercN));
            ImVec2 br = view.project(lonE, latFromMercY(mercS));
            dl->AddImage((ImTextureID)(uintptr_t)tex, tl, br);
            drawn++;
        }
    }
    return drawn;
}

unsigned int planeIconTexture() {
    static GLuint tex = 0;
    if (tex) { return tex; }
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 32, 32, 0, GL_RGBA, GL_UNSIGNED_BYTE, adsb_plane_icon_rgba);
    return tex;
}

} // namespace adsbmap
