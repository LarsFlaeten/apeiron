#include "VoiceAnnouncer.h"

#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

#ifdef APEIRON_VOICE
#include <espeak-ng/speak_lib.h>
#endif

namespace obc {

// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------

namespace {

struct Msg {
    std::string text;
    bool        priority;  // true = warn (front-of-queue)
};

std::deque<Msg>         g_queue;
std::mutex              g_mutex;
std::condition_variable g_cv;

bool        g_running   = false;
bool        g_interrupt = false;   // signals thread to cancel current synth
std::thread g_thread;
std::ofstream g_log;

// ---------------------------------------------------------------------------
// Log a message with an ISO-ish timestamp.
void logMsg(const std::string& level, const std::string& text)
{
    if (!g_log.is_open()) return;
    using namespace std::chrono;
    auto now = system_clock::now();
    auto tt  = system_clock::to_time_t(now);
    auto ms  = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    std::tm tm{};
    localtime_r(&tt, &tm);
    char ts[32];
    std::snprintf(ts, sizeof(ts), "%02d:%02d:%02d.%03d",
                  tm.tm_hour, tm.tm_min, tm.tm_sec,
                  static_cast<int>(ms.count()));
    g_log << ts << " [" << level << "] " << text << "\n";
    g_log.flush();
}

// ---------------------------------------------------------------------------
// Background synthesis thread.
void threadFunc()
{
    while (true) {
        Msg msg;
        {
            std::unique_lock lock(g_mutex);
            g_cv.wait(lock, []{ return !g_queue.empty() || !g_running; });
            if (!g_running && g_queue.empty()) break;
            if (g_queue.empty()) continue;
            msg = std::move(g_queue.front());
            g_queue.pop_front();
            g_interrupt = false;
        }

#ifdef APEIRON_VOICE
        espeak_Synth(msg.text.c_str(),
                     msg.text.size() + 1,
                     0,
                     POS_CHARACTER,
                     0,
                     espeakCHARS_AUTO,
                     nullptr,
                     nullptr);
        espeak_Synchronize();
#endif
    }
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void init()
{
    g_log.open("obc_voice.log", std::ios::app);
    logMsg("INFO", "VoiceAnnouncer init");

#ifdef APEIRON_VOICE
    espeak_Initialize(AUDIO_OUTPUT_PLAYBACK, 0, nullptr, 0);
    espeak_SetVoiceByName("en");
    // Slightly slower, cleaner for OBC readouts.
    espeak_SetParameter(espeakRATE, 160, 0);
    espeak_SetParameter(espeakPITCH, 55, 0);
    logMsg("INFO", "eSpeak NG initialized");
#else
    logMsg("INFO", "eSpeak NG not compiled in — voice is silent");
#endif

    g_running = true;
    g_thread  = std::thread(threadFunc);
}

void shutdown()
{
    {
        std::unique_lock lock(g_mutex);
        g_running = false;
    }
    g_cv.notify_all();
    if (g_thread.joinable()) g_thread.join();

#ifdef APEIRON_VOICE
    espeak_Terminate();
#endif

    logMsg("INFO", "VoiceAnnouncer shutdown");
}

void speak(const std::string& text)
{
    logMsg("INFO", text);
    {
        std::unique_lock lock(g_mutex);
        g_queue.push_back({ text, false });
    }
    g_cv.notify_one();
}

void warn(const std::string& text)
{
    logMsg("WARN", text);
    {
        std::unique_lock lock(g_mutex);
        // Insert ahead of normal messages but after any existing warnings.
        auto it = g_queue.begin();
        while (it != g_queue.end() && it->priority) ++it;
        g_queue.insert(it, { text, true });
    }
    g_cv.notify_one();
}

void speakImmediate(const std::string& text)
{
    logMsg("IMMD", text);
    {
        std::unique_lock lock(g_mutex);
        g_queue.clear();
        g_interrupt = true;
        g_queue.push_front({ text, true });
    }
#ifdef APEIRON_VOICE
    espeak_Cancel();
#endif
    g_cv.notify_one();
}

void silence()
{
    std::unique_lock lock(g_mutex);
    g_queue.clear();
}

} // namespace obc
