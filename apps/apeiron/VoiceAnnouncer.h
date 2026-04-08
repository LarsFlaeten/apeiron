#pragma once

#include <string>

// ---------------------------------------------------------------------------
// VoiceAnnouncer — OBC text-to-speech announcement queue.
//
// All functions are thread-safe.  Synthesis runs on a dedicated background
// thread so calls never block the render loop.
//
// Priority levels:
//   speak()         — normal info; queued at the back.
//   warn()          — warning; inserted at the front of the queue.
//   speakImmediate()— cancels current utterance, clears the queue, speaks now.
//
// All messages are also written to a timestamped log file.
//
// Build note: requires APEIRON_VOICE to be defined (set by CMake when
// libespeak-ng-dev is found).  When not defined every function is a no-op
// so callers need no #ifdef guards.
// ---------------------------------------------------------------------------

namespace obc {

// Must be called once at startup (before any other function).
void init();

// Must be called once at shutdown — joins the background thread cleanly.
void shutdown();

// Queue a normal info announcement.
void speak(const std::string& text);

// Queue a warning; pushed to the front, ahead of any pending normal messages.
void warn(const std::string& text);

// Cancel the current utterance, discard the queue, and speak immediately.
void speakImmediate(const std::string& text);

// Discard the queue without speaking (does not cancel the current utterance).
void silence();

} // namespace obc
