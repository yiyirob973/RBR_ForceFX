// waveOut output for the dynamic surface audio: 16-bit stereo, a small ring of buffers filled from the mixer on its own thread.
// Windows only. RBR is a 32-bit process, so this is built as Win32 (x86) with the rest of ForceFX.
#pragma once
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <windows.h>
#include <mmsystem.h>
#include <atomic>
#include <string>
#include <thread>
#include <vector>
#include "audio.hpp"

#pragma comment(lib, "winmm.lib")

namespace rsa {

// 16-bit stereo waveOut with a small ring of buffers filled from the mixer on its own thread.
class WaveOutPlayer {
public:
    std::string error;
    std::atomic<int> writeFailures{0}, firstWriteError{0};   // waveOutWrite results, shown in the debug log

    // "0: Speakers (Realtek) | 1: Headset ..." so OutputDevice in the ini can be chosen by number.
    static std::string describeDevices() {
        std::string out;
        UINT n = waveOutGetNumDevs();
        for (UINT i = 0; i < n; ++i) {
            WAVEOUTCAPSA caps{};
            if (waveOutGetDevCapsA(i, &caps, sizeof caps) == MMSYSERR_NOERROR) out += (out.empty() ? "" : " | ") + std::to_string(i) + ": " + caps.szPname;
        }
        return out.empty() ? std::string("no waveOut devices found") : out;
    }

    bool start(Mixer& mixer, int device) {
        mixer_ = &mixer;
        WAVEFORMATEX fmt{};
        fmt.wFormatTag = WAVE_FORMAT_PCM; fmt.nChannels = 2; fmt.nSamplesPerSec = kSampleRate;
        fmt.wBitsPerSample = 16; fmt.nBlockAlign = 4; fmt.nAvgBytesPerSec = kSampleRate * 4;
        event_ = CreateEvent(nullptr, FALSE, FALSE, nullptr);
        MMRESULT r = waveOutOpen(&hwo_, device < 0 ? WAVE_MAPPER : (UINT)device, &fmt, (DWORD_PTR)event_, 0, CALLBACK_EVENT);
        if (r != MMSYSERR_NOERROR) { error = "waveOutOpen failed, code " + std::to_string((int)r); CloseHandle(event_); event_ = nullptr; hwo_ = nullptr; return false; }
        for (int i = 0; i < kBuffers; ++i) {
            pcm_[i].assign(kFrames * 2, 0);
            hdr_[i] = WAVEHDR{};
            hdr_[i].lpData = (LPSTR)pcm_[i].data();
            hdr_[i].dwBufferLength = (DWORD)(pcm_[i].size() * sizeof(int16_t));
            waveOutPrepareHeader(hwo_, &hdr_[i], sizeof(WAVEHDR));
            fillAndQueue(i);
        }
        running_ = true;
        thread_ = std::thread(&WaveOutPlayer::loop, this);
        return true;
    }

    void stop() {
        running_ = false;
        if (event_) SetEvent(event_);
        if (thread_.joinable()) thread_.join();
        if (hwo_) {
            waveOutReset(hwo_);
            for (int i = 0; i < kBuffers; ++i) waveOutUnprepareHeader(hwo_, &hdr_[i], sizeof(WAVEHDR));
            waveOutClose(hwo_); hwo_ = nullptr;
        }
        if (event_) { CloseHandle(event_); event_ = nullptr; }
    }
    ~WaveOutPlayer() { stop(); }

private:
    static constexpr int kBuffers = 4, kFrames = 512;
    Mixer* mixer_ = nullptr;
    HWAVEOUT hwo_ = nullptr;
    HANDLE event_ = nullptr;
    WAVEHDR hdr_[kBuffers];
    std::vector<int16_t> pcm_[kBuffers];
    std::vector<float> tmp_ = std::vector<float>(kFrames * 2);
    std::thread thread_;
    std::atomic<bool> running_{false};

    void fillAndQueue(int i) {
        mixer_->read(tmp_.data(), kFrames);
        for (int k = 0; k < kFrames * 2; ++k) {
            float v = tmp_[k] < -1.f ? -1.f : (tmp_[k] > 1.f ? 1.f : tmp_[k]);
            pcm_[i][k] = (int16_t)(v * 32767.f);
        }
        hdr_[i].dwFlags &= ~WHDR_DONE;
        MMRESULT wr = waveOutWrite(hwo_, &hdr_[i], sizeof(WAVEHDR));
        if (wr != MMSYSERR_NOERROR) { if (writeFailures++ == 0) firstWriteError = (int)wr; }
    }
    void loop() {
        while (running_) {
            WaitForSingleObject(event_, 50);
            for (int i = 0; i < kBuffers && running_; ++i)
                if (hdr_[i].dwFlags & WHDR_DONE) fillAndQueue(i);
        }
    }
};

}  // namespace rsa
