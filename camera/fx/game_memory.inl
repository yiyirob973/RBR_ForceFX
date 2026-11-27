// Part of camera_fx.hpp: included from inside its anonymous namespace (ffx::cam), in the order listed there. Not a header on its own.
// ================================================================================================ game memory
// The hooks below sit at fixed addresses of the RallySimFans executable, so they are only installed in that one.
bool isRsfExe() {
    char p[MAX_PATH]; const DWORD n = GetModuleFileNameA(nullptr, p, MAX_PATH);
    std::string f(p, n); const size_t k = f.find_last_of("\\/"); if (k != std::string::npos) f = f.substr(k + 1);
    for (char& ch : f) ch = (char)std::tolower((unsigned char)ch);
    return f == "richardburnsrally_sse.exe";
}
uintptr_t absAddr(uintptr_t a) {   // 0x400000-based absolute address -> real address (handles a relocated exe)
    return (uintptr_t)GetModuleHandleA(nullptr) + a - 0x400000;
}

// Camera type via the pointer chain openRBRVR uses. 0xFFFFFFFF = not valid (yet).
uint32_t readCameraType() {
    __try {
        uintptr_t carInfo = *(uintptr_t*)absAddr(0x165FC68);
        if (!carInfo) return 0xFFFFFFFF;
        uintptr_t camData = *(uintptr_t*)(carInfo + 0x758);
        if (!camData) return 0xFFFFFFFF;
        uintptr_t camInfo = *(uintptr_t*)(camData + 0x10);
        if (!camInfo) return 0xFFFFFFFF;
        return *(uint32_t*)camInfo;
    } __except (EXCEPTION_EXECUTE_HANDLER) { return 0xFFFFFFFF; }
}
bool isCockpit(uint32_t t) { return t >= 3 && t <= 6; }      // cockpit camera family in openRBRVR (5 = interior in your logs)
bool sane3(const float* v) {
    for (int i = 0; i < 3; ++i) if (!std::isfinite(v[i]) || std::fabs(v[i]) > 1.0e5f) return false;
    return true;
}

