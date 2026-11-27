// Part of camera_fx.hpp: included from inside its anonymous namespace (ffx::cam), in the order listed there. Not a header on its own.
// ================================================================================================ config
struct Cfg {
    // [General]
    int   apply = 1, log = 0, anyExe = 0, method = 4, autoFallback = 1, maxApplyHz = 0, diag = 0, diagDeep = 0, diagAfter = 300, test = 0, toggleKey = 121, reloadKey = 120, needCtrl = 1;
    float intensity = 1.0f;
    // [Input]  (NGP accelerations look like g: peaks ~0.5-1)
    float smooth = 0.35f, accelClamp = 2.5f, deadZone = 0.06f, minSpeed = 5.f, fullSpeed = 25.f, yawFloor = 0.2f, steerFloor = 0.1f;
    int   invSurge = 1, invSway = 1, invHeave = 1, invYaw = 1, invSteer = 1;     // 1 or -1: flips an NGP signal
    // [Translation]  metres of static head offset per unit of acceleration
    float kSurge = 0.012f, kSway = 0.014f, kHeave = 0.005f, brakeBoost = 1.15f;
    float torsoShare = 0.70f, torsoFreq = 1.2f, torsoDamp = 0.85f, neckFreq = 3.0f, neckDamp = 0.60f;
    // [Bumps]
    float bumpGain = 0.0015f, bumpFreq = 6.0f, bumpDamp = 0.45f, bumpTau = 0.25f, bumpThreshold = 0.15f;
    // [Rotation]  degrees
    float pitchPerG = 0.6f, pitchFreq = 2.8f, pitchDamp = 0.60f;
    float rollPerG = 1.0f, rollFreq = 2.4f, rollDamp = 0.60f, horizonLock = 0.0f, carRollScale = 1.0f; int invCarRoll = 1;   // roll: Method 4 only
    float lookYaw = 1.8f, lookSteerMix = 0.40f, lookFreq = 1.0f, lookDamp = 0.90f;
    // [Shift]
    float shiftKick = 0.05f;
    // [Impact]  crashes, jumps and landings
    float impactScale = 1.0f, crashThr = 3.0f, crashIgnore = 15.f, crashKick = 0.25f, crashPitchKick = 25.f, crashRollKick = 25.f, crashCooldown = 0.4f, maxKick = 0.5f;
    float airFrac = 0.08f, landFrac = 0.30f, airMs = 120.f, airRef = 0.9f, landRefG = 2.5f, landMinS = 0.25f, landMaxS = 2.0f;
    float landKick = 0.20f, landPitchKick = 10.f, airLift = 0.004f;
    float shakePos = 0.010f, shakeAng = 1.2f, shakeFreq1 = 11.f, shakeFreq2 = 17.f, shakeDecay = 0.35f, shakeGain = 0.30f;
    // [Limits]
    float maxPos = 0.025f, maxAng = 3.0f, maxRoll = 3.0f;
    // [Windscreen]  proof-of-concept overlay
    int   wsRainSource = 1, wsWetFromGame = 0, wsDrawAt = 1, wsDepthTest = 1, wsEnable = 0, wsMode = 3, wsDrops = 450, wsMud = 16, wsSignSway = 1, wsSignSurge = 1, wsToggleKey = 119,
          wsWiperSource = 1, wsWiperAxis = 0, wsWiperAxisSign = 1, wsHideGameWipers = 0, wsWiperDraw = 0, wsMicro = 3500, wsPools = 0, wsHideGameFx = 1, wsRefraction = 1, wsSnowMax = 140, wsMudSmear = 1, wsWipers = 0, wsWiperAuto = 1, wsWiperDebug = 0, wsWiperSyncKey = 118, wsFly = 1, wsWiperBow = 1, wsWiperBlur = 1, wsGround = 1, wsWiperMeasure = 1, wsCarryOver = 1;
    float wsRainLight = 0.45f, wsRainHeavy = 1.0f, wsMudDry = 0.15f, wsMudDamp = 0.7f, wsMudWet = 1.0f, wsSpeedRef = 120.f, wsRainLevel = 0.7f, wsRoughRef = 0.25f, wsRain = 100.f, wsMudRate = 6.f, wsTrail = 3.f, wsGlassDist = 0.9f, wsDropSize = 1.0f, wsMudSize = 1.0f, wsOpacity = 1.0f, wsWind = 1.0f, wsGravity = 1.0f,
          wsWiperLenM = 0.5f, wsMicroRate = 380.f, wsVeil = 0.7f, wsPoolMinPx = 8.f, wsRefractK = 2.0f, wsRefractMinPx = 4.0f, wsSnowLight = 0.5f, wsSnowHeavy = 1.0f, wsSnowLevel = 0.f, wsSnowRate = 18.f, wsSnowMelt = 14.f, wsSnowSize = 1.0f, wsMudSnow = 0.6f,
          wsWiperPeriod = 1.4f, wsWiperPause = 0.f, wsGrimeLife = 300.f, wsCarryMinutes = 10.f, wsWiperInner = 0.25f,
          wsDirt = 1.0f, wsGlow = 0.05f, wsFlyTime = 0.28f, wsMudR = 0.52f, wsMudG = 0.40f, wsMudB = 0.28f, wsWiperThick = 1.0f, wsFilmDecay = 0.15f, wsSpray = 1.0f,
          wsW1X = 0.30f, wsW1Y = 1.0f, wsW1Len = 0.70f, wsW1A0 = 5.f, wsW1A1 = 100.f, wsW2X = 0.60f, wsW2Y = 1.0f, wsW2Len = 0.f, wsW2A0 = 5.f, wsW2A1 = 100.f;
    // [Axes]  camera frame (from your log: forward = -Z, up = +Y, right = +X)
    int   axRight = 0, axUp = 1, axFwd = 2;
    int   sgRight = 1, sgUp = 1, sgFwd = -1;
    // [Render]  Method 4 (openRBRVR-style matrix patch): signs in D3D view space (x right, y up, z forward)
    int   fixedFunction = 0, rSgRight = 1, rSgUp = 1, rSgFwd = 1, rSgPitch = 1, rSgYaw = 1, rSgRoll = 1;
} g_cfg;

std::atomic<bool> g_enabled{true};        // head movement live switch (ForceFX menu, Ctrl+F10)
std::atomic<bool> g_ready{false};         // the hooks are installed and packets are being processed
std::atomic<bool> g_wsOn{false};         // Windscreen live switch (ForceFX menu, Ctrl+F8). wsEnable only says whether its hooks were installed at startup
std::string g_dir;
FILE* g_log = nullptr;                                 // ForceFXCamera.log, only when Log=1
std::function<void(const std::string&)> g_hostLog;
std::function<bool(int[4])> g_hostIds;
std::function<std::string()> g_hostCarKey;                // the car in use, from the main plugin                  // material ids under the tyres, from the surface scan (may be empty)     // ForceFX.log

// Logf: detail for tuning and bug reports, written to ForceFXCamera.log only when Log=1.
// Note: what a user needs to know when something does not work (hooks, failures); also goes to ForceFX.log.
void Logf(const char* fmt, ...) {
    if (!g_log) return;
    va_list a; va_start(a, fmt); vfprintf(g_log, fmt, a); va_end(a); fputc('\n', g_log); fflush(g_log);
}
void Note(const char* fmt, ...) {
    char b[512];
    va_list a; va_start(a, fmt); vsnprintf(b, sizeof(b), fmt, a); va_end(a);
    if (g_log) { fputs(b, g_log); fputc('\n', g_log); fflush(g_log); }
    if (g_hostLog) g_hostLog(std::string("Camera: ") + b);
}

float iniF(const std::string& path, const char* sec, const char* key, float def) {
    char b[64]; GetPrivateProfileStringA(sec, key, "", b, sizeof(b), path.c_str());
    char* semi = strchr(b, ';'); if (semi) *semi = 0;                // allow trailing ; comments
    return b[0] ? (float)atof(b) : def;
}

struct Opt { const char* sec; const char* key; float* f; int* i; float lo, hi; };
std::vector<Opt> optTable(Cfg& c) {
    return {
        {"Camera", "Apply", nullptr, &c.apply, 0, 1},           {"Camera", "Log", nullptr, &c.log, 0, 1},
        {"Camera", "AnyExe", nullptr, &c.anyExe, 0, 1}, {"Camera", "Method", nullptr, &c.method, 2, 4}, {"Camera", "AutoFallback", nullptr, &c.autoFallback, 0, 1}, {"Camera", "MaxApplyHz", nullptr, &c.maxApplyHz, 0, 240},
        {"Camera", "Diag", nullptr, &c.diag, 0, 1}, {"Camera", "DiagDeep", nullptr, &c.diagDeep, 0, 1}, {"Camera", "DiagAfter", nullptr, &c.diagAfter, 1, 100000},
        {"Camera", "Test", nullptr, &c.test, 0, 1},             {"Camera", "Intensity", &c.intensity, nullptr, 0, 2},
        {"Camera", "ToggleKey", nullptr, &c.toggleKey, 0, 254}, {"Camera", "ReloadKey", nullptr, &c.reloadKey, 0, 254},
        {"Camera", "NeedCtrl", nullptr, &c.needCtrl, 0, 1},

        {"CameraInput", "Smooth", &c.smooth, nullptr, 0, 0.95f},       {"CameraInput", "AccelClamp", &c.accelClamp, nullptr, 0.5f, 5000}, {"CameraInput", "DeadZone", &c.deadZone, nullptr, 0, 1},
        {"CameraInput", "MinSpeed", &c.minSpeed, nullptr, 0, 200},     {"CameraInput", "FullSpeed", &c.fullSpeed, nullptr, 1, 400},
        {"CameraInput", "YawFloor", &c.yawFloor, nullptr, 0.001f, 1000}, {"CameraInput", "SteerFloor", &c.steerFloor, nullptr, 0.001f, 1000},
        {"CameraInput", "InvertSurge", nullptr, &c.invSurge, -1, 1},   {"CameraInput", "InvertSway", nullptr, &c.invSway, -1, 1},
        {"CameraInput", "InvertHeave", nullptr, &c.invHeave, -1, 1},   {"CameraInput", "InvertYaw", nullptr, &c.invYaw, -1, 1},
        {"CameraInput", "InvertSteer", nullptr, &c.invSteer, -1, 1},

        {"CameraTranslation", "Surge", &c.kSurge, nullptr, 0, 0.2f},   {"CameraTranslation", "Sway", &c.kSway, nullptr, 0, 0.2f},
        {"CameraTranslation", "Heave", &c.kHeave, nullptr, 0, 0.2f},   {"CameraTranslation", "BrakeBoost", &c.brakeBoost, nullptr, 0.5f, 3},
        {"CameraTranslation", "TorsoShare", &c.torsoShare, nullptr, 0, 1},
        {"CameraTranslation", "TorsoFreq", &c.torsoFreq, nullptr, 0.3f, 8}, {"CameraTranslation", "TorsoDamp", &c.torsoDamp, nullptr, 0.2f, 2},
        {"CameraTranslation", "NeckFreq", &c.neckFreq, nullptr, 0.5f, 12}, {"CameraTranslation", "NeckDamp", &c.neckDamp, nullptr, 0.1f, 2},

        {"CameraBumps", "Gain", &c.bumpGain, nullptr, 0, 0.1f},        {"CameraBumps", "Freq", &c.bumpFreq, nullptr, 1, 20},
        {"CameraBumps", "Damp", &c.bumpDamp, nullptr, 0.1f, 2},        {"CameraBumps", "HighPassTau", &c.bumpTau, nullptr, 0.05f, 2}, {"CameraBumps", "Threshold", &c.bumpThreshold, nullptr, 0, 3},

        {"CameraRotation", "PitchPerG", &c.pitchPerG, nullptr, 0, 20}, {"CameraRotation", "PitchFreq", &c.pitchFreq, nullptr, 0.3f, 10},
        {"CameraRotation", "PitchDamp", &c.pitchDamp, nullptr, 0.2f, 2}, {"CameraRotation", "LookYaw", &c.lookYaw, nullptr, 0, 30},
        {"CameraRotation", "LookSteerMix", &c.lookSteerMix, nullptr, 0, 1}, {"CameraRotation", "LookFreq", &c.lookFreq, nullptr, 0.2f, 6},
        {"CameraRotation", "LookDamp", &c.lookDamp, nullptr, 0.2f, 2},
        {"CameraRotation", "RollPerG", &c.rollPerG, nullptr, 0, 10}, {"CameraRotation", "RollFreq", &c.rollFreq, nullptr, 0.3f, 10},
        {"CameraRotation", "RollDamp", &c.rollDamp, nullptr, 0.2f, 2}, {"CameraRotation", "HorizonLock", &c.horizonLock, nullptr, 0, 1},
        {"CameraRotation", "CarRollScale", &c.carRollScale, nullptr, 0, 100}, {"CameraRotation", "InvertCarRoll", nullptr, &c.invCarRoll, -1, 1},

        {"CameraShift", "Kick", &c.shiftKick, nullptr, -2, 2},

        {"CameraImpact", "Scale", &c.impactScale, nullptr, 0, 3}, {"CameraImpact", "CrashThreshold", &c.crashThr, nullptr, 0.5f, 50},
        {"CameraImpact", "CrashIgnoreAbove", &c.crashIgnore, nullptr, 2, 500}, {"CameraImpact", "CrashKick", &c.crashKick, nullptr, 0, 2},
        {"CameraImpact", "CrashPitchKick", &c.crashPitchKick, nullptr, 0, 200}, {"CameraImpact", "CrashRollKick", &c.crashRollKick, nullptr, 0, 200},
        {"CameraImpact", "CrashCooldown", &c.crashCooldown, nullptr, 0.1f, 5}, {"CameraImpact", "MaxKick", &c.maxKick, nullptr, 0.05f, 2},
        {"CameraImpact", "AirLoadFraction", &c.airFrac, nullptr, 0.01f, 0.5f}, {"CameraImpact", "LandLoadFraction", &c.landFrac, nullptr, 0.05f, 0.9f},
        {"CameraImpact", "AirborneMs", &c.airMs, nullptr, 30, 1000}, {"CameraImpact", "AirTimeRef", &c.airRef, nullptr, 0.2f, 3},
        {"CameraImpact", "LandRefG", &c.landRefG, nullptr, 0.5f, 20}, {"CameraImpact", "LandMinStrength", &c.landMinS, nullptr, 0, 2},
        {"CameraImpact", "LandMaxStrength", &c.landMaxS, nullptr, 0.5f, 5}, {"CameraImpact", "LandKick", &c.landKick, nullptr, 0, 2},
        {"CameraImpact", "LandPitchKick", &c.landPitchKick, nullptr, 0, 200}, {"CameraImpact", "AirLift", &c.airLift, nullptr, 0, 0.05f},
        {"CameraImpact", "ShakePos", &c.shakePos, nullptr, 0, 0.05f}, {"CameraImpact", "ShakeAngleDeg", &c.shakeAng, nullptr, 0, 10},
        {"CameraImpact", "ShakeFreq1", &c.shakeFreq1, nullptr, 2, 40}, {"CameraImpact", "ShakeFreq2", &c.shakeFreq2, nullptr, 2, 40},
        {"CameraImpact", "ShakeDecay", &c.shakeDecay, nullptr, 0.05f, 3}, {"CameraImpact", "ShakeGain", &c.shakeGain, nullptr, 0, 2},

        {"CameraLimits", "MaxPos", &c.maxPos, nullptr, 0.005f, 0.15f}, {"CameraLimits", "MaxAngleDeg", &c.maxAng, nullptr, 0.5f, 25}, {"CameraLimits", "MaxRollDeg", &c.maxRoll, nullptr, 0.5f, 15},

        {"Windscreen", "Mode", nullptr, &c.wsMode, 1, 3},
        {"Windscreen", "WetFromGame", nullptr, &c.wsWetFromGame, 0, 1},
        {"Windscreen", "DrawAt", nullptr, &c.wsDrawAt, 0, 2},    {"Windscreen", "DepthTest", nullptr, &c.wsDepthTest, 0, 1},
        {"Windscreen", "GlassDist", &c.wsGlassDist, nullptr, 0.2f, 5},
        {"Windscreen", "RainSource", nullptr, &c.wsRainSource, 0, 1},
        {"Windscreen", "RainLight", &c.wsRainLight, nullptr, 0, 1}, {"Windscreen", "RainHeavy", &c.wsRainHeavy, nullptr, 0, 1},
        {"Windscreen", "MudDry", &c.wsMudDry, nullptr, 0, 3}, {"Windscreen", "MudDamp", &c.wsMudDamp, nullptr, 0, 3}, {"Windscreen", "MudWet", &c.wsMudWet, nullptr, 0, 3},
        {"Windscreen", "RainLevel", &c.wsRainLevel, nullptr, 0, 1}, {"Windscreen", "SpeedRef", &c.wsSpeedRef, nullptr, 20, 300},
        {"Windscreen", "RoughRef", &c.wsRoughRef, nullptr, 0.01f, 5},
        {"Windscreen", "Rain", &c.wsRain, nullptr, 0, 100},      {"Windscreen", "MudRate", &c.wsMudRate, nullptr, 0, 60},
        {"Windscreen", "Trail", &c.wsTrail, nullptr, 0.5f, 30},
        {"Windscreen", "Drops", nullptr, &c.wsDrops, 0, 400},    {"Windscreen", "DropSize", &c.wsDropSize, nullptr, 0.2f, 4},
        {"Windscreen", "Mud", nullptr, &c.wsMud, 0, 120},         {"Windscreen", "MudSize", &c.wsMudSize, nullptr, 0.2f, 4},
        {"Windscreen", "Opacity", &c.wsOpacity, nullptr, 0, 1},  {"Windscreen", "Wind", &c.wsWind, nullptr, 0, 3},
        {"Windscreen", "Gravity", &c.wsGravity, nullptr, 0, 3},
        {"Windscreen", "SignSway", nullptr, &c.wsSignSway, -1, 1}, {"Windscreen", "SignSurge", nullptr, &c.wsSignSurge, -1, 1},
        {"Windscreen", "ToggleKey", nullptr, &c.wsToggleKey, 0, 254},
        {"Windscreen", "HideGameFx", nullptr, &c.wsHideGameFx, 0, 2},
        {"Windscreen", "Micro", nullptr, &c.wsMicro, 0, 8000}, {"Windscreen", "MicroRate", &c.wsMicroRate, nullptr, 0, 1000}, {"Windscreen", "Veil", &c.wsVeil, nullptr, 0, 1},
        {"Windscreen", "Pools", nullptr, &c.wsPools, 0, 1}, {"Windscreen", "PoolMinPx", &c.wsPoolMinPx, nullptr, 3, 80},
        {"Windscreen", "Refraction", nullptr, &c.wsRefraction, 0, 1}, {"Windscreen", "RefractK", &c.wsRefractK, nullptr, 0.2f, 6}, {"Windscreen", "RefractMinPx", &c.wsRefractMinPx, nullptr, 1, 40},
        {"Windscreen", "SnowLight", &c.wsSnowLight, nullptr, 0, 1}, {"Windscreen", "SnowHeavy", &c.wsSnowHeavy, nullptr, 0, 1}, {"Windscreen", "SnowLevel", &c.wsSnowLevel, nullptr, 0, 1},
        {"Windscreen", "SnowRate", &c.wsSnowRate, nullptr, 0, 100}, {"Windscreen", "SnowMelt", &c.wsSnowMelt, nullptr, 1, 300}, {"Windscreen", "SnowSize", &c.wsSnowSize, nullptr, 0.2f, 4},
        {"Windscreen", "SnowMax", nullptr, &c.wsSnowMax, 0, 400}, {"Windscreen", "MudSnow", &c.wsMudSnow, nullptr, 0, 3}, {"Windscreen", "MudSmear", nullptr, &c.wsMudSmear, 0, 1},
        {"Windscreen", "Fly", nullptr, &c.wsFly, 0, 1}, {"Windscreen", "FlyTime", &c.wsFlyTime, nullptr, 0.08f, 1.5f}, {"Windscreen", "Dirt", &c.wsDirt, nullptr, 0, 2}, {"Windscreen", "Glow", &c.wsGlow, nullptr, 0, 0.6f},
        {"Windscreen", "FilmDecay", &c.wsFilmDecay, nullptr, 0, 2}, {"Windscreen", "Spray", &c.wsSpray, nullptr, 0, 4},
        {"Windscreen", "MudR", &c.wsMudR, nullptr, 0, 1}, {"Windscreen", "MudG", &c.wsMudG, nullptr, 0, 1}, {"Windscreen", "MudB", &c.wsMudB, nullptr, 0, 1},
        {"Windscreen", "WiperThick", &c.wsWiperThick, nullptr, 0.3f, 3}, {"Windscreen", "WiperBow", nullptr, &c.wsWiperBow, 0, 1}, {"Windscreen", "WiperBlur", nullptr, &c.wsWiperBlur, 0, 1},
        {"Windscreen", "Wipers", nullptr, &c.wsWipers, 0, 1}, {"Windscreen", "WiperSource", nullptr, &c.wsWiperSource, 0, 1},
        {"Windscreen", "WiperAxis", nullptr, &c.wsWiperAxis, 0, 2}, {"Windscreen", "WiperAxisSign", nullptr, &c.wsWiperAxisSign, -1, 1}, {"Windscreen", "WiperLenM", &c.wsWiperLenM, nullptr, 0.1f, 2},
        {"Windscreen", "HideGameWipers", nullptr, &c.wsHideGameWipers, 0, 1}, {"Windscreen", "WiperDraw", nullptr, &c.wsWiperDraw, 0, 1}, {"Windscreen", "WiperAuto", nullptr, &c.wsWiperAuto, 0, 1}, {"Windscreen", "WiperDebug", nullptr, &c.wsWiperDebug, 0, 1},
        {"Windscreen", "Ground", nullptr, &c.wsGround, 0, 1}, {"Windscreen", "CarryOver", nullptr, &c.wsCarryOver, 0, 1}, {"Windscreen", "CarryMinutes", &c.wsCarryMinutes, nullptr, 0, 600}, {"Windscreen", "GrimeLife", &c.wsGrimeLife, nullptr, 0, 3600},
        {"Windscreen", "WiperMeasure", nullptr, &c.wsWiperMeasure, 0, 1}, {"Windscreen", "WiperInner", &c.wsWiperInner, nullptr, 0, 0.9f},
        {"Windscreen", "WiperSyncKey", nullptr, &c.wsWiperSyncKey, 0, 254}, {"Windscreen", "WiperPeriod", &c.wsWiperPeriod, nullptr, 0.3f, 10}, {"Windscreen", "WiperPause", &c.wsWiperPause, nullptr, 0, 20},
        {"Windscreen", "W1X", &c.wsW1X, nullptr, -0.5f, 1.5f}, {"Windscreen", "W1Y", &c.wsW1Y, nullptr, -0.5f, 1.5f}, {"Windscreen", "W1Len", &c.wsW1Len, nullptr, 0, 3},
        {"Windscreen", "W1A0", &c.wsW1A0, nullptr, -360, 360}, {"Windscreen", "W1A1", &c.wsW1A1, nullptr, -360, 360},
        {"Windscreen", "W2X", &c.wsW2X, nullptr, -0.5f, 1.5f}, {"Windscreen", "W2Y", &c.wsW2Y, nullptr, -0.5f, 1.5f}, {"Windscreen", "W2Len", &c.wsW2Len, nullptr, 0, 3},
        {"Windscreen", "W2A0", &c.wsW2A0, nullptr, -360, 360}, {"Windscreen", "W2A1", &c.wsW2A1, nullptr, -360, 360},

        {"CameraAxes", "Right", nullptr, &c.axRight, 0, 2}, {"CameraAxes", "Up", nullptr, &c.axUp, 0, 2}, {"CameraAxes", "Forward", nullptr, &c.axFwd, 0, 2},
        {"CameraAxes", "SignRight", nullptr, &c.sgRight, -1, 1}, {"CameraAxes", "SignUp", nullptr, &c.sgUp, -1, 1}, {"CameraAxes", "SignForward", nullptr, &c.sgFwd, -1, 1},
        {"CameraRender", "FixedFunction", nullptr, &c.fixedFunction, 0, 1},
        {"CameraRender", "SignRight", nullptr, &c.rSgRight, -1, 1}, {"CameraRender", "SignUp", nullptr, &c.rSgUp, -1, 1}, {"CameraRender", "SignForward", nullptr, &c.rSgFwd, -1, 1},
        {"CameraRender", "SignPitch", nullptr, &c.rSgPitch, -1, 1}, {"CameraRender", "SignYaw", nullptr, &c.rSgYaw, -1, 1}, {"CameraRender", "SignRoll", nullptr, &c.rSgRoll, -1, 1},
    };
}

// ---- per-stage rain / mud for Windscreen. RBR has no weather system: whether a stage is wet is a fixed property of the stage, and the
// telemetry does not say. The plugin is told the map id when a stage starts (not its name), so [WindscreenStages] maps id -> rain[,mud].
struct WsStageRule { int id; float rain, mud; };
std::vector<WsStageRule> g_wsStageRules;
int   g_wsStageId = -1;
float g_wsRainEff = 0.7f, g_wsMudMul = 1.f, g_wsRuleRain = -1.f;                         // rain level (0..1) and mud multiplier in force for the current stage
void wsLoadStageRules(const std::string& path) {
    g_wsStageRules.clear();
    static char buf[8192];
    const DWORD n = GetPrivateProfileSectionA("WindscreenStages", buf, sizeof(buf), path.c_str());
    for (const char* q = buf; q < buf + n && *q; q += strlen(q) + 1) {
        const char* eq = strchr(q, '='); if (!eq) continue;
        char* end = nullptr; const long id = strtol(q, &end, 10);
        if (end == q) continue;                                        // keys must start with the map id (comment lines never do)
        char* e2 = nullptr; const float rain = strtof(eq + 1, &e2);
        if (e2 == eq + 1) continue;
        while (*e2 == ' ' || *e2 == '\t') ++e2;
        float mud = 1.f; if (*e2 == ',') mud = strtof(e2 + 1, nullptr);
        g_wsStageRules.push_back({(int)id, std::max(0.f, std::min(1.f, rain)), std::max(0.f, std::min(5.f, mud))});
    }
}
void wsApplyStage() {                                                // an explicit [WindscreenStages] rule overrides what is read from the game
    g_wsRuleRain = -1.f; g_wsMudMul = 1.f;
    for (const WsStageRule& r : g_wsStageRules) if (r.id == g_wsStageId) {
        g_wsRuleRain = r.rain; g_wsMudMul = r.mud;
        Logf("windscreen: map id %d has an override rule: rain=%.2f mud x%.2f", g_wsStageId, g_wsRuleRain, g_wsMudMul); return;
    }
    if (!g_wsStageRules.empty()) Logf("windscreen: map id %d has no override rule (%d rules loaded)", g_wsStageId, (int)g_wsStageRules.size());
}

int g_rawMethod = 2;
void loadConfig() {
    const std::string p = g_dir + "ForceFX.ini";     // a missing file or key simply uses the built-in default
    g_rawMethod = (int)std::lround(iniF(p, "Camera", "Method", 2));
    Cfg c; auto t = optTable(c);
    for (const Opt& o : t) {
        if (o.f) *o.f = std::max(o.lo, std::min(iniF(p, o.sec, o.key, *o.f), o.hi));
        else     *o.i = (int)std::lround(std::max(o.lo, std::min(iniF(p, o.sec, o.key, (float)*o.i), o.hi)));
    }
    auto sgn = [](int& v) { v = (v < 0) ? -1 : 1; };                 // 0 is not a valid sign
    sgn(c.invSurge); sgn(c.invSway); sgn(c.invHeave); sgn(c.invYaw); sgn(c.invSteer); sgn(c.sgRight); sgn(c.sgUp); sgn(c.sgFwd);
    sgn(c.wsSignSway); sgn(c.wsSignSurge); sgn(c.invCarRoll); sgn(c.rSgRight); sgn(c.rSgUp); sgn(c.rSgFwd); sgn(c.rSgPitch); sgn(c.rSgYaw); sgn(c.rSgRoll);
    if (c.axRight == c.axUp || c.axRight == c.axFwd || c.axUp == c.axFwd) { c.axRight = 0; c.axUp = 1; c.axFwd = 2; }
    if (c.fullSpeed <= c.minSpeed + 0.5f) c.fullSpeed = c.minSpeed + 0.5f;
    c.wsEnable = g_cfg.wsEnable;                                     // set once at startup from the ForceFX settings; not part of this file
    g_cfg = c;
    wsLoadStageRules(p); wsApplyStage();
}

