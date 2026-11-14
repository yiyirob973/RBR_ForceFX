// RBR_ForceFX.cpp
//
// Stable version:
// - confirmed wheel placement is LF/RF/LB/RB in telemetry order
// - left / right per-tire surface handling is kept
// - temporary memory-scan / MemScan branch removed
//

// ------------------------------------------------------------------ telemetry
// Car.Index is a uint32 in some NGP builds and an integral float in others, so accept either.
inline bool readIndex(const uint8_t* b, int len, int off, int& out) {
    if (off < 0 || off + 4 > len) return false;
    uint32_t u; std::memcpy(&u, b + off, 4);
    if (u <= 255u) { out = (int)u; return true; }
    float f; std::memcpy(&f, b + off, 4);
    if (std::isfinite(f) && f >= 0.0f && f <= 255.0f && f == std::floor(f)) { out = (int)f; return true; }
    return false;
}

inline bool readF(const uint8_t* b, int len, int off, float& out) {
    if (off < 0 || off + 4 > len) return false;
    float v; std::memcpy(&v, b + off, 4);
    if (!std::isfinite(v)) return false;
    out = v;
    return true;
}

struct Signals {
    float rpm = 0, speed = 0 /* m/s */, throttle = 0, brake = 0;
    float rpmMaxTelem = 0;   // rev limit straight from the telemetry (0 = not available)
    float rough = 0;      // raw front-weighted tyre roughness
    float piston = 0;     // damper piston velocity level
    float texture = 0;    // combined road texture 0..1
    float texL = 0, texR = 0;   // per-side surface texture: left / right
    float bump = 0;       // heave acceleration spike envelope
    float dive = 0;       // front load transfer under braking 0..1
    float lock = 0;       // front wheel lock-up 0..1
    float frontRev = 0;   // front wheel revolutions per second
    int   carIndex = -1;  // NGP Car.Index (-1 = unknown)
    // weight / suspension load, relative to this car's own static load (1 = at rest)
    float loadL = 1, loadR = 1, loadF = 1;
    float airborne = 0;   // 0 = wheels loaded, 1 = off the ground
    int   surf = 0;       // ground class under the car (Surf::Cls); 0 = unknown, which changes nothing
    float thumpL = 0, thumpR = 0;   // compression / landing envelope per side
    unsigned evtSeq = 0;  // goes up by one for every landing or hard hit; the synth starts one boost per event
    float evtSev = 0;     // severity of the current event 0..1 (refined upward while a landing is still compressing)
    float staticLoad = 0; // learned static strut load, all four wheels (units of the telemetry)
};

struct Hp  { float y = 0, x = 0; bool init = false;
             float step(float in, float a) { if (!init) { x = in; init = true; y = 0; return 0; } y = a * (y + in - x); x = in; return y; } };
struct Ema { float v = 0; bool init = false;
             float step(float in, float a) { if (!init) { v = in; init = true; } else v += (in - v) * a; return v; } };

// Analysis state that carries over from packet to packet. NGP wheel order: LF, RF, LB, RB.
// Stable mapping: front-left = 0, front-right = 1, rear-left = 2, rear-right = 3.
// The log's "tire 1 should be front left, tire 3 should be rear left" is therefore
// an external log index convention, not a change to the NGP low-level order.
struct Dsp {
    Hp  sfHp[4], pistonHp[4], accHp;
    Ema sfVar[4], pistonVar[4], loadShort[4], rev[4];
    Ema loadRef, rollRatio, baseFront;
    uint32_t lastSeg[4] = {0, 0, 0, 0};
    bool haveSeg[4] = {false, false, false, false};
    float bumpEnv = 0, diveSm = 0, lockSm = 0;
    Ema statRef[4], rLsm, rRsm, rFsm;
    float thumpL = 0, thumpR = 0;
    float peakRw = 0, totRatio = 1;
    float airT = 0, landWin = 0, landAir = 0, landPeak = 0, landBump = 0, landJump = 0, imRefr = 0;
    unsigned evtSeq = 0; float evtSev = 0;
    float totLow = 1.0f;
};

void ParsePacket(const Config& c, const uint8_t* b, int n, Signals& s, Dsp& d, float dt) {
    dt = clampf(dt, 0.002f, 0.05f);
    float v;
    { int ci; if (readIndex(b, n, c.offCarIndex, ci)) s.carIndex = ci; }
    if (readF(b, n, c.offRpm, v))   s.rpm = std::max(0.0f, v * c.rpmScale);
    if (readF(b, n, c.offSpeed, v)) s.speed = std::fabs(v) * (c.speedIsKph ? (1.0f / 3.6f) : 0.44704f);
    if (readF(b, n, c.offRpmMax, v) && v * c.rpmScale >= 1500.0f && v * c.rpmScale <= 20000.0f) s.rpmMaxTelem = v * c.rpmScale;
    s.throttle = readF(b, n, c.offThrottle, v) ? clamp01(v) : 0.6f;
    s.brake    = readF(b, n, c.offBrake, v)    ? clamp01(v) : 0.0f;

    const float aHp = hpCoef(dt, 0.032f);
    const float aVar = alphaFor(dt, 0.10f);
    const float aLoad = alphaFor(dt, 0.3f);

    float rough[4] = {0, 0, 0, 0}, absSf[4] = {0, 0, 0, 0}, pRms[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4 && c.offSuspBase >= 0; ++i) {
        const int base = c.offSuspBase + i * c.suspStride;
        float sf = 0, pv = 0;
        if (readF(b, n, base + c.offStrut, sf)) {
            absSf[i] = std::fabs(sf);
            float ls = d.loadShort[i].step(absSf[i], aLoad);
            float hpSf = d.sfHp[i].step(sf, aHp);
            float var = d.sfVar[i].step(hpSf * hpSf, aVar);
            float denom = std::max(ls, 0.3f * d.loadRef.v);
            rough[i] = denom > 1e-6f ? std::sqrt(std::max(var, 0.0f)) / denom : 0.0f;
        }
        if (readF(b, n, base + c.offPiston, pv)) {
            float hp = d.pistonHp[i].step(pv, hpCoef(dt, 0.04f));
            pRms[i] = std::sqrt(std::max(d.pistonVar[i].step(hp * hp, aVar), 0.0f));
        }
        int so = base + c.offSegment;
        if (so >= 0 && so + 4 <= n) {
            uint32_t seg; std::memcpy(&seg, b + so, 4);
            uint32_t sg = seg % 8u;
            if (d.haveSeg[i]) {
                int delta = (int)((sg + 8u - d.lastSeg[i]) % 8u);
                d.rev[i].step(delta / 8.0f / dt, alphaFor(dt, 0.15f));
            }
            d.lastSeg[i] = sg; d.haveSeg[i] = true;
        }
    }

    const float meanLoad = 0.25f * (absSf[0] + absSf[1] + absSf[2] + absSf[3]);
    if (s.speed > 3.0f && meanLoad > 0) d.loadRef.step(meanLoad, alphaFor(dt, 8.0f));

    {
        const float tot = absSf[0] + absSf[1] + absSf[2] + absSf[3];
        float statTot = 0; bool seeded = true;
        for (int i = 0; i < 4; ++i) { statTot += d.statRef[i].v; seeded = seeded && d.statRef[i].init; }
        if (!seeded || tot > 0.5f * statTot)
            for (int i = 0; i < 4; ++i) d.statRef[i].step(absSf[i], alphaFor(dt, 30.0f));
        statTot = 0;
        for (int i = 0; i < 4; ++i) statTot += d.statRef[i].v;
        s.staticLoad = statTot;
        if (statTot > 1e-3f) {
            float rw[4];
            for (int i = 0; i < 4; ++i)
                rw[i] = clampf(absSf[i] / std::max(d.statRef[i].v, 0.0375f * statTot), 0.0f, 3.5f);
            const float rl = 0.65f * rw[0] + 0.35f * rw[2];
            const float rr2 = 0.65f * rw[1] + 0.35f * rw[3];
            const float aR = alphaFor(dt, 0.08f);
            s.loadL = d.rLsm.step(rl, aR);
            s.loadR = d.rRsm.step(rr2, aR);
            s.loadF = d.rFsm.step(0.5f * (rw[0] + rw[1]), aR);
            s.airborne = s.speed > 3.0f ? 1.0f - smooth01(0.06f, 0.35f, tot / statTot) : 0.0f;
            d.totRatio = tot / statTot;
            d.peakRw = std::max(std::max(rw[0], rw[1]), std::max(rw[2], rw[3]));
            const float eDecay = std::exp(-dt / 0.14f);
            d.thumpL = std::max(clamp01((rl  - 1.7f) / 1.2f), d.thumpL * eDecay);
            d.thumpR = std::max(clamp01((rr2 - 1.7f) / 1.2f), d.thumpR * eDecay);
            s.thumpL = d.thumpL; s.thumpR = d.thumpR;
        }
    }

    // Road texture: front tyres dominate for steering, but the left / right side split is retained.
    const float rF = std::max(rough[0], rough[1]);
    const float rR = std::max(rough[2], rough[3]);
    s.rough = 0.75f * rF + 0.25f * rR;
    const float roughLevel = std::pow(clamp01((s.rough - c.roughLow) / (c.roughHigh - c.roughLow)), c.roughGamma);
    s.piston = std::pow(clamp01(std::max(pRms[0], pRms[1]) * c.pistonScale), 1.2f);
    s.texture = clamp01(std::max(roughLevel, 0.8f * s.piston));

    // Stable left/right side mapping from NGP telemetry order:
    // LF=0, RF=1, LB=2, RB=3
    auto side = [&](int front, int back) {
        const float r = 0.75f * rough[front] + 0.25f * rough[back];
        const float lv = std::pow(clamp01((r - c.roughLow) / (c.roughHigh - c.roughLow)), c.roughGamma);
        const float pi = std::pow(clamp01(pRms[front] * c.pistonScale), 1.2f);
        return clamp01(std::max(lv, 0.8f * pi));
    };
    s.texL = side(0, 2);
    s.texR = side(1, 3);

    // bumps / landings
    if (readF(b, n, c.offAccHeave, v)) {
        float hp = d.accHp.step(v, hpCoef(dt, 0.05f));
        d.bumpEnv = std::max(std::fabs(hp) * c.bumpScale, d.bumpEnv * std::exp(-dt / 0.12f));
    }
    s.bump = clamp01(d.bumpEnv);

    // landings / hard hits
    {
        d.imRefr = std::max(0.0f, d.imRefr - dt);
        const float rawTot = d.totRatio;
        d.totLow = std::min(rawTot, d.totLow + (rawTot - d.totLow) * alphaFor(dt, 0.12f));
        const float jump = std::max(0.0f, rawTot - d.totLow);
        const bool off = s.speed > 3.0f && rawTot < c.airLevel;

        auto severity = [&](float peak, float air, float bump, float jmp) {
            const bool sens = air > 0.0f;
            const float peakT = clamp01((peak - (sens ? 1.15f : 1.3f)) / (sens ? 1.3f : 1.7f));
            const float jumpT = clamp01((jmp - (sens ? 0.25f : 0.35f)) / (sens ? 1.0f : 1.4f));
            const float airT2 = clamp01(air / 0.5f);
            const float bumpT = clamp01(bump);
            const float top = std::max(std::max(peakT, jumpT), std::max(0.85f * airT2, 0.7f * bumpT));
            const float sum = peakT + jumpT + airT2 + bumpT;
            float sev = clamp01(top + 0.15f * (sum - top));
            if (air > 0.04f) sev = std::max(sev, 0.55f + 0.45f * airT2);
            return sev;
        };

        auto fire = [&](float sev) { ++d.evtSeq; d.evtSev = sev; };

        if (off) {
            d.airT += dt;
        } else {
            if (d.airT > 0.04f && d.landWin <= 0.0f) {
                d.landWin = 0.10f; d.landAir = d.airT; d.landPeak = 0; d.landBump = 0; d.landJump = 0;
                fire(severity(0.0f, d.landAir, 0.0f, 0.0f));
            }
            d.airT = 0;
        }
        if (d.landWin > 0.0f) {
            d.landPeak = std::max(d.landPeak, d.peakRw);
            d.landBump = std::max(d.landBump, clamp01(d.bumpEnv));
            d.landJump = std::max(d.landJump, jump);
            const float sev = severity(d.landPeak, d.landAir, d.landBump, d.landJump);
            d.evtSev = std::max(d.evtSev, sev);
            d.landWin -= dt;
            if (d.landWin <= 0.0f) {
                d.landWin = 0.0f; d.imRefr = 0.15f;
            }
        } else if (!off && d.imRefr <= 0.0f && jump > 0.35f * c.landingThresh && d.peakRw > 1.0f + 0.5f * c.landingThresh) {
            const float sev = severity(d.peakRw, 0.0f, clamp01(d.bumpEnv), jump);
            if (sev > 0.12f) {
                d.imRefr = 0.15f;
                fire(sev);
            }
        }
        s.evtSeq = d.evtSeq;
        s.evtSev = clamp01(d.evtSev * c.impactGain);
    }

    // braking: weight moving onto the front tyres and front wheel lock-up
    const float tot = absSf[0] + absSf[1] + absSf[2] + absSf[3];
    if (tot > 1e-3f) {
        float share = (absSf[0] + absSf[1]) / tot;
        if (s.brake < 0.05f && s.speed > 3.0f) d.baseFront.step(share, alphaFor(dt, 2.0f));
        float target = (s.brake > 0.1f && d.baseFront.init) ? clamp01((share - d.baseFront.v) / 0.10f) : 0.0f;
        d.diveSm += (target - d.diveSm) * alphaFor(dt, 0.1f);
    }
    s.dive = d.diveSm;

    const float rr = 0.5f * (d.rev[0].v + d.rev[1].v);
    s.frontRev = rr;
    if (s.throttle < 0.05f && s.brake < 0.02f && s.speed > 8.0f && rr > 0.5f)
        d.rollRatio.step(rr / s.speed, alphaFor(dt, 1.0f));
    const float ratio = d.rollRatio.init ? d.rollRatio.v : 1.0f / c.tyreCircumference;
    float lockT = 0;
    if (s.speed > 4.0f && ratio > 1e-3f && s.brake > 0.2f) {
        float slip = rr / (ratio * s.speed) - 1.0f;
        lockT = clamp01((-slip - c.lockThreshold) / 0.5f);
    }
    d.lockSm += (lockT - d.lockSm) * alphaFor(dt, lockT > d.lockSm ? 0.05f : 0.15f);
    s.lock = d.lockSm;
}

// -------------------------------------------------------------- synthesis
struct Synth {
    float sm[S_COUNT] = {};
    float surfW[Surf::C_COUNT] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float thrSm = 0, lope = 0, rpmPeak = 0, heat = 0;
    float jAge = 5.0f, evAmp = 0.0f;
    unsigned seenEvt = 0;
    float boostMul = 1.0f, boostAdd = 0.0f;
    float roadJit[2] = {1.0f, 1.0f}, ampJit[2] = {1.0f, 1.0f}, engJit = 1.0f, engHzJit = 1.0f, unevPhase = 0;
    double nextJit = 0;

    void Run(const Config& c, const Signals& s, double t, float dt, float userGain, int mag[S_COUNT], float hz[S_COUNT]) {
        thrSm += (s.throttle - thrSm) * alphaFor(dt, 0.12f);

        // engine
        rpmPeak = std::max(rpmPeak, s.rpm);
        const float rmax = s.rpmMaxTelem > 0 ? s.rpmMaxTelem : (c.rpmMax > 0 ? c.rpmMax : std::max(rpmPeak, c.rpmMaxInit));
        const float x = clamp01(s.rpm / rmax);
        const float fire = s.rpm / 60.0f * c.cylinders * 0.5f;
        const float fireMax = rmax / 60.0f * c.cylinders * 0.5f;
        const float k = fireMax > c.engineMaxHz ? c.engineMaxHz / fireMax : 1.0f;
        const float buzzW = smooth01(0.50f, 1.0f, x);
        const float grit = c.engineGrit * buzzW;
        hz[S_ENG1] = clampf(fire * k * (1.0f + 0.04f * grit * engHzJit), c.engineMinHz, c.engineMaxHz);

        const float idle = 1.0f - smooth01(0.08f, 0.45f, x);
        const float res = gauss(x, 0.55f, 0.08f) + 0.8f * gauss(x, 0.80f, 0.07f);
        const float buzz = smooth01(0.60f, 1.0f, x);
        const float A1 = clamp01(0.22f + 0.45f * idle + 0.35f * res + 0.30f * buzz);
        const float loadF = 0.45f + 0.55f * thrSm;
        const float comp = clampf(1.4f - 0.009f * (hz[S_ENG1] - c.engineMinHz), 0.55f, 1.4f);
        const float engAir = 1.0f - 0.35f * c.airborneFade * s.airborne;
        const float unevMod = 1.0f - 0.35f * c.unevenness * (0.5f + 0.5f * std::sin(unevPhase));
        float t1 = s.rpm > 200 ? c.engineMaxMag * clamp01(A1 * loadF * comp * unevMod * engAir * (1.0f - 0.45f * grit * (0.5f - 0.5f * engJit))) : 0.0f;
        if (s.rpm > 200 && x > 0.985f && thrSm > 0.8f) t1 *= (std::fmod(t, 0.08) < 0.04 ? 1.0f : 0.25f);

        lope += (rnd() - lope) * 0.05f;
        const float xover = smooth01(0.45f, 0.75f, x) * c.engineGrit;
        const float hzLump = clampf(fire * 0.5f, 7.0f, 42.0f);
        const float hzBeat = clampf(hz[S_ENG1] * 1.37f, c.engineMinHz, kHardMaxHz);
        hz[S_ENG2] = hzLump + (hzBeat - hzLump) * xover;
        unevPhase += 6.2831853f * hzLump * dt;
        if (unevPhase > 6.2831853f) unevPhase -= 6.2831853f;
        const float A2 = (0.15f + 0.85f * idle) * (0.5f + 0.5f * thrSm) * (1.0f + 0.25f * lope * (0.2f + idle))
                       + 0.55f * grit * (0.6f + 0.4f * thrSm);
        float t2 = s.rpm > 200 ? c.engineMaxMag * 0.8f * clamp01(A2 * c.lumpScale) * engAir : 0.0f;

        // road
        const float spN = clamp01(s.speed / c.maxSpeed);
        const float gSpd = smooth01(1.0f, 10.0f, s.speed);
        const float texMono = clamp01(s.texture + 0.9f * s.bump);
        const float wf = clampf(c.weightFeel, 0.5f, 2.0f);
        const float thump[2] = { s.thumpL, s.thumpR };
        const float sideTex[2] = { s.texL, s.texR };
        const float sideLoad[2] = { s.loadL, s.loadR };

        Surf::Feel fe = Surf::kNeutral;
        if (Surf::g_on) {
            const float aw = alphaFor(dt, Surf::g_fade);
            for (int i = 0; i < Surf::C_COUNT; ++i) surfW[i] += ((i == s.surf ? 1.0f : 0.0f) - surfW[i]) * aw;
            fe = Surf::Blend(surfW);
        }

        float texSide[2], loadGain[2];
        for (int k2 = 0; k2 < 2; ++k2) {
            const float tm = texMono + (clamp01(sideTex[k2] + 0.9f * s.bump) - texMono) * c.roadDirectional;
            texSide[k2] = clamp01(tm + c.thumpGain * wf * thump[k2]);
            texSide[k2] = std::max(texSide[k2], fe.floor);
            loadGain[k2] = 1.0f + c.loadInfluence * (clampf(sideLoad[k2], 0.4f, 1.8f) - 1.0f);
        }

        const float roadAir = 1.0f - c.airborneFade * s.airborne;
        if (t >= nextJit) {
            for (int k2 = 0; k2 < 2; ++k2) {
                const float tx = texSide[k2];
                const float gr = clamp01(c.roadGrain + fe.grainAdd);
                roadJit[k2] = 1.0f + (0.10f + 0.40f * gr * (tx > 0.3f ? 1.0f : 0.5f)) * rnd();
                const float lo = std::max(0.02f, 1.0f - 0.65f * gr - fe.dropout);
                ampJit[k2] = (lo + (1.0f - lo) * (rnd() * 0.5f + 0.5f)) * fe.base;
                if (fe.spark > 0.0f && rnd() * 0.5f + 0.5f < fe.spark) ampJit[k2] = fe.sparkMul;
            }
            engJit = rnd(); engHzJit = rnd();
            nextJit = t + 0.012;
        }

        float t3[2];
        for (int k2 = 0; k2 < 2; ++k2) {
            const float spd = c.roadMinHz + (c.roadMaxHz - c.roadMinHz) * clamp01(0.35f * spN + 0.65f * texSide[k2]);
            float hzR = spd * roadJit[k2] * (k2 ? 1.31f : 1.0f) / std::sqrt(wf);
            if (fe.hz != 1.0f) hzR = clampf(hzR * fe.hz, 20.0f, kHardMaxHz);
            hz[k2 == 0 ? S_ROAD : S_ROADR] = hzR;
            t3[k2] = c.roadMaxMag * 0.7f * texSide[k2] * gSpd * ampJit[k2] * loadGain[k2] * roadAir * fe.amp;
        }

        const int sA = c.roadSideInvert ? 1 : 0;
        const float tRoadA = t3[sA], tRoadB = t3[sB];
        { const float hzA = hz[sA == 0 ? S_ROAD : S_ROADR], hzB = hz[sB == 0 ? S_ROAD : S_ROADR];
          hz[S_ROAD] = hzA; hz[S_ROADR] = hzB; }

        // brake
        const float gB = smooth01(2.0f, 8.0f, s.speed);
        const float Ab = clamp01(std::pow(s.brake, 1.2f) * (0.3f + 0.7f * s.dive) * 0.6f + s.lock);
        hz[S_BRAKE] = s.lock > 0.1f ? clampf(38.0f + 25.0f * rnd(), 25.0f, 70.0f) : clampf(s.frontRev * 2.0f, 12.0f, 60.0f);
        hz[S_BRAKE] *= c.brakeHzScale;
        float t4 = c.brakeMaxMag * Ab * gB * clampf(0.65f + 0.35f * s.loadF, 0.5f, 1.3f);

        // landing boost
        if (s.evtSeq != seenEvt) { seenEvt = s.evtSeq; jAge = 0.0f; evAmp = s.evtSev; }
        if (jAge < 0.08f) evAmp = std::max(evAmp, s.evtSev);
        jAge = std::min(jAge + dt, 5.0f);
        {
            const float ts = std::sqrt(wf);
            const float hold = 0.06f * ts, dec = 0.30f * ts;
            const float env = jAge < hold ? 1.0f : (float)std::exp(-(jAge - hold) / dec);
            const float lev = (c.fxLandBoost && jAge < 1.5f) ? clamp01(evAmp) * env * clampf(userGain, 0.0f, 1.0f) : 0.0f;
            boostMul = 1.0f + c.landingBoost * lev;
            boostAdd = c.landingAdd * lev;
        }

        auto follow = [&](int i, float target, float up, float down) {
            sm[i] += (target - sm[i]) * (target > sm[i] ? up : down);
        };
        if (!c.fxEngine) { t1 = 0.0f; t2 = 0.0f; }
        follow(S_ENG1, t1, 0.35f, 0.15f);
        follow(S_ENG2, t2, 0.25f, 0.12f);
        const float roadDown = clampf(0.40f / wf, 0.15f, 0.70f);
        const float roadRel = clampf(roadDown * fe.rel, 0.05f, 0.95f);
        follow(S_ROAD,  c.fxRoad ? tRoadA : 0.0f, fe.atk, roadRel);
        follow(S_ROADR, c.fxRoad ? tRoadB : 0.0f, fe.atk, roadRel);
        follow(S_BRAKE, c.fxBrake ? t4 : 0.0f, 0.50f, 0.20f);

        const float pR = c.roadMaxMag > 0 ? clamp01(std::max(std::fabs(sm[S_ROAD]), std::fabs(sm[S_ROADR])) / (float)c.roadMaxMag) : 0.0f;
        const float dk = clampf(c.priorityDuck, 0.0f, 1.0f);
        float duck[S_COUNT];
        duck[S_ROAD] = duck[S_ROADR] = duck[S_BRAKE] = 1.0f;
        duck[S_ENG1] = duck[S_ENG2] = 1.0f - dk * pR;
        float sumO = 0;
        for (int i = 0; i < S_COUNT; ++i) sumO += std::fabs(sm[i]) * duck[i];
        const float room = (float)c.totalMaxMag;
        const float scale = sumO > room && sumO > 0.0f ? room / sumO : 1.0f;
        const float sum = sumO * scale;
        const float ug = clampf(userGain, 0.0f, 1.0f);
        const float lvl = std::min(sum, (float)kHardTotalMax) / 10000.0f * ug;
        heat += (lvl * lvl - heat) * alphaFor(dt, 20.0f);
        const float derate = heat > c.sustainedLimit ? std::sqrt(c.sustainedLimit / heat) : 1.0f;

        const float g = ug * derate;
        for (int i = 0; i < S_COUNT; ++i) {
            float m = sm[i] * duck[i] * scale * g;
            if (!std::isfinite(m)) { m = 0.0f; sm[i] = 0.0f; }
            mag[i] = std::max(0, std::min((int)m, kHardTotalMax));
        }
    }
};

// ----------------------------------------------------------------- plugin
// TEMPORARY memory scanner removed; stable build only keeps Surf and final force-feedback logic.

void ForceFXPlugin::StageStarted(int, const char*, bool) {
    m_stageReset = true;
    m_carRescan = true;
    Surf::OnStage();
}

// Remove any lingering MemScan references from worker loop / config / status / menu if present.
// That branch is no longer used.
// Example patterns to delete from the file if still present:
//
//   MemScan::OnStage();
//   MemScan::Start(...);
//   MemScan::Stop(...);
//   MemScan::Status(...);
//   g_scan / g_reader usage
//   ForceFX.ini [MemScan] settings block
//   MemScan section in comments / docs
//
// End of stable build.
