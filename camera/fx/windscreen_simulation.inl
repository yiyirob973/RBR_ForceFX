// Part of camera_fx.hpp: included from inside its anonymous namespace (ffx::cam), in the order listed there. Not a header on its own.
// ---- simulation ----
void wsInsert(const WsDrop& n, int cap) {
    int count = 0, oi = -1; float oa = -1.f;
    for (int i = 0; i < (int)g_wsDrops.size(); ++i) {
        const WsDrop& o = g_wsDrops[i]; if (o.kind != n.kind) continue;
        ++count;
        if (o.age > oa && std::fabs(o.vx) + std::fabs(o.vy) < 0.02f) { oa = o.age; oi = i; }
    }
    if (count >= cap) { if (oi >= 0) g_wsDrops[oi] = n; }              // glass is full: the oldest standing one gives way
    else g_wsDrops.push_back(n);
}
void wsAddWater(float x, float y, float r, float age, int cap) {
    WsDrop d{}; d.kind = 0; d.x = x; d.y = y; d.r = r; d.age = age; d.tx = x; d.ty = y; d.tr = r; d.rot = wsRange(0.f, 6.2832f);   // rot = meander phase, and the pool orientation
    d.asp = wsRange(0.8f, 1.3f); d.var = (int)(wsRnd() * 4.f) & 3;
    const float rn = r / std::max(0.2f, g_cfg.wsDropSize);                                        // grip: small beads cling, medium ones let go sooner, flat pools cling again
    d.thr = (rn < 0.0055f ? 0.52f : rn < 0.0115f ? 0.52f - (rn - 0.0055f) / 0.006f * 0.22f : 0.55f) + wsRange(-0.04f, 0.04f);
    wsInsert(d, cap);
}
void wsAddMicro(float x, float y, float r, int cap) {                    // tiny droplets: they stick, never run, and are cleaned by wipers and by bigger drops running over them
    WsDrop d{}; d.kind = 2; d.x = x; d.y = y; d.r = r; d.age = 1.f; d.melt0 = d.melt = wsRange(70.f, 160.f);
    wsInsert(d, cap);
}
void wsAddFlake(float x, float y, float r, int cap) {
    WsDrop d{}; d.kind = 1; d.x = x; d.y = y; d.r = r; d.age = 0.f; d.tx = x; d.ty = y; d.tr = r; d.rot = wsRange(0.f, 6.2832f);
    d.thr = wsRange(0.9f, 1.3f);                                       // snow sticks much harder than water
    d.melt0 = d.melt = std::max(1.f, g_cfg.wsSnowMelt * wsRange(0.7f, 1.3f));
    wsInsert(d, cap);
}
// Particles in flight (see WsFly). wsFlyPos gives the position and the size scale (0..1) at progress p.
float wsPersp(float p, float zFar) { const float z = zFar + (1.f - zFar) * p; return (1.f / z - 1.f / zFar) / (1.f - 1.f / zFar); }          // 0 far .. 1 at the glass; accelerates
void wsFlyPos(const WsFly& f, float p, float& x, float& y, float& sc) {
    p = clamp01(p);
    if (f.mode == 1) {                                                 // thrown up from below: quick, slowing, growing
        const float e = 1.f - (1.f - p) * (1.f - p), bend = 4.f * e * (1.f - e) * f.arc;
        x = f.x0 + (f.x1 - f.x0) * e + bend; y = f.y0 + (f.y1 - f.y0) * e; sc = 0.30f + 0.70f * e;
    } else {                                                           // perspective approach from far away: slow at first, fast at the end
        const float zFar = f.kind == 1 ? 7.f : 12.f, e = wsPersp(p, zFar);
        x = f.x0 + (f.x1 - f.x0) * e; y = f.y0 + (f.y1 - f.y0) * e; sc = 1.f / (zFar + (1.f - zFar) * p);
    }
}
void wsMudLand(const WsFly& f) {                                       // a clod (or spray droplet) hits the glass: it flattens into a splat, satellites fly out of it
    WsMud m{}; m.x = f.x1; m.y = f.y1; m.r = f.r; m.rot = wsRange(0.f, 6.2832f); m.fade = 1.f; m.stretch = 1.f; m.variant = f.variant; m.tint = f.tint; m.shade = f.shade; m.dur = 0.f;
    g_wsMud.push_back(m);
    if (g_cfg.wsGround) g_wsDirtCol = wsl::mixRgb(g_wsDirtCol, wsl::tintColour(f.tint, wsMudRgb()), f.kind == 3 ? 0.04f : 0.12f);
    if (f.kind == 3) { wsGridAdd(g_wsDirt, f.x1, f.y1, 2.2f, 0.10f); return; }
    const int n = std::min(14, 2 + (int)(f.r * 170.f));
    if (f.r > 0.014f) for (int i = 0; i < n; ++i) {
        const float a = wsRange(0.f, 6.2832f), dist = f.r * wsRange(1.5f, 3.4f);
        WsMud s{}; s.ox = f.x1; s.oy = f.y1; s.x = f.x1 + std::cos(a) * dist / g_wsAspect; s.y = f.y1 + std::sin(a) * dist;
        s.r = f.r * wsRange(0.10f, 0.30f); s.rot = wsRange(0.f, 6.2832f); s.fade = 1.f; s.stretch = 1.f; s.shade = f.shade * wsRange(0.9f, 1.1f);
        s.variant = (wsRnd() < 0.5f) ? kMudSpray : kMudSpray2; s.tint = f.tint; s.dur = wsRange(0.05f, 0.12f); g_wsMud.push_back(s);
    }
    wsGridAdd(g_wsDirt, f.x1, f.y1, 2.5f + f.r * 110.f, 0.35f + f.r * 12.f);        // the glass behind and around it gets hazy
}
void wsMudThrow(float power, int tint, bool spray) {                   // a burst of clods (or fine dirty spray) heads for the glass: off your own wheels, or thrown by a car ahead
    const float sz = g_cfg.wsMudSize;
    const bool own = wsRnd() < 0.6f;
    float cx, cy, ox, oy;
    if (own) { const float side = wsRnd() < 0.5f ? 0.27f : 0.73f; cx = std::max(0.05f, std::min(0.95f, side + wsRange(-0.22f, 0.22f))); cy = wsRange(0.42f, 0.84f); ox = side + wsRange(-0.05f, 0.05f); oy = 1.03f; }
    else     { cx = wsRange(0.12f, 0.88f); cy = wsRange(0.22f, 0.75f); ox = 0.5f + wsRange(-0.08f, 0.08f); oy = 0.42f + wsRange(-0.04f, 0.04f); }
    const int n = spray ? 7 + (int)(wsRnd() * 8.f * power) : 3 + (int)(wsRnd() * 5.f * power);
    const float baseDur = own ? 0.30f : 0.45f, spread = own ? 0.11f : 0.15f;
    for (int i = 0; i < n; ++i) {
        const bool lead = !spray && i == 0;
        WsFly f{}; f.kind = spray ? 3 : 1; f.mode = own ? 1 : 0; f.tint = tint; f.shade = wsRange(0.85f, 1.15f);
        f.r = (spray ? wsRange(0.0035f, 0.010f) : lead ? wsRange(0.032f, 0.062f) * std::sqrt(power) : wsRange(0.007f, 0.022f)) * sz;
        f.variant = lead ? ((int)(wsRnd() * 4.f) & 3) : spray ? kMudSpeckle : ((wsRnd() < 0.5f) ? kMudSpray : kMudSpray2);
        if (!lead && !spray && f.r > 0.016f) f.variant = (int)(wsRnd() * 4.f) & 3;                     // mid-size clods splat too
        f.x1 = std::max(0.02f, std::min(0.98f, cx + (wsRange(-1.f, 1.f) + wsRange(-1.f, 1.f)) * 0.5f * spread / g_wsAspect * 1.6f));
        f.y1 = std::max(0.05f, std::min(0.90f, cy + (wsRange(-1.f, 1.f) + wsRange(-1.f, 1.f)) * 0.5f * spread));
        f.x0 = ox + wsRange(-0.03f, 0.03f); f.y0 = oy + wsRange(-0.02f, 0.02f); f.arc = wsRange(-0.06f, 0.06f);
        f.dur = baseDur * wsRange(0.8f, 1.25f); f.t = -wsRange(0.f, 0.20f);
        g_wsFly.push_back(f);
    }
}
void wsWaterSplash(float power) {                                     // a tyre hit standing water: a wash of clear water is thrown up at the glass, blurs it and runs until the wipers clear it
    power = std::max(0.f, std::min(1.f, power));
    const int n = 10 + (int)(power * 34.f);
    for (int i = 0; i < n; ++i) {
        const float side = wsRnd() < 0.5f ? 0.27f : 0.73f;
        WsFly f{}; f.kind = 0; f.mode = 1; f.shade = 1.f; f.variant = 1;                                         // variant 1: lands with a wide wash of film, not a single bead
        f.r = (wsRnd() < 0.7f ? wsRange(0.003f, 0.009f) : wsRange(0.009f, 0.020f)) * (0.7f + 0.5f * power) * g_cfg.wsDropSize;
        f.x1 = std::max(0.03f, std::min(0.97f, side + (wsRange(-1.f, 1.f) + wsRange(-1.f, 1.f)) * 0.30f / g_wsAspect * 1.6f));
        f.y1 = 0.12f + 0.72f * std::pow(wsRnd(), 0.8f);
        f.x0 = side + wsRange(-0.05f, 0.05f); f.y0 = 1.03f; f.arc = wsRange(-0.06f, 0.06f);
        f.dur = wsRange(0.22f, 0.40f); f.t = -wsRange(0.f, 0.25f);
        g_wsFly.push_back(f);
    }
}
void wsFlyUpdate(float dt) {                                           // advance everything in flight, land what has arrived
    for (WsFly& f : g_wsFly) {
        f.t += dt / std::max(0.05f, f.dur);
        if (f.t < 1.f) continue;
        switch (f.kind) {
            case 0: wsAddWater(f.x1, f.y1, f.r, 0.f, g_cfg.wsDrops); if (f.variant == 1) wsGridAdd(g_wsFilm, f.x1, f.y1, 6.f, 0.55f); else wsGridAdd(g_wsFilm, f.x1, f.y1, 3.5f, (f.r < 0.0055f ? 0.30f : 0.55f) * (0.25f + 0.75f * g_wsRainEff * g_wsRainEff)); break;   // a splash lands as a wash, rain as a bead
            case 2: wsAddFlake(f.x1, f.y1, f.r, g_cfg.wsSnowMax); wsGridAdd(g_wsFilm, f.x1, f.y1, 4.f, 1.0f); break;
            default: wsMudLand(f); break;
        }
        f.kind = -1;
    }
    g_wsFly.erase(std::remove_if(g_wsFly.begin(), g_wsFly.end(), [](const WsFly& f) { return f.kind < 0; }), g_wsFly.end());
    if (g_wsFly.size() > 3000) g_wsFly.erase(g_wsFly.begin(), g_wsFly.begin() + (g_wsFly.size() - 3000));
}

void wsSync() {                                                        // match the particle sets to the INI
    const bool wantD = (g_cfg.wsMode & 1) != 0, wantM = (g_cfg.wsMode & 2) != 0;
    if (g_wsReset) {                                                   // new stage: clean glass, rain and mud arrive from here on
        g_wsDrops.clear(); g_wsMud.clear(); g_wsTrails.clear(); g_wsFly.clear(); std::fill(g_wsFilm, g_wsFilm + kGX * kGY, 0.f); std::fill(g_wsDirt, g_wsDirt + kGX * kGY, 0.f);
        g_wsBow[0] = g_wsBow[1] = 0.f; g_wsRainAcc = g_wsSnowAcc = g_wsMudAcc = g_wsMicroAcc = 0.f; g_wsSprayAcc = 0.f; g_wsReset = false; g_wsWipeT0 = GetTickCount(); wsHuntReset(); g_wsDirtCol = wsMudRgb(); g_wsGround = wsl::Ground(); g_wsGroundTick = 0; g_wsGroundSeen = false;
        g_wsRng = (g_wsRng ^ (uint32_t)GetTickCount()) | 1u;
    }
    if (g_wsResetWater) {                                              // next stage of the same rally: the water has dried, the mud is still there
        g_wsResetWater = false;
        g_wsDrops.clear(); g_wsTrails.clear(); g_wsFly.clear(); std::fill(g_wsFilm, g_wsFilm + kGX * kGY, 0.f);
        g_wsBow[0] = g_wsBow[1] = 0.f; g_wsRainAcc = g_wsSnowAcc = g_wsMudAcc = g_wsMicroAcc = 0.f; g_wsSprayAcc = 0.f; g_wsWipeT0 = GetTickCount(); wsHuntReset();
        g_wsGround = wsl::Ground(); g_wsGroundTick = 0; g_wsGroundSeen = false;
        g_wsRng = (g_wsRng ^ (uint32_t)GetTickCount()) | 1u;
    }
    if (!wantD) { g_wsDrops.clear(); g_wsTrails.clear(); std::fill(g_wsFilm, g_wsFilm + kGX * kGY, 0.f); }
    if (!wantM) { g_wsMud.clear(); std::fill(g_wsDirt, g_wsDirt + kGX * kGY, 0.f); }
}

// Wipers (emulated): the game's wiper pose is not exposed anywhere known, so two blades sweep on a timed cycle that you line up with the real ones
// (geometry in the ini, Ctrl+F7 to sync the timing, WiperDebug=1 to see them). Pose = pivot + length + angle range in glass space.
struct WsBlade { float px, py, len, a0, a1; };
void wsBlades(WsBlade (&b)[2]) {
    b[0] = {g_cfg.wsW1X, g_cfg.wsW1Y, g_cfg.wsW1Len, g_cfg.wsW1A0, g_cfg.wsW1A1};
    b[1] = {g_cfg.wsW2X, g_cfg.wsW2Y, g_cfg.wsW2Len, g_cfg.wsW2A0, g_cfg.wsW2A1};
    for (int i = 0; i < 2; ++i)                                        // WiperSource=1 and the hunter has found the real blade: its pose replaces the ini geometry
        if (g_cfg.wsWiperSource && g_wsModelBlade[i].ok) { b[i].px = g_wsModelBlade[i].px; b[i].py = g_wsModelBlade[i].py; b[i].len = g_wsModelBlade[i].len; }
}
void wsWiperUpdate() {
    WsBlade bl[2]; wsBlades(bl);
    const bool wet = g_wsRainEff > 0.02f || g_wsSnowEff > 0.02f;
    const bool active = g_cfg.wsWipers && (!g_cfg.wsWiperAuto || wet);
    float w = 0.f;
    if (active) {
        const float period = std::max(0.3f, g_cfg.wsWiperPeriod), cyc = period + std::max(0.f, g_cfg.wsWiperPause);
        const float t = std::fmod((float)(GetTickCount() - g_wsWipeT0) / 1000.f, cyc);
        if (t < period) w = 0.5f - 0.5f * std::cos(6.2832f * t / period);       // up and back, smooth at both ends
    }
    g_wsWiperW = w;
    for (int i = 0; i < 2; ++i) {
        float ang = (bl[i].a0 + (bl[i].a1 - bl[i].a0) * w) * 0.0174533f;
        if (g_cfg.wsWiperSource && g_wsModelBlade[i].ok) {            // the real blade: follow its angle, kept continuous across +-180 degrees
            const float prev = g_wsBladeInit ? g_wsBladeAng[i] : g_wsModelBlade[i].ang, d = g_wsModelBlade[i].ang - prev;
            ang = prev + std::atan2(std::sin(d), std::cos(d));
        }
        g_wsBladePrev[i] = g_wsBladeInit ? g_wsBladeAng[i] : ang; g_wsBladeAng[i] = ang;
    }
    g_wsBladeInit = true;
}
void wsWiperGhost(float dt) {                                          // lagging copies of each blade angle: the motion-blur wedge is drawn between them and the blade
    for (int i = 0; i < 2; ++i) {
        const float a = g_wsBladeAng[i];
        for (int k = 0; k < 2; ++k) {
            const float tau = k == 0 ? 0.035f : 0.080f; float& g = g_wsGhost[i][k];
            if (std::fabs(g - a) > 1.0f) g = a; else g += (a - g) * (1.f - std::exp(-dt / tau));
        }
    }
}
void wsSyncWipers() { g_wsWipeT0 = GetTickCount(); g_wsBladeInit = false; Logf("wiper cycle synced to now (Ctrl+F7): the real wipers should be starting their upward stroke"); }

// A thick film does not sit still: where the water is deep it sags and runs down the glass in streams, each column wandering a cell either way for a
// moment, and what reaches the bottom edge drains off. Ticks eight times a second, so a sheet takes a few seconds to run down the screen.
float g_wsFlowAcc = 0.f, g_wsFlowT = 0.f;
void wsFilmFlow(float dt, float rain) {
    g_wsFlowAcc += dt * 8.f; g_wsFlowT += dt;
    for (int guard = 0; g_wsFlowAcc >= 1.f && guard < 4; ++guard) {
        g_wsFlowAcc -= 1.f;
        const float share = std::min(0.5f, 0.25f + 0.25f * rain);     // of what is above the threshold, per tick
        for (int j = kGY - 1; j >= 0; --j) for (int i = 0; i < kGX; ++i) {
            float& f = g_wsFilm[j * kGX + i];
            if (f < 0.40f) continue;
            const float give = (f - 0.40f) * share;
            f -= give;
            if (j + 1 >= kGY) continue;                                // the bottom edge: it drains away
            const unsigned h = (unsigned)i * 73856093u ^ (unsigned)((int)(g_wsFlowT * 0.6f)) * 19349663u;
            const int ti = std::max(0, std::min(kGX - 1, i + (int)(h % 3u) - 1));
            float& below = g_wsFilm[(j + 1) * kGX + ti]; below = std::min(1.f, below + give);
        }
    }
}

void wsWipe(float W, float H) {
    if (!g_cfg.wsWipers) return;
    WsBlade bl[2]; wsBlades(bl);
    std::vector<WsDrop> spawn; std::vector<WsMud> smears;
    const float life = std::max(0.5f, g_cfg.wsTrail);
    for (int bi = 0; bi < 2; ++bi) {
        // Where the rubber is now and where it was in the last frame, as two end points each. Whatever lies between those two positions was wiped,
        // so the real blade's length, the rake of the glass and the perspective are all already in the numbers.
        wsl::Seg cur{}, prev{}; bool have = false;
        if (g_cfg.wsWiperSource && g_wsModelBlade[bi].ok) {
            cur = g_wsModelBlade[bi].seg; prev = g_wsSegPrev[bi]; have = g_wsSegHave[bi];
            g_wsSegPrev[bi] = cur; g_wsSegHave[bi] = true;
        } else {                                                       // the emulated blade from the ini: a straight arm turning about its pivot
            g_wsSegHave[bi] = false;
            if (bl[bi].len <= 0.f) continue;
            const float pxv = bl[bi].px * W, pyv = bl[bi].py * H, L = bl[bi].len * H, in = std::max(0.f, std::min(0.9f, g_cfg.wsWiperInner));
            auto at = [&](float a) { const float c = std::cos(a), sn = -std::sin(a); return wsl::Seg{pxv + c * L * in, pyv + sn * L * in, pxv + c * L, pyv + sn * L}; };
            cur = at(g_wsBladeAng[bi]); prev = at(g_wsBladePrev[bi]); have = true;
        }
        if (!have || wsl::bladeMotion(prev, cur, 0.35f * W) != 1) continue;     // parked, or a jump (settings reloaded, a different blade): no sweep this frame
        const float half = 3.f + 0.004f * H;                           // blade half thickness in pixels
        float mass = 0.f;                                              // water gathered by the blade this frame (feeds the ridge in front of it)
        auto swept = [&](float X, float Y, float R, float& tx, float& ty) -> bool { return wsl::sweptBy(prev, cur, X, Y, R, half, tx, ty); };   // (tx, ty): the way the blade moved there
        // A wiper does not erase water, it moves it. A bead in the blade's path is pushed along in front of the rubber (a few slip under it and stay), the
        // beads it meets on the way run together into bigger ones, and a little is always left behind as a smear and a few small beads. What is
        // pushed off the edge of the glass is gone. Micro droplets and flakes are too small to hold together and are simply smeared away.
        const float asp = H / W;                                       // viewport height per viewport width: radii are in heights, positions in widths
        std::vector<size_t> pushed;
        for (size_t di = 0; di < g_wsDrops.size(); ++di) {
            WsDrop& d = g_wsDrops[di];
            if (d.dead) continue;
            float tx, ty; if (!swept(d.x * W, d.y * H, d.r * H, tx, ty)) continue;
            g_wsWiped++; mass += (d.kind == 2 ? 0.12f : 1.f) * std::min(1.f, d.r * 90.f);
            if (d.kind != 0) { d.dead = true; continue; }
            if (wsRnd() < 0.06f) continue;                             // slipped under the blade: the bead stays where it is
            const float ox = d.x, oy = d.y;
            d.x += tx * 1.15f / W; d.y += ty * 1.15f / H;              // rides a little ahead of the rubber
            d.tx = d.x; d.ty = d.y; d.tr = d.r; d.vx = d.vy = 0.f; d.sliding = false;   // pushed, not running: no trail from the push
            if (d.r > 0.004f) {
                wsGridAdd(g_wsFilm, ox, oy, 2.f, 0.10f);                // the smear the rubber leaves
                d.r *= 0.985f;
                if (wsRnd() < 0.04f) {                                 // and now and then a small bead left behind (the bead is pushed every frame of the stroke, so this is per frame)
                    WsDrop n{}; n.kind = 0; n.x = ox; n.y = oy; n.r = wsRange(0.0016f, 0.0028f) * g_cfg.wsDropSize; n.age = 2.f; n.tx = n.x; n.ty = n.y; n.tr = n.r; n.thr = 0.55f;
                    spawn.push_back(n);
                }
            }
            pushed.push_back(di);
        }
        for (size_t a = 0; a < pushed.size(); ++a) {                    // beads pushed together run into one
            WsDrop& pa = g_wsDrops[pushed[a]]; if (pa.dead) continue;
            for (size_t b = a + 1; b < pushed.size(); ++b) {
                WsDrop& pb = g_wsDrops[pushed[b]]; if (pb.dead) continue;
                const float ex = (pb.x - pa.x) / asp, ey = pb.y - pa.y;
                if (std::sqrt(ex * ex + ey * ey) < (pa.r + pb.r) * 0.9f) { pa.r = std::sqrt(pa.r * pa.r + pb.r * pb.r); pb.dead = true; }
            }
        }
        for (WsTrail& t : g_wsTrails) { float tx, ty; if (swept((t.x0 + t.x1) * 0.5f * W, (t.y0 + t.y1) * 0.5f * H, t.w0 * H, tx, ty)) { t.age = life + 1.f; g_wsWiped++; } }
        for (WsMud& m : g_wsMud) {
            if (m.age < 0.f || m.fade <= 0.f) continue;
            float tx, ty; if (!swept(m.x * W, m.y * H, m.r * H * m.stretch, tx, ty)) continue;
            m.fade *= 0.55f; m.r *= 0.94f; g_wsWiped++; mass += 0.4f;                // mostly cleared, the rest is dragged along the stroke
            if (m.variant != kMudSmear && !wsIsSprayV(m.variant) && m.r > 0.008f && m.fade > 0.15f && g_cfg.wsMudSmear) {
                WsMud s{}; s.x = m.x; s.y = m.y; s.r = m.r * 0.9f; s.variant = kMudSmear; s.stretch = 2.6f; s.tint = m.tint; s.fade = 0.85f * m.fade; s.age = 0.f; s.shade = m.shade;
                s.rot = std::atan2(-tx, ty); smears.push_back(s);
            }
        }
        const float cw = W / (float)kGX, ch = H / (float)kGY, cr = 0.75f * std::max(cw, ch);   // the haze grids: water is pushed along by the blade, dirt is mostly smeared (some stays)
        std::vector<float> filmAdd((size_t)kGX * kGY, 0.f);            // where the pushed water lands: added after the pass, so it is not pushed twice in one frame
        for (int j = 0; j < kGY; ++j) for (int i = 0; i < kGX; ++i) {
            float tx, ty; if (!swept(((float)i + 0.5f) * cw, ((float)j + 0.5f) * ch, cr, tx, ty)) continue;
            float& f = g_wsFilm[j * kGX + i]; float& dt = g_wsDirt[j * kGX + i];
            const float wetNow = std::max(f, g_wsRainEff > 0.02f ? 0.6f : 0.f);
            const float resid = g_wsRainEff > 0.02f ? 0.10f + 0.16f * g_wsRainEff : 0.f;   // a real blade never leaves it bone dry in the rain: a thin film stays, and fine droplets are left along the stroke
            const float len = std::sqrt(tx * tx + ty * ty);
            if (f > 0.01f && len > 0.01f) {                            // most of the water moves the way the blade moves, not all of it and not evenly
                const float step = std::max(1.f, std::min(3.f, len / std::max(cw, ch) + 0.8f));
                const int ti = i + (int)std::lround(tx / len * step), tj = j + (int)std::lround(ty / len * step);
                const float carry = f * (0.70f + 0.25f * wsRnd());
                if (ti >= 0 && ti < kGX && tj >= 0 && tj < kGY) filmAdd[(size_t)tj * kGX + ti] += carry;   // past the edge of the glass it is gone
                mass += carry * 0.05f;
                f = std::max(f - carry, std::min(f, resid));
            }
            dt *= 0.62f - 0.30f * clamp01(wetNow);                      // a wet blade lifts dirt, a dry one mostly smears it
            if (g_cfg.wsMicro > 0 && g_wsRainEff > 0.05f && wsRnd() < 0.06f * g_wsRainEff) wsAddMicro(((float)i + wsRange(0.f, 1.f)) / (float)kGX, ((float)j + wsRange(0.f, 1.f)) / (float)kGY, wsRange(0.0008f, 0.0022f) * g_cfg.wsDropSize, g_cfg.wsMicro);
        }
        for (int k = 0; k < kGX * kGY; ++k) if (filmAdd[(size_t)k] > 0.f) g_wsFilm[k] = std::min(1.f, g_wsFilm[k] + filmAdd[(size_t)k]);
        if (g_cfg.wsWiperBow) g_wsBow[bi] = std::min(1.5f, g_wsBow[bi] + mass * 0.045f);
    }
    for (const WsDrop& n : spawn) wsInsert(n, g_cfg.wsDrops);
    for (const WsMud& s : smears) g_wsMud.push_back(s);
}

// The surface scan says what is under each tyre. Read it ten times a second and smooth it, so one tyre on a verge does not flip the glass.
void wsGroundUpdate(float dt) {
    if (!g_cfg.wsGround || !g_hostIds) { g_wsGround = wsl::Ground(); return; }
    const DWORD now = GetTickCount();
    if (now - g_wsGroundTick >= 100) {
        g_wsGroundTick = now;
        int ids[4] = {-1, -1, -1, -1};
        if (g_hostIds(ids)) {
            const bool changed = memcmp(ids, g_wsGroundIds, sizeof(ids)) != 0;
            memcpy(g_wsGroundIds, ids, sizeof(ids)); g_wsGroundSeen = true; g_wsGroundGood = now;
            static DWORD lastLog = 0;
            if (changed && now - lastLog > 1500) {
                lastLog = now;
                const wsl::Ground t = wsl::groundFromIds(ids, g_wsMat, wsl::GroundCfg{g_cfg.wsMudDry, g_cfg.wsMudDamp, g_cfg.wsMudWet, g_cfg.wsMudSnow});
                Logf("windscreen ground: FL %s | FR %s | RL %s | RR %s  ->  loose %.2f wetMul %.2f spray %.2f dust %.2f snow %.2f", g_wsMat.name(ids[0]), g_wsMat.name(ids[1]), g_wsMat.name(ids[2]), g_wsMat.name(ids[3]),
                     t.loose, t.mudMul, t.spray, t.dust, t.snow);
            }
        } else if (now - g_wsGroundGood > 1500) g_wsGroundSeen = false;        // the scan has not had all four tyres for a while
    }
    if (!g_wsGroundSeen) { g_wsGround = wsl::Ground(); return; }          // no scan yet: the heave-based estimate and the stage's wetness are used instead
    const wsl::Ground t = wsl::groundFromIds(g_wsGroundIds, g_wsMat, wsl::GroundCfg{g_cfg.wsMudDry, g_cfg.wsMudDamp, g_cfg.wsMudWet, g_cfg.wsMudSnow});
    wsl::smoothGround(g_wsGround, t, 1.f - std::exp(-dt / 0.5f));
}

void wsStep(float dt, float W, float H) {
    const float aspect = H / W; g_wsAspect = W / H;
    // telemetry -> glass conditions: speed (km/h) is the airflow, the high-passed heave is road roughness (loose surface), sway = cornering
    const float spd = g_in.speed.load();
    const float spdN = clamp01(spd / std::max(10.f, g_cfg.wsSpeedRef));
    g_wsRough += (std::fabs(g_in.heaveHp.load()) - g_wsRough) * std::min(1.f, dt * 1.5f);
    const float loose = clamp01(g_wsRough / std::max(0.01f, g_cfg.wsRoughRef));
    g_wsSpdN = spdN; g_wsLoose = loose;
    const float wind = spdN * g_cfg.wsWind;
    static DWORD lastGW = 0;
    if (GetTickCount() - lastGW > 250) { lastGW = GetTickCount(); g_wsGW = wsReadGameWeather(); }
    const bool useGame = g_cfg.wsRainSource && g_wsGW.ok;
    if (g_wsRuleRain >= 0.f) g_wsRainEff = g_wsRuleRain;                       // explicit override for this stage
    else if (useGame) g_wsRainEff = (g_wsGW.sky == 3) ? g_cfg.wsRainLight : (g_wsGW.sky == 4) ? g_cfg.wsRainHeavy : 0.f;   // the game's own sky type
    else g_wsRainEff = g_cfg.wsRainLevel;
    g_wsSnowEff = useGame ? ((g_wsGW.sky == 6) ? g_cfg.wsSnowLight : (g_wsGW.sky == 7) ? g_cfg.wsSnowHeavy : 0.f) : g_cfg.wsSnowLevel;
    const bool snowy = g_wsSnowEff > 0.01f;
    const float wetMul = useGame ? (snowy ? g_cfg.wsMudSnow : g_wsGW.wet == 0 ? g_cfg.wsMudDry : g_wsGW.wet == 1 ? g_cfg.wsMudDamp : g_cfg.wsMudWet) : 1.f;   // mud needs a wet road
    if (g_cfg.wsWetFromGame) {                                         // experiment: is the game drawing its own wet overlay right now?
        const bool wetNow = (GetTickCount() - g_wsGameTick.load()) < 300;
        g_wsWet += ((wetNow ? 1.f : 0.f) - g_wsWet) * std::min(1.f, dt * 0.5f);
    } else g_wsWet = 1.f;
    if (g_hostCarKey && (g_wsLastCar.empty() || GetTickCount() < g_wsCarCheckUntil)) {   // a different car starts with a clean windscreen
        const std::string k = g_hostCarKey();
        if (!k.empty() && k != g_wsLastCar) {
            if (wsl::carChanged(g_wsLastCar, k)) { g_wsReset = true; Logf("windscreen: a different car, glass cleaned"); }
            g_wsLastCar = k;
        }
    }
    wsGroundUpdate(dt);
    const bool gnd = g_cfg.wsGround && g_wsGround.ok;                   // the glass follows what the tyres are on
    wsHuntPose(W, H);
    wsWiperUpdate();

    // ---- arrivals ----
    if ((g_cfg.wsMode & 1) && g_cfg.wsDrops > 0) {                     // rain impacts, more of them the faster you drive
        const float rainRate = g_cfg.wsRain * g_wsRainEff * (0.2f + 0.8f * std::pow(spdN, 0.8f)) * g_wsWet;
        g_wsDbgRain = rainRate; g_wsRainAcc += rainRate * dt;
        for (int guard = 0; g_wsRainAcc >= 1.f && guard < 50; ++guard) {
            g_wsRainAcc -= 1.f;
            const float u = wsRnd();                                                         // many small beads, some medium, a few big flat pools
            const float pl = 0.03f + 0.10f * g_wsRainEff;                                  // a few big drops, more of them in heavier rain
            float r = u < 0.62f ? wsRange(0.0016f, 0.0050f) : u < 1.f - pl ? wsRange(0.0050f, 0.0105f) : wsRange(0.0105f, 0.0165f + 0.005f * g_wsRainEff);
            const float tx = wsRange(0.03f, 0.97f), ty = 0.06f + 0.74f * std::pow(wsRnd(), 0.85f), rr = r * g_cfg.wsDropSize;
            if (g_cfg.wsFly && rr >= 0.0032f) {                                            // it comes from somewhere: from above when you are slow, from the vanishing point when you are fast
                WsFly f{}; f.kind = 0; f.mode = 0; f.r = rr; f.x1 = tx; f.y1 = ty; f.shade = 1.f;
                const float fast = clamp01(spdN * 1.4f);
                f.x0 = tx * (1.f - fast) + (0.5f + wsRange(-0.04f, 0.04f)) * fast; f.y0 = (-0.25f) * (1.f - fast) + 0.40f * fast;
                f.dur = g_cfg.wsFlyTime * wsRange(0.7f, 1.3f) * (1.15f - 0.4f * fast); f.t = -wsRange(0.f, 0.6f / std::max(1.f, rainRate) + 0.02f);
                g_wsFly.push_back(f);
            } else { wsAddWater(tx, ty, rr, 0.f, g_cfg.wsDrops); wsGridAdd(g_wsFilm, tx, ty, 3.f, 0.12f + 0.45f * g_wsRainEff * g_wsRainEff); }
        }
        if (g_cfg.wsMicro > 0) {                                       // the fine speckle of tiny droplets that makes wet glass look wet
            g_wsMicroAcc += g_cfg.wsMicroRate * g_wsRainEff * (0.5f + 0.5f * spdN) * g_wsWet * dt;
            for (int guard = 0; g_wsMicroAcc >= 1.f && guard < 80; ++guard) {
                g_wsMicroAcc -= 1.f;
                const float mx = wsRange(0.02f, 0.98f), my = wsRange(0.04f, 0.86f);
                wsAddMicro(mx, my, wsRange(0.0006f, 0.0020f) * g_cfg.wsDropSize, g_cfg.wsMicro); wsGridAdd(g_wsFilm, mx, my, 3.f, 0.04f + 0.13f * g_wsRainEff * g_wsRainEff);
            }
        }
    }
    if ((g_cfg.wsMode & 1) && snowy && g_cfg.wsSnowMax > 0) {          // snow flakes landing
        g_wsSnowAcc += g_cfg.wsSnowRate * g_wsSnowEff * (0.3f + 0.7f * std::pow(spdN, 0.7f)) * dt;
        for (int guard = 0; g_wsSnowAcc >= 1.f && guard < 30; ++guard) {
            g_wsSnowAcc -= 1.f;
            const float fx = wsRange(0.03f, 0.97f), fy = 0.06f + 0.74f * std::pow(wsRnd(), 0.85f), fr = (0.006f + 0.020f * wsRnd() * wsRnd()) * g_cfg.wsSnowSize;
            if (g_cfg.wsFly) {
                WsFly f{}; f.kind = 2; f.mode = 0; f.r = fr; f.x1 = fx; f.y1 = fy; f.shade = 1.f; f.x0 = 0.5f + wsRange(-0.05f, 0.05f) + (fx - 0.5f) * 0.1f; f.y0 = 0.38f + wsRange(-0.04f, 0.04f);
                f.dur = g_cfg.wsFlyTime * wsRange(1.1f, 2.0f); f.t = -wsRange(0.f, 0.3f); g_wsFly.push_back(f);
            } else wsAddFlake(fx, fy, fr, g_cfg.wsSnowMax);
        }
    }
    const float splash = g_wsSplash.exchange(0.f);
    const float waterHit = g_wsWaterHit.exchange(0.f);
    if (g_cfg.wsMode != 0 && waterHit > 0.05f) wsWaterSplash(waterHit);
    if (g_cfg.wsMode & 2) {
        const float lg = gnd ? g_wsGround.loose * (0.75f + 0.25f * loose) : loose;   // what the tyres can throw; bumps shake a little more loose
        const float mudRate = g_cfg.wsMudRate * g_wsMudMul * (gnd ? g_wsGround.mudMul : wetMul) / 60.f * spdN * std::pow(lg, 1.5f) * (1.f + 0.5f * std::min(1.f, std::fabs(g_in.sway.load())));   // speed on loose, wet ground, more when cornering
        g_wsDbgMud = mudRate * 60.f; g_wsMudAcc += mudRate * dt;
        auto clodTint = [&]() { return gnd ? wsl::pickTint(g_wsGround.clod, wsRnd(), snowy ? (int)wsl::kSlush : (int)wsl::kMud) : (snowy ? 1 : 0); };   // thrown snow is grey slush
        auto fineTint = [&]() { return gnd ? wsl::pickTint(g_wsGround.fine, wsRnd(), (int)wsl::kSpray) : (snowy ? 1 : 0); };
        for (int guard = 0; g_wsMudAcc >= 1.f && guard < 8; ++guard) { g_wsMudAcc -= 1.f; wsMudThrow(1.f, clodTint(), false); }
        if (splash > 0.05f) { wsMudThrow(std::min(1.8f, 0.6f + splash * 0.5f), clodTint(), false); if (splash > 0.6f) wsMudThrow(1.f, fineTint(), true); }   // landing / impact
        float sprayPot;                                                // dirty water, or dust on a dry loose road, thrown up behind the tyres
        if (gnd) sprayPot = std::max(g_wsGround.spray + 0.8f * g_wsGround.dust, g_wsRainEff * 0.35f);
        else sprayPot = std::max(useGame ? (g_wsGW.wet == 2 ? 1.f : g_wsGW.wet == 1 ? 0.4f : 0.f) : 0.f, g_wsRainEff * 0.7f);
        const float sprayRate = g_cfg.wsSpray * sprayPot * g_wsMudMul * 0.9f * std::pow(spdN, 1.5f) * (gnd ? 0.5f + 0.5f * std::min(1.f, lg + 0.3f) : 0.4f + 0.6f * std::min(1.f, loose + 0.3f));
        g_wsSprayAcc += sprayRate * dt;
        for (int guard = 0; g_wsSprayAcc >= 1.f && guard < 4; ++guard) { g_wsSprayAcc -= 1.f; wsMudThrow(1.f, fineTint(), true); }
    }

    // ---- drops: stick, slide, shed water, swallow neighbours; flakes melt ----
    const float sway = g_in.sway.load() * (float)g_cfg.wsSignSway, surge = g_in.surge.load() * (float)g_cfg.wsSignSurge;
    const float Fx = -sway * g_cfg.wsGravity * 0.9f;                   // cornering: drops lag behind sideways
    const float Fy = 0.12f * g_cfg.wsGravity - wind + surge * 0.5f * g_cfg.wsGravity;   // gravity down, wind up, accel / brake along the glass
    const float F = std::sqrt(Fx * Fx + Fy * Fy);
    const float leak = 0.010f * g_cfg.wsDropSize, minR = 0.0012f * g_cfg.wsDropSize;
    std::vector<WsDrop> melted;
    for (size_t i = 0; i < g_wsDrops.size(); ++i) {
        WsDrop& d = g_wsDrops[i]; d.age += dt;
        if (d.kind == 2) { d.melt -= dt; if (d.melt <= 0.f) d.dead = true; continue; }          // micro droplet: only evaporates
        if (d.kind == 1) {                                             // snow flake
            d.melt -= dt;
            if (d.melt <= 0.f) {                                       // melted: becomes a water bead
                WsDrop n{}; n.kind = 0; n.x = d.x; n.y = d.y; n.r = d.r * 0.55f; n.age = 2.f; n.tx = n.x; n.ty = n.y; n.tr = n.r; n.thr = 0.45f + wsRange(-0.05f, 0.05f);
                n.melt0 = n.melt = wsRange(10.f, 24.f);               // slush water evaporates / runs off after a while
                melted.push_back(n); d.dead = true; continue;
            }
        }
        if (d.kind == 0 && d.melt0 > 0.f) { d.melt -= dt; if (d.melt <= 0.f) { d.dead = true; continue; } }
        const float thr = d.thr * (1.f - 0.45f * loose);               // a rough road shakes drops loose
        if (!d.sliding) { if (F > thr && d.age > 0.25f) d.sliding = true; }    // a fresh impact sticks for a moment
        else if (F < thr * 0.65f) d.sliding = false;                           // static friction is higher than kinetic: once moving it keeps going
        const bool slide = d.sliding;
        if (slide) {
            const float sizeK = 0.7f + std::min(1.3f, d.r / 0.007f);           // a bigger drop is heavier and runs faster
            const float k = (F - 0.65f * thr) / F * 4.0f * sizeK * dt;         // kinetic friction is lower than static: a drop that lets go runs
            const float wob = std::sin(d.age * 3.2f + d.rot) * 0.9f * dt;                  // slow heading drift, proportional to speed: smooth meander, no kinks
            const float ovx = d.vx, ovy = d.vy;
            d.vx += Fx * k - ovy * wob; d.vy += Fy * k + ovx * wob;
            const float drag = std::exp(-2.0f * dt); d.vx *= drag; d.vy *= drag;
        } else { const float stick = std::exp(-14.f * dt); d.vx *= stick; d.vy *= stick; }
        const float dh = std::sqrt(d.vx * d.vx + d.vy * d.vy) * dt;    // distance travelled this step, in viewport heights
        if (dh > 0.f) {
            d.x += d.vx * dt * aspect; d.y += d.vy * dt;
            if (d.kind == 0 && d.r > 0.0035f) {                         // a running drop leaves a clear track in the haze
                const int gi = (int)(d.x * (float)kGX), gj = (int)(d.y * (float)kGY);
                if (gi >= 0 && gi < kGX && gj >= 0 && gj < kGY) { g_wsFilm[gj * kGX + gi] *= 0.80f; g_wsDirt[gj * kGX + gi] *= 0.93f; }
            }
            if (d.kind == 0) {
                d.r -= leak * dh;
                const float ex = (d.x - d.tx) / aspect, ey = d.y - d.ty;
                if (std::sqrt(ex * ex + ey * ey) > 0.016f) {           // leave a trail segment
                    g_wsTrails.push_back({d.tx, d.ty, d.x, d.y, std::min(d.tr * 0.40f, 0.0025f), std::min(d.r * 0.40f, 0.0025f), 0.f});
                    d.tx = d.x; d.ty = d.y; d.tr = d.r;
                }
                if (slide && d.age > 0.4f) for (size_t j = 0; j < g_wsDrops.size(); ++j) {   // merge with whatever it runs into
                    WsDrop& o = g_wsDrops[j];
                    if (j == i || o.dead || o.age < 0.15f || (o.kind != 0 && o.kind != 2)) continue;
                    const float ox = (o.x - d.x) / aspect, oy = o.y - d.y;
                    if (std::sqrt(ox * ox + oy * oy) < (d.r + o.r) * (o.kind == 2 ? 0.95f : 0.85f)) { d.r = std::sqrt(d.r * d.r + o.r * o.r * (o.kind == 2 ? 0.25f : 0.9f)); o.dead = true; }   // leaves a clean track behind
                }
            }
        }
        if ((d.kind == 0 && d.r < minR) || d.x < -0.05f || d.x > 1.05f || d.y < -0.06f || d.y > 0.95f) d.dead = true;
    }
    wsFlyUpdate(dt);
    wsWipe(W, H); wsWiperGhost(dt);
    for (int i = 0; i < 2; ++i) g_wsBow[i] *= std::exp(-dt * 1.6f);
    // Dirt dries and flakes off, faster at speed (airflow), so the corners the wipers never reach do not fill up for ever. GrimeLife = seconds at full speed.
    const float shed = g_cfg.wsGrimeLife > 1.f ? (0.25f + 0.75f * spdN) / g_cfg.wsGrimeLife : 0.f;
    wsFilmFlow(dt, g_wsRainEff);
    {   // haze: water film runs off / evaporates slowly, rain washes dirt away a little, and a clean stretch of glass stays clean
        const float fk = std::exp(-dt * g_cfg.wsFilmDecay * (snowy ? 0.4f : 1.f)), dk = std::exp(-dt * (0.004f + 0.03f * g_wsRainEff + shed));
        for (int i = 0; i < kGX * kGY; ++i) { g_wsFilm[i] *= fk; g_wsDirt[i] *= dk; }
    }
    g_wsDrops.erase(std::remove_if(g_wsDrops.begin(), g_wsDrops.end(), [](const WsDrop& d) { return d.dead; }), g_wsDrops.end());
    for (const WsDrop& n : melted) wsInsert(n, g_cfg.wsDrops);

    // ---- trails and mud age ----
    const float life = std::max(0.5f, g_cfg.wsTrail);
    for (WsTrail& t : g_wsTrails) t.age += dt;
    g_wsTrails.erase(std::remove_if(g_wsTrails.begin(), g_wsTrails.end(), [&](const WsTrail& t) { return t.age > life; }), g_wsTrails.end());
    if (g_wsTrails.size() > 4000) g_wsTrails.erase(g_wsTrails.begin(), g_wsTrails.begin() + (g_wsTrails.size() - 4000));
    int alive = 0, aliveSpray = 0; for (const WsMud& m : g_wsMud) if (!m.dying) { if (wsIsSprayV(m.variant)) ++aliveSpray; else ++alive; }
    const int sprayCap = g_cfg.wsMud * 3;                              // spray dots are small and numerous: their own, larger cap
    for (size_t i = 0; i < g_wsMud.size() && (alive > g_cfg.wsMud || aliveSpray > sprayCap); ++i) {   // over a cap: the oldest fade away
        WsMud& m = g_wsMud[i]; if (m.dying) continue;
        if (wsIsSprayV(m.variant)) { if (aliveSpray > sprayCap) { m.dying = true; --aliveSpray; } }
        else if (alive > g_cfg.wsMud) { m.dying = true; --alive; }
    }
    for (WsMud& m : g_wsMud) {
        m.age += dt; if (m.dying) m.fade -= dt / 2.f;
        else if (m.age > 0.3f) m.fade -= dt * (g_wsRainEff * 0.010f + (m.variant == kMudSpeckle ? 0.03f : 0.f) + (m.variant == kMudSmear ? 1.5f : 1.f) * shed);   // rain slowly washes it off; dirty water dries
    }
    g_wsMud.erase(std::remove_if(g_wsMud.begin(), g_wsMud.end(), [](const WsMud& m) { return m.fade <= 0.06f; }), g_wsMud.end());
}

