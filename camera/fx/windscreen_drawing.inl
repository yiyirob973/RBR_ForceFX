// Part of camera_fx.hpp: included from inside its anonymous namespace (ffx::cam), in the order listed there. Not a header on its own.
// ---- drawing ----
bool wsDraw(IDirect3DDevice9* d) {
    if (!d || g_wsTexFailed) return false;
    if (!wsOnBackbuffer(d)) { g_wsSkipped++; return false; }
    if (!wsEnsureResources(d)) return false;
    D3DVIEWPORT9 vp{};
    if (FAILED(d->GetViewport(&vp)) || vp.Width < 64 || vp.Height < 64) return false;
    const float W = (float)vp.Width, H = (float)vp.Height; g_wsVpW = vp.Width; g_wsVpH = vp.Height;

    static LARGE_INTEGER last{}; LARGE_INTEGER now; QueryPerformanceCounter(&now);
    float dt = last.QuadPart ? (float)(now.QuadPart - last.QuadPart) / (float)g_qf.QuadPart : 0.f; last = now;
    dt = std::max(0.f, std::min(dt, 0.05f));
    wsSync(); wsStep(dt, W, H); wsHuntTick();

    const float gz = wsGlassZ();                                       // glass depth in z-buffer units, < 0 = projection not seen yet
    const bool useZ = g_cfg.wsDepthTest && gz >= 0.f;
    M4 C = m4Ident(); bool haveC = false;
    if (gz >= 0.f && g_wsPok) { C = m4Mul(g_wsP, m4Mul(g_patchActive ? g_Hinv : m4Ident(), g_wsPinv)); haveC = true; }
    static bool logged = false;
    if (!logged) { logged = true; Logf("Windscreen: first draw, DrawAt=%d depthTest=%d glassZ=%.5f glassLock=%d refraction=%d hideGameFx=%d viewport=%ux%u", g_cfg.wsDrawAt, useZ ? 1 : 0, gz, haveC ? 1 : 0, g_cfg.wsRefraction, g_cfg.wsHideGameFx, vp.Width, vp.Height); }

    // Glass space: every position is where that spot of the glass appears with the camera at its base pose. Each vertex goes through the same head
    // transform (P * Hinv * P^-1) the game's 3D scene gets, at the glass depth, so everything stays fixed on the car when the head moves.
    auto xf = [&](float px, float py, float& ox, float& oy, float& oz) -> bool {
        oz = 0.f;
        if (!haveC) { ox = px; oy = py; return true; }
        const float c0[4] = {px / W * 2.f - 1.f, 1.f - py / H * 2.f, gz, 1.f};
        float c1[4];
        for (int i = 0; i < 4; ++i) c1[i] = C.m[i][0] * c0[0] + C.m[i][1] * c0[1] + C.m[i][2] * c0[2] + C.m[i][3] * c0[3];
        if (c1[3] < 1e-4f) return false;                               // behind the eye
        const float iw = 1.f / c1[3];
        ox = (c1[0] * iw * 0.5f + 0.5f) * W; oy = (0.5f - c1[1] * iw * 0.5f) * H; oz = clamp01(c1[2] * iw);
        return true;
    };
    static const float uq[4] = {0, 1, 1, 0}, vq[4] = {0, 0, 1, 1};
    auto emit = [&](std::vector<WsVert>& v, const float (&px)[4], const float (&py)[4], DWORD col, float u0, float v0, float u1, float v1) {
        WsVert q[4];
        for (int i = 0; i < 4; ++i) {
            float ox, oy, oz; if (!xf(px[i], py[i], ox, oy, oz)) return;
            q[i] = { (float)vp.X + ox - 0.5f, (float)vp.Y + oy - 0.5f, useZ ? oz : 0.f, 1.f, col, u0 + (u1 - u0) * uq[i], v0 + (v1 - v0) * vq[i] };
        }
        v.push_back(q[0]); v.push_back(q[1]); v.push_back(q[2]); v.push_back(q[0]); v.push_back(q[2]); v.push_back(q[3]);
    };
    auto corners = [&](float cx, float cy, float hw, float hh, float ang, float (&px)[4], float (&py)[4]) {
        static const float sx[4] = {-1, 1, 1, -1}, sy[4] = {-1, -1, 1, 1};
        const float c = std::cos(ang), s = std::sin(ang);
        for (int i = 0; i < 4; ++i) { const float lx = sx[i] * hw, ly = sy[i] * hh; px[i] = cx + lx * c - ly * s; py[i] = cy + lx * s + ly * c; }
    };
    auto quad = [&](std::vector<WsVert>& v, float cx, float cy, float hw, float hh, float ang, DWORD col, float u0, float v0, float u1, float v1) {
        float px[4], py[4]; corners(cx, cy, hw, hh, ang, px, py); emit(v, px, py, col, u0, v0, u1, v1);
    };
    auto strip = [&](std::vector<WsVert>& v, float x0, float y0, float x1, float y1, float w0, float w1, DWORD col) {   // trapezoid from p0 to p1, half widths in pixels
        const float dx = x1 - x0, dy = y1 - y0, len = std::sqrt(dx * dx + dy * dy); if (len < 0.01f) return;
        const float nx = -dy / len, ny = dx / len;
        const float px[4] = {x0 - nx * w0, x0 + nx * w0, x1 + nx * w1, x1 - nx * w1}, py[4] = {y0 - ny * w0, y0 + ny * w0, y1 + ny * w1, y1 - ny * w1};
        emit(v, px, py, col, 0.f, 0.f, 1.f, 1.f);
    };
    const float refK = g_cfg.wsRefractK;
    auto emitRefr = [&](std::vector<WsVert2>& out, float cx, float cy, float hw, float hh, float ang, DWORD col, float mu0, float mv0, float mu1, float mv1, float K) {   // refractive drop: scene sampled inverted and magnified
        float px[4], py[4], ox[4], oy[4], oz[4]; corners(cx, cy, hw, hh, ang, px, py);
        for (int i = 0; i < 4; ++i) if (!xf(px[i], py[i], ox[i], oy[i], oz[i])) return;
        const float mx = (ox[0] + ox[1] + ox[2] + ox[3]) * 0.25f, my = (oy[0] + oy[1] + oy[2] + oy[3]) * 0.25f;
        WsVert2 q[4];
        for (int i = 0; i < 4; ++i)
            q[i] = { (float)vp.X + ox[i] - 0.5f, (float)vp.Y + oy[i] - 0.5f, useZ ? oz[i] : 0.f, 1.f, col,
                     (mx + (mx - ox[i]) * K) / W, (my + (my - oy[i]) * K) / H, mu0 + (mu1 - mu0) * uq[i], mv0 + (mv1 - mv0) * vq[i] };
        out.push_back(q[0]); out.push_back(q[1]); out.push_back(q[2]); out.push_back(q[0]); out.push_back(q[2]); out.push_back(q[3]);
    };
    auto emitFrost = [&](std::vector<WsVert2>& out, float cx, float cy, float hw, float hh, float ang, DWORD col, float mu0, float mv0, float mu1, float mv1) {   // blurred view, masked by a texture
        float px[4], py[4], ox[4], oy[4], oz[4]; corners(cx, cy, hw, hh, ang, px, py);
        for (int i = 0; i < 4; ++i) if (!xf(px[i], py[i], ox[i], oy[i], oz[i])) return;
        WsVert2 q[4];
        for (int i = 0; i < 4; ++i) q[i] = { (float)vp.X + ox[i] - 0.5f, (float)vp.Y + oy[i] - 0.5f, useZ ? oz[i] : 0.f, 1.f, col, ox[i] / W, oy[i] / H, mu0 + (mu1 - mu0) * uq[i], mv0 + (mv1 - mv0) * vq[i] };
        out.push_back(q[0]); out.push_back(q[1]); out.push_back(q[2]); out.push_back(q[0]); out.push_back(q[2]); out.push_back(q[3]);
    };
    g_wsVSprite.clear(); g_wsVShade.clear(); g_wsVShadePool.clear(); g_wsVMud.clear(); g_wsVTrail.clear(); g_wsVFlake.clear(); g_wsVSplash.clear(); g_wsVDbg.clear(); g_wsVMicro.clear(); g_wsVVeil.clear(); g_wsVRefr.clear(); g_wsVRefrPool.clear(); g_wsVFrost.clear(); g_wsVDirt.clear(); g_wsVGlow.clear(); g_wsVFlyMud.clear(); g_wsVFlyFlake.clear(); g_wsVStreak.clear(); g_wsVWBlur.clear();
    const float op = clamp01(g_cfg.wsOpacity);
    auto rgba = [](float r, float g, float b, float a) { return D3DCOLOR_ARGB((int)(255.f * clamp01(a)), (int)(255.f * clamp01(r)), (int)(255.f * clamp01(g)), (int)(255.f * clamp01(b))); };
    const float life = std::max(0.5f, g_cfg.wsTrail);


    bool anyRefr = false;
    for (const WsDrop& dr : g_wsDrops) if (dr.kind == 0 && dr.r * H >= g_cfg.wsRefractMinPx) { anyRefr = true; break; }
    const float filmMax = wsGridMax(g_wsFilm), dirtMax = wsGridMax(g_wsDirt);
    bool anyFrost = false; for (const WsMud& m : g_wsMud) if (m.age >= 0.f && !wsIsSprayV(m.variant) && m.r * H > 9.f) { anyFrost = true; break; }
    const bool veilWanted = !g_wsVeilFailed && ((g_cfg.wsVeil > 0.001f && filmMax > 0.01f) || (g_cfg.wsDirt > 0.001f && (dirtMax > 0.01f || anyFrost)));
    const bool needScene = (g_cfg.wsRefraction && !g_wsRefrFailed && anyRefr) || veilWanted;
    const bool sceneOK = needScene && wsCopyScene(d, vp, veilWanted);
    const bool useRefr = sceneOK && g_cfg.wsRefraction && anyRefr;
    const bool useVeil = sceneOK && veilWanted && g_wsBlurTex[1] && !g_wsVeilFailed;
    g_wsRefrUsed = useRefr; g_wsVeilUsed = useVeil;
    const bool snowyNow = g_wsSnowEff > 0.01f;
    auto buildHaze = [&](const float* grid, std::vector<WsVert>& out, float maxA, float gamma, float cr, float cg, float cb, bool additive) {   // the grid as a smooth sheet of blurred view
        static float va[(kGX + 1) * (kGY + 1)];
        for (int gy = 0; gy <= kGY; ++gy) for (int gx = 0; gx <= kGX; ++gx) {
            float sum = 0.f; int n = 0;
            for (int dy = -1; dy <= 0; ++dy) for (int dx = -1; dx <= 0; ++dx) { const int cx = gx + dx, cy = gy + dy; if (cx >= 0 && cx < kGX && cy >= 0 && cy < kGY) { sum += grid[cy * kGX + cx]; ++n; } }
            va[gy * (kGX + 1) + gx] = n ? std::pow(clamp01(sum / (float)n), gamma) : 0.f;
        }
        for (int gy = 0; gy < kGY; ++gy) for (int gx = 0; gx < kGX; ++gx) {
            const float al[4] = {va[gy * (kGX + 1) + gx], va[gy * (kGX + 1) + gx + 1], va[(gy + 1) * (kGX + 1) + gx + 1], va[(gy + 1) * (kGX + 1) + gx]};
            if (std::max(std::max(al[0], al[1]), std::max(al[2], al[3])) < 0.006f) continue;
            const float gxs[4] = {(float)gx, (float)gx + 1.f, (float)gx + 1.f, (float)gx}, gys[4] = {(float)gy, (float)gy, (float)gy + 1.f, (float)gy + 1.f};
            WsVert q[4]; bool ok = true;
            for (int i = 0; i < 4 && ok; ++i) {
                float ox, oy, oz; ok = xf(gxs[i] / (float)kGX * W, gys[i] / (float)kGY * H, ox, oy, oz);
                const float k = al[i] * maxA;
                q[i] = { (float)vp.X + ox - 0.5f, (float)vp.Y + oy - 0.5f, useZ ? oz : 0.f, 1.f, additive ? rgba(cr * k, cg * k, cb * k, 1.f) : rgba(cr, cg, cb, k), ox / W, oy / H };
            }
            if (ok) { out.push_back(q[0]); out.push_back(q[1]); out.push_back(q[2]); out.push_back(q[0]); out.push_back(q[2]); out.push_back(q[3]); }
        }
    };
    if (useVeil) {
        if (g_cfg.wsVeil > 0.001f && filmMax > 0.01f) {
            buildHaze(g_wsFilm, g_wsVVeil, clamp01(g_cfg.wsVeil) * 0.85f, 0.85f, 0.90f, 0.94f, 0.96f, false);
            if (g_cfg.wsGlow > 0.001f) buildHaze(g_wsFilm, g_wsVGlow, g_cfg.wsGlow, 1.0f, 1.f, 1.f, 1.f, true);      // bright lights bloom through wet glass
        }
        if (g_cfg.wsDirt > 0.001f && dirtMax > 0.01f) {
            const wsl::Rgb hc = g_cfg.wsGround ? g_wsDirtCol : wsMudRgb();       // with the surface scan the haze is the colour of what has been landing
            const float dr = snowyNow ? 0.86f : hc.r * 1.35f + 0.04f, dg = snowyNow ? 0.88f : hc.g * 1.35f + 0.04f, db = snowyNow ? 0.92f : hc.b * 1.35f + 0.04f;
            buildHaze(g_wsDirt, g_wsVDirt, std::min(0.95f, 0.78f * g_cfg.wsDirt), 0.80f, std::min(1.f, dr), std::min(1.f, dg), std::min(1.f, db), false);
        }
    }
    const bool useFrost = sceneOK && g_wsBlurTex[0] && !g_wsVeilFailed && g_cfg.wsDirt > 0.001f;
    for (const WsMud& m : g_wsMud) {
        if (m.age < 0.f) continue;
        float x = m.x, y = m.y, ang = m.rot, stretch = m.stretch, pop;
        if (m.dur > 0.f) {                                             // thrown outwards from the impact point: quick, with a streak along the motion
            pop = 1.f;
            if (m.age < m.dur) { const float k = m.age / m.dur, e = 1.f - (1.f - k) * (1.f - k); x = m.ox + (m.x - m.ox) * e; y = m.oy + (m.y - m.oy) * e; ang = std::atan2(-(m.x - m.ox) * W, (m.y - m.oy) * H); stretch = 1.f + 2.6f * (1.f - k); }
        } else pop = (0.5f + 0.5f * wsSmooth(0.f, 0.09f, m.age)) * (1.f + 0.10f * std::sin(3.14159f * clamp01(m.age / 0.16f)));      // flattens on impact, a little overshoot
        const float hw = m.r * H * pop, a = op * clamp01(m.fade), sh = m.shade > 0.f ? m.shade : 1.f;
        const bool dirty = m.variant == kMudSpeckle;
        DWORD col;
        const wsl::Rgb tc = wsl::tintColour(m.tint, wsMudRgb());       // the colour of what it is: mud, slush, dust, road spray or grass
        if (m.tint == wsl::kSlush) { const float k = std::min(1.1f, sh); col = rgba(tc.r * k, tc.g * k, tc.b * k, a); }
        else { const float k = sh * (dirty ? 1.25f : 1.f); col = rgba(tc.r * k, tc.g * k, tc.b * k, a * (dirty ? 0.85f : 1.f)); }
        const float u0 = (float)(m.variant & 3) * 0.25f, v0 = (float)(m.variant >> 2) * 0.5f;
        if (useFrost && m.variant <= kMudSmear && hw >= 9.f && m.dur <= 0.f)      // mud is not clear: the view behind it is frosted and tinted
            { const wsl::Rgb fc = wsl::frostColour(m.tint, wsMudRgb()); emitFrost(g_wsVFrost, x * W, y * H, hw, hw * stretch, ang, rgba(fc.r, fc.g, fc.b, m.tint == wsl::kSlush ? 0.75f * a : 0.78f * a * std::min(1.f, g_cfg.wsDirt)), u0, v0, u0 + 0.25f, v0 + 0.5f); }
        quad(g_wsVMud, x * W, y * H, hw, hw * stretch, ang, col, u0, v0, u0 + 0.25f, v0 + 0.5f);
    }
    for (const WsFly& f : g_wsFly) {                                   // particles on their way to the glass
        if (f.t <= 0.f || f.kind < 0) continue;
        const float p = std::min(1.f, f.t); float fx, fy, sc, bx, by, bsc; wsFlyPos(f, p, fx, fy, sc);
        const float rpx = f.r * H, X = fx * W, Y = fy * H;
        wsFlyPos(f, std::max(0.f, p - 0.20f), bx, by, bsc);
        const float vis = wsSmooth(0.f, 0.30f, p);
        if (f.kind == 0) {                                             // rain: a short bright streak along its path
            const float w1 = std::max(0.7f, rpx * 0.55f * (0.30f + 0.70f * sc));
            strip(g_wsVStreak, bx * W, by * H, X, Y, w1 * 0.25f, w1, rgba(0.88f, 0.92f, 0.97f, op * 0.60f * vis));
        } else if (f.kind == 2) {                                      // snow: a soft white fleck that grows as it comes
            const float sz = std::max(1.2f, rpx * (0.20f + 0.80f * sc));
            quad(g_wsVFlyFlake, X, Y, sz, sz, f.x1 * 7.f, rgba(1.f, 1.f, 1.f, op * 0.9f * vis), 0.f, 0.f, 1.f, 1.f);
        } else {                                                       // mud clod / dirty spray: a dark blob, stretched along its flight, with a faint streak behind it
            const float dx = (X - bx * W), dy = (Y - by * H), len = std::sqrt(dx * dx + dy * dy), ang = len > 0.5f ? std::atan2(-dx, dy) : 0.f;
            const float hw = std::max(1.5f, rpx * (f.mode == 1 ? (0.40f + 0.60f * sc) : (0.25f + 0.75f * sc))), st = 1.f + std::min(1.4f, len / std::max(4.f, hw) * 0.35f);
            const float sh = f.shade, al = op * (f.kind == 3 ? 0.80f : 0.95f);
            const wsl::Rgb tc = wsl::tintColour(f.tint, wsMudRgb()); const float kc = f.tint == wsl::kSlush ? 1.f : sh * (f.kind == 3 ? 1.25f : 1.f);
            const DWORD col = rgba(tc.r * kc, tc.g * kc, tc.b * kc, al);
            const float u0 = (float)(f.variant & 3) * 0.25f, v0 = (float)(f.variant >> 2) * 0.5f;
            if (f.kind == 1 && len > 2.f) strip(g_wsVStreak, bx * W, by * H, X, Y, hw * 0.15f, hw * 0.55f, f.tint == wsl::kSlush ? rgba(0.85f, 0.88f, 0.92f, op * 0.28f) : rgba(tc.r, tc.g, tc.b, op * 0.30f));
            quad(g_wsVFlyMud, X, Y, hw, hw * st, ang, col, u0, v0, u0 + 0.25f, v0 + 0.5f);
        }
    }
    for (const WsTrail& t : g_wsTrails) {
        const float a = std::pow(std::max(0.f, 1.f - t.age / life), 1.6f) * op * 0.55f;
        strip(g_wsVTrail, t.x0 * W, t.y0 * H, t.x1 * W, t.y1 * H, t.w0 * H, t.w1 * H, rgba(1.f, 1.f, 1.f, a));
    }
    for (const WsDrop& dr : g_wsDrops) {
        const float rpx = dr.r * H;
        if (dr.kind == 1) {                                            // snow flake
            const float prog = 1.f - clamp01(dr.melt / std::max(1.f, dr.melt0)), pop = 0.5f + 0.5f * wsSmooth(0.f, 0.15f, dr.age);
            const float s = rpx * pop * (1.f - 0.35f * prog), a = op * 0.95f * (1.f - 0.45f * prog);
            quad(g_wsVFlake, dr.x * W, dr.y * H, s, s, dr.rot, rgba(1.f, 1.f, 1.f, a), 0.f, 0.f, 1.f, 1.f);
            continue;
        }
        if (dr.kind == 2) {                                            // micro droplet
            const float s = std::max(rpx, 1.2f) * 1.25f, lf = clamp01(dr.melt / 3.f);
            quad(g_wsVMicro, dr.x * W, dr.y * H, s, s, 0.f, rgba(1.f, 1.f, 1.f, op * 0.9f * lf), 0.f, 0.f, 1.f, 1.f);
            continue;
        }
        const float vlen = std::sqrt(dr.vx * dr.vx + dr.vy * dr.vy);
        const bool moving = vlen > 0.02f;
        const float stretch = moving ? 1.0f + std::min(0.7f, vlen * 1.6f) : 1.f;        // a runner is a little longer than wide, it does not carry a tail
        const float ang = moving ? std::atan2(-dr.vx, dr.vy) : 0.f;    // local y axis follows the motion, the head leads
        const float sc = 0.4f + 0.6f * wsSmooth(0.f, 0.12f, dr.age);   // impact: the drop spreads out
        const float lf = dr.melt0 > 0.f ? clamp01(dr.melt / 3.f) : 1.f;   // melted snow fades out
        const bool pool = !moving && g_cfg.wsPools && rpx >= g_cfg.wsPoolMinPx;
        if (pool) {                                                    // big standing drop: a flat irregular pool
            const float sq = std::sqrt(dr.asp), hwp = rpx * 1.75f * sq * sc, hhp = rpx * 1.75f / sq * sc, cu = (float)(dr.var & 1) * 0.5f, cv = (float)(dr.var >> 1) * 0.5f;
            if (useRefr) emitRefr(g_wsVRefrPool, dr.x * W, dr.y * H, hwp, hhp, dr.rot, rgba(0.78f, 0.83f, 0.83f, op * lf), cu, cv, cu + 0.5f, cv + 0.5f, refK * 0.35f);
            quad(g_wsVShadePool, dr.x * W, dr.y * H, hwp, hhp, dr.rot, rgba(1.f, 1.f, 1.f, op * lf), cu, cv, cu + 0.5f, cv + 0.5f);
        } else {
            const float rh = std::min(rpx, 0.013f * H * std::max(0.5f, g_cfg.wsDropSize)), hw = rh * sc * 1.08f, hh = hw * stretch;
            const int cell = moving ? 1 : 0;
            if (useRefr && rpx >= g_cfg.wsRefractMinPx) {
                emitRefr(g_wsVRefr, dr.x * W, dr.y * H, hw, hh, ang, rgba(0.95f, 0.97f, 0.98f, op * lf), cell * 0.5f, 0.f, cell * 0.5f + 0.5f, 1.f, refK);
                quad(g_wsVShade, dr.x * W, dr.y * H, hw, hh, ang, rgba(1.f, 1.f, 1.f, op * lf), cell * 0.5f, 0.f, cell * 0.5f + 0.5f, 1.f);
            } else {
                quad(g_wsVSprite, dr.x * W, dr.y * H, hw, hh, ang, rgba(1.f, 1.f, 1.f, op * 0.9f * lf), 0.f, 0.f, 1.f, 1.f);
            }
        }
        if (dr.age < 0.14f) {                                          // impact flash
            const float k = dr.age / 0.14f, s = rpx * (1.2f + 1.6f * k);
            if (rpx >= 3.f) quad(g_wsVSplash, dr.x * W, dr.y * H, s, s, 0.f, rgba(1.f, 1.f, 1.f, (1.f - k) * 0.20f * op), 0.f, 0.f, 1.f, 1.f);
        }
    }
    if (g_cfg.wsWipers && g_wsBladeInit) {
        WsBlade bl[2]; wsBlades(bl);
        const float thk = std::max(0.3f, g_cfg.wsWiperThick), aw = 0.0058f * H * thk, bw = 0.0070f * H * thk;
        for (int i = 0; i < 2; ++i) {
            if (bl[i].len <= 0.f || (g_cfg.wsWiperSource && !g_wsModelBlade[i].ok)) continue;      // with WiperSource=1 nothing is drawn until the real blade pose is known (no doubled wipers)
            const float pxv = bl[i].px * W, pyv = bl[i].py * H, L = bl[i].len * H, a = g_wsBladeAng[i], ca = std::cos(a), sa = -std::sin(a);
            const float dir = (g_wsBladeAng[i] - g_wsBladePrev[i]) >= 0.f ? 1.f : -1.f;
            if (g_cfg.wsWiperBow && g_wsBow[i] > 0.03f) {              // the ridge of water the blade pushes ahead of it
                const float tx = -std::sin(a) * dir, ty = -std::cos(a) * dir, off = bw * 1.15f, k = std::min(1.f, g_wsBow[i]), wd = (1.6f + 3.6f * k) * std::max(0.6f, H / 1080.f);
                strip(g_wsVTrail, pxv + ca * L * 0.27f + tx * off, pyv + sa * L * 0.27f + ty * off, pxv + ca * L + tx * off, pyv + sa * L + ty * off, wd * 0.8f, wd, rgba(1.f, 1.f, 1.f, std::min(0.9f, 0.95f * k) * op));
            }
            if (!g_cfg.wsWiperDraw) continue;                          // the car's own blades are drawn by the game; the rest is for the plugin's blades
            if (g_cfg.wsWiperBlur) for (int k = 1; k >= 0; --k) {       // motion blur: a faint wedge between the blade and where it was a moment ago
                const float g = g_wsGhost[i][k]; if (std::fabs(g - a) < 0.004f) continue;
                const float r0 = L * 0.26f, r1 = L, cg = std::cos(g), sg = -std::sin(g);
                const float px4[4] = {pxv + cg * r0, pxv + cg * r1, pxv + ca * r1, pxv + ca * r0}, py4[4] = {pyv + sg * r0, pyv + sg * r1, pyv + sa * r1, pyv + sa * r0};
                emit(g_wsVWBlur, px4, py4, rgba(0.03f, 0.03f, 0.035f, (k == 0 ? 0.32f : 0.16f) * op), 0.f, 0.f, 1.f, 1.f);
            }
            const float nx = -sa * (ca > 0.f ? -1.f : 1.f), ny = ca * (ca > 0.f ? -1.f : 1.f);        // unit normal on the upper side of the blade (for the highlights)
            strip(g_wsVDbg, pxv, pyv, pxv + ca * L * 0.97f, pyv + sa * L * 0.97f, aw * 1.15f, aw * 0.80f, rgba(0.085f, 0.085f, 0.095f, 0.98f * op));                  // arm
            strip(g_wsVDbg, pxv + nx * aw * 0.55f, pyv + ny * aw * 0.55f, pxv + ca * L * 0.95f + nx * aw * 0.4f, pyv + sa * L * 0.95f + ny * aw * 0.4f, aw * 0.16f, aw * 0.12f, rgba(0.40f, 0.42f, 0.46f, 0.40f * op));   // arm highlight
            strip(g_wsVDbg, pxv + ca * L * 0.26f, pyv + sa * L * 0.26f, pxv + ca * L, pyv + sa * L, bw, bw * 0.92f, rgba(0.012f, 0.012f, 0.016f, 0.99f * op));          // rubber blade
            strip(g_wsVDbg, pxv + ca * L * 0.27f + nx * bw * 0.45f, pyv + sa * L * 0.27f + ny * bw * 0.45f, pxv + ca * L * 0.99f + nx * bw * 0.45f, pyv + sa * L * 0.99f + ny * bw * 0.45f, bw * 0.20f, bw * 0.18f, rgba(0.45f, 0.47f, 0.52f, 0.38f * op));   // glint on the blade
            quad(g_wsVDbg, pxv, pyv, aw * 1.9f, aw * 1.9f, 0.f, rgba(0.085f, 0.085f, 0.095f, 0.98f * op), 0.f, 0.f, 1.f, 1.f);                                           // pivot cap (octagon)
            quad(g_wsVDbg, pxv, pyv, aw * 1.9f, aw * 1.9f, 0.7854f, rgba(0.085f, 0.085f, 0.095f, 0.98f * op), 0.f, 0.f, 1.f, 1.f);
        }
    }
    if (g_cfg.wsWiperDebug) {                                          // calibration: where the emulated blades are right now
        WsBlade bl[2]; wsBlades(bl);
        for (int i = 0; i < 2; ++i) {
            if (bl[i].len <= 0.f) continue;
            const float pxv = bl[i].px * W, pyv = bl[i].py * H, L = bl[i].len * H, a = g_wsBladeAng[i];
            const DWORD col = i == 0 ? rgba(1.f, 0.1f, 0.8f, 0.9f) : rgba(0.1f, 0.9f, 1.f, 0.9f);
            strip(g_wsVDbg, pxv, pyv, pxv + std::cos(a) * L, pyv - std::sin(a) * L, 2.5f, 2.5f, col);
            quad(g_wsVDbg, pxv, pyv, 8.f, 8.f, 0.f, col, 0.f, 0.f, 1.f, 1.f);
            if (!(g_cfg.wsWiperSource && g_wsModelBlade[i].ok)) {                              // emulated: the two ends of the sweep, faint
                const float a0 = bl[i].a0 * 0.0174533f, a1 = bl[i].a1 * 0.0174533f;
                strip(g_wsVDbg, pxv, pyv, pxv + std::cos(a0) * L, pyv - std::sin(a0) * L, 1.2f, 1.2f, rgba(0.2f, 1.f, 0.2f, 0.45f));
                strip(g_wsVDbg, pxv, pyv, pxv + std::cos(a1) * L, pyv - std::sin(a1) * L, 1.2f, 1.2f, rgba(1.f, 0.9f, 0.1f, 0.45f));
            }
        }
    }
    if (g_wsVMud.empty() && g_wsVDirt.empty() && g_wsVGlow.empty() && g_wsVFrost.empty() && g_wsVFlyMud.empty() && g_wsVFlyFlake.empty() && g_wsVStreak.empty() && g_wsVWBlur.empty() && g_wsVTrail.empty() && g_wsVSprite.empty() && g_wsVRefr.empty() && g_wsVRefrPool.empty() && g_wsVShadePool.empty() && g_wsVMicro.empty() && g_wsVVeil.empty() && g_wsVFlake.empty() && g_wsVSplash.empty() && g_wsVDbg.empty()) { g_wsDrawn++; return true; }

    g_wsDrawing = true;
    g_wsSB->Capture();                                                 // save everything the game had set
    d->SetVertexShader(nullptr); d->SetPixelShader(nullptr);
    d->SetRenderState(D3DRS_ZENABLE, useZ ? TRUE : FALSE);  d->SetRenderState(D3DRS_ZFUNC, D3DCMP_LESSEQUAL);   d->SetRenderState(D3DRS_ZWRITEENABLE, FALSE);
    d->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);    d->SetRenderState(D3DRS_SEPARATEALPHABLENDENABLE, FALSE);
    d->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA); d->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
    d->SetRenderState(D3DRS_BLENDOP, D3DBLENDOP_ADD);
    d->SetRenderState(D3DRS_ALPHATESTENABLE, FALSE);    d->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
    d->SetRenderState(D3DRS_LIGHTING, FALSE);           d->SetRenderState(D3DRS_FOGENABLE, FALSE);
    d->SetRenderState(D3DRS_STENCILENABLE, FALSE);      d->SetRenderState(D3DRS_SCISSORTESTENABLE, FALSE);
    d->SetRenderState(D3DRS_COLORWRITEENABLE, 0xF);     d->SetRenderState(D3DRS_SRGBWRITEENABLE, FALSE);
    d->SetRenderState(D3DRS_FILLMODE, D3DFILL_SOLID);   d->SetRenderState(D3DRS_SPECULARENABLE, FALSE);
    for (DWORD s = 0; s < 2; ++s) {
        d->SetSamplerState(s, D3DSAMP_MINFILTER, D3DTEXF_LINEAR); d->SetSamplerState(s, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR); d->SetSamplerState(s, D3DSAMP_MIPFILTER, D3DTEXF_NONE);
        d->SetSamplerState(s, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP); d->SetSamplerState(s, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP); d->SetSamplerState(s, D3DSAMP_SRGBTEXTURE, 0);
    }
    d->SetTextureStageState(0, D3DTSS_TEXCOORDINDEX, 0);
    d->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_MODULATE);   d->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_TEXTURE); d->SetTextureStageState(0, D3DTSS_COLORARG2, D3DTA_DIFFUSE);
    d->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_MODULATE);   d->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);  d->SetTextureStageState(0, D3DTSS_ALPHAARG2, D3DTA_DIFFUSE);
    d->SetTextureStageState(1, D3DTSS_COLOROP, D3DTOP_DISABLE);    d->SetTextureStageState(1, D3DTSS_ALPHAOP, D3DTOP_DISABLE);
    d->SetTextureStageState(2, D3DTSS_COLOROP, D3DTOP_DISABLE);    d->SetTextureStageState(2, D3DTSS_ALPHAOP, D3DTOP_DISABLE);
    d->SetTexture(1, nullptr);
    auto batch = [&](std::vector<WsVert>& v, IDirect3DTexture9* tex) {
        if (v.empty()) return;
        d->SetFVF(kWsFvf1); d->SetTexture(0, tex); d->DrawPrimitiveUP(D3DPT_TRIANGLELIST, (UINT)(v.size() / 3), v.data(), sizeof(WsVert));
    };
    batch(g_wsVStreak, g_wsStreakTex); batch(g_wsVFlyFlake, g_wsFlakeTex);          // things still outside the glass: rain streaks, snow, the trails behind mud
    if (useVeil) {
        batch(g_wsVVeil, g_wsBlurTex[1]); batch(g_wsVDirt, g_wsBlurTex[1]);          // haze: a blurred copy of the frame where the glass is wet / dirty
        if (!g_wsVGlow.empty()) {                                                    // bloom: the same blurred copy added on top (bright lights glow)
            d->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_ONE); d->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_ONE);
            batch(g_wsVGlow, g_wsBlurTex[1]);
            d->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA); d->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
        }
    }
    auto drawRefr = [&](std::vector<WsVert2>& v, IDirect3DTexture9* maskTex, IDirect3DTexture9* sceneTex) {   // stage 0: a copy of the frame tinted by the vertex colour; stage 1: the mask supplies the alpha
        if (v.empty()) return;
        d->SetFVF(kWsFvf2); d->SetTexture(0, sceneTex); d->SetTexture(1, maskTex);
        d->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1); d->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_DIFFUSE);
        d->SetTextureStageState(1, D3DTSS_TEXCOORDINDEX, 1);
        d->SetTextureStageState(1, D3DTSS_COLOROP, D3DTOP_SELECTARG1); d->SetTextureStageState(1, D3DTSS_COLORARG1, D3DTA_CURRENT);
        d->SetTextureStageState(1, D3DTSS_ALPHAOP, D3DTOP_MODULATE);   d->SetTextureStageState(1, D3DTSS_ALPHAARG1, D3DTA_TEXTURE); d->SetTextureStageState(1, D3DTSS_ALPHAARG2, D3DTA_CURRENT);
        d->DrawPrimitiveUP(D3DPT_TRIANGLELIST, (UINT)(v.size() / 3), v.data(), sizeof(WsVert2));
        d->SetTextureStageState(1, D3DTSS_COLOROP, D3DTOP_DISABLE); d->SetTextureStageState(1, D3DTSS_ALPHAOP, D3DTOP_DISABLE); d->SetTexture(1, nullptr);
        d->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_MODULATE); d->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);
    };
    if (useFrost) drawRefr(g_wsVFrost, g_wsMudTex, g_wsBlurTex[0]);                 // frosted view inside the mud splats
    batch(g_wsVMud, g_wsMudTex); batch(g_wsVFlyMud, g_wsMudTex); batch(g_wsVTrail, g_wsTrailTex); batch(g_wsVMicro, g_wsMicroTex);
    drawRefr(g_wsVRefrPool, g_wsPoolMaskTex, g_wsSceneTex); batch(g_wsVShadePool, g_wsPoolShadeTex);
    drawRefr(g_wsVRefr, g_wsMaskTex, g_wsSceneTex);
    batch(g_wsVShade, g_wsShadeTex); batch(g_wsVSprite, g_wsSpriteTex); batch(g_wsVFlake, g_wsFlakeTex); batch(g_wsVSplash, g_wsSplashTex); batch(g_wsVWBlur, g_wsWhiteTex); batch(g_wsVDbg, g_wsWhiteTex);
    g_wsSB->Apply();                                                   // restore the game's state
    g_wsDrawing = false;
    g_wsDrawn++;
    return true;
}

