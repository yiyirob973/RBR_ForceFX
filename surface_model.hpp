// What the plugin knows about the ground under one tyre, in one form for both the road feel and the audio. Pure data and two small mappings,
// no Windows or game access, tested in tests/surface_model_test.cpp.
//
// The tyre reader (Surf::Reader in RBR_ForceFX.cpp) is the one place that decides what a tyre is on: the game's material id, the class it
// belongs to (with [Surface] Map= corrections and the telemetry's cobble detection applied), and from those the ground (road_feel.hpp).
// The audio used to read the same ids but work the class, wetness and looseness out again from its own table. Now the plugin hands it this.
#pragma once

namespace SurfaceModel {

struct Tyre {
    int id = -1;             // the game's material id under the tyre (-1 = not known)
    int audioClass = -1;     // 0 tarmac, 1 loose (gravel, dirt), 2 rough (grass, soft ground), 3 slippery (snow, ice); -1 = the reader has no class
    int wet = 0;             // the game's 0 dry, 1 damp, 2 wet
    float loose = -1.0f;     // how much loose material the ground has, 0..1 (-1 = not known)
    bool ridged = false;     // cobbles or planks (assigned with Map=, or seen in the telemetry)
};

// The reader's classes (Surf::Cls in surface_feel.hpp): 0 none, 1 tarmac, 2 gravel, 3 snow, 4 ice, 5 soft, 6 cobble, 7 wood.
// The audio has four sound classes. Cobbles and wooden planks are hard ground: they use the tarmac sounds.
inline int AudioClassOf(int surfCls) {
    switch (surfCls) {
        case 1: return 0;            // tarmac
        case 2: return 1;            // gravel
        case 3: return 3;            // snow
        case 4: return 3;            // ice
        case 5: return 2;            // soft ground, grass
        case 6: return 0;            // cobble
        case 7: return 0;            // wood
        default: return -1;
    }
}

// The other way: the reader's class for one of the audio's four (used by the materials.csv overrides, which name audio classes).
inline int SurfClassOfAudio(int audioClass) {
    switch (audioClass) {
        case 0: return 1;
        case 1: return 2;
        case 2: return 5;
        case 3: return 3;
        default: return 0;
    }
}

}  // namespace SurfaceModel
