// Material IDs from NGP's physics.lsp (NGP 7.4.778, 133 entries). GENERATED: edit ForceFXAudio\materials.csv
// to override at runtime instead of changing this table.
// cls: 0 Tarmac, 1 Loose (gravel/dirt/water), 2 Rough (grass/rough), 3 Slippery (snow), -1 not ground (objects).
// wet: 0 dry/none, 1 damp, 2 wet.
#pragma once
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

namespace ffx {

struct MaterialInfo { int id; int cls; int wet; const char* name; };

static const MaterialInfo kMaterials[] = {
    {0, -1, 0, "Undefined"},
    {1, 1, 0, "Gravel_Fine_Deeper_Dry"},
    {2, 1, 1, "Gravel_Fine_Deeper_Damp"},
    {3, 1, 2, "Gravel_Fine_Deeper_Wet"},
    {5, 1, 0, "Gravel_Medium_Deeper_Dry"},
    {6, 1, 1, "Gravel_Medium_Deeper_Damp"},
    {7, 1, 2, "Gravel_Medium_Deeper_Wet"},
    {9, 1, 0, "Gravel_Fine_Deep_Dry"},
    {10, 1, 1, "Gravel_Fine_Deep_Damp"},
    {11, 1, 2, "Gravel_Fine_Deep_Wet"},
    {13, 1, 0, "Gravel_Medium_Deep_Dry"},
    {14, 1, 1, "Gravel_Medium_Deep_Damp"},
    {15, 1, 2, "Gravel_Medium_Deep_Wet"},
    {17, 1, 0, "Gravel_Fine_Shallow_Dry"},
    {18, 1, 1, "Gravel_Fine_Shallow_Damp"},
    {19, 1, 2, "Gravel_Fine_Shallow_Wet"},
    {21, 1, 0, "Gravel_Medium_Shallow_Dry"},
    {22, 1, 1, "Gravel_Medium_Shallow_Damp"},
    {23, 1, 2, "Gravel_Medium_Shallow_Wet"},
    {24, -1, 0, "Spret_Bush_Small"},
    {25, -1, 0, "Spret_Bush_Medium"},
    {26, -1, 0, "Spret_Bush_Large"},
    {27, -1, 0, "Lov_Bush_Small"},
    {28, -1, 0, "Lov_Bush_Medium"},
    {29, -1, 0, "Lov_Bush_Large"},
    {30, -1, 0, "Tree_Small"},
    {31, -1, 0, "Tree_Medium"},
    {32, -1, 0, "Tree_Large"},
    {33, 1, 0, "Gravel_Coarse_Deeper_Dry"},
    {34, 1, 1, "Gravel_Coarse_Deeper_Damp"},
    {35, 1, 2, "Gravel_Coarse_Deeper_Wet"},
    {37, 0, 0, "Tarmac_Fine_Clean_Dry"},
    {38, 0, 1, "Tarmac_Fine_Clean_Damp"},
    {39, 0, 2, "Tarmac_Fine_Clean_Wet"},
    {41, 1, 0, "Gravel_Coarse_Deep_Dry"},
    {42, 1, 1, "Gravel_Coarse_Deep_Damp"},
    {43, 1, 2, "Gravel_Coarse_Deep_Wet"},
    {45, 0, 0, "Tarmac_Fine_Sprinkled_Dry"},
    {46, 0, 1, "Tarmac_Fine_Sprinkled_Damp"},
    {47, 0, 2, "Tarmac_Fine_Sprinkled_Wet"},
    {49, 1, 0, "Gravel_Coarse_Shallow_Dry"},
    {50, 1, 1, "Gravel_Coarse_Shallow_Damp"},
    {51, 1, 2, "Gravel_Coarse_Shallow_Wet"},
    {53, 0, 0, "Tarmac_Fine_Covered_Dry"},
    {54, 0, 1, "Tarmac_Fine_Covered_Damp"},
    {55, 0, 2, "Tarmac_Fine_Covered_Wet"},
    {56, -1, 0, "Rock_Small"},
    {57, -1, 0, "Rock_Medium"},
    {58, -1, 0, "Rock_Large"},
    {59, -1, 0, "Trunk_Small"},
    {60, -1, 0, "Trunk_Medium"},
    {61, -1, 0, "Trunk_Large"},
    {62, -1, 0, "Snowwall"},
    {63, -1, 0, "Metal_Pole"},
    {65, 0, 0, "Tarmac_Medium_Clean_Dry"},
    {66, 0, 1, "Tarmac_Medium_Clean_Damp"},
    {67, 0, 2, "Tarmac_Medium_Clean_Wet"},
    {69, 0, 0, "Tarmac_Coarse_Clean_Dry"},
    {70, 0, 1, "Tarmac_Coarse_Clean_Damp"},
    {71, 0, 2, "Tarmac_Coarse_Clean_Wet"},
    {73, 0, 0, "Tarmac_Medium_Sprinkled_Dry"},
    {74, 0, 1, "Tarmac_Medium_Sprinkled_Damp"},
    {75, 0, 2, "Tarmac_Medium_Sprinkled_Wet"},
    {77, 0, 0, "Tarmac_Coarse_Sprinkled_Dry"},
    {78, 0, 1, "Tarmac_Coarse_Sprinkled_Damp"},
    {79, 0, 2, "Tarmac_Coarse_Sprinkled_Wet"},
    {81, 0, 0, "Tarmac_Medium_Covered_Dry"},
    {82, 0, 1, "Tarmac_Medium_Covered_Damp"},
    {83, 0, 2, "Tarmac_Medium_Covered_Wet"},
    {85, 0, 0, "Tarmac_Coarse_Covered_Dry"},
    {86, 0, 1, "Tarmac_Coarse_Covered_Damp"},
    {87, 0, 2, "Tarmac_Coarse_Covered_Wet"},
    {88, -1, 0, "Snowwall Bottom"},
    {97, 2, 0, "Grass_Hard_Dry"},
    {98, 2, 1, "Grass_Hard_Damp"},
    {99, 2, 2, "Grass_Hard_Wet"},
    {101, 3, 0, "SnowOnGravel_Shallow"},
    {102, 3, 0, "SnowOnGravel_Medium"},
    {103, 3, 0, "SnowOnGravel_Deep"},
    {105, 2, 0, "Grass_Medium_Dry"},
    {106, 2, 1, "Grass_Medium_Damp"},
    {107, 2, 2, "Grass_Medium_Wet"},
    {113, 2, 0, "Grass_Soft_Dry"},
    {114, 2, 1, "Grass_Soft_Damp"},
    {115, 2, 2, "Grass_Soft_Wet"},
    {117, 3, 0, "SnowOnIce_Shallow"},
    {118, 3, 0, "SnowOnIce_Medium"},
    {119, 3, 0, "SnowOnIce_Deep"},
    {120, -1, 0, "Tree_Stump"},
    {121, -1, 0, "Bendable_Tree"},
    {129, 2, 0, "Rough_Rough_Dry"},
    {130, 2, 2, "Rough_Rough_Wet"},
    {133, 1, 0, "Dirt_Hard_Dry"},
    {134, 1, 1, "Dirt_Hard_Damp"},
    {135, 1, 2, "Dirt_Hard_Wet"},
    {137, 2, 0, "Rough_VeryRough_Dry"},
    {138, 2, 2, "Rough_VeryRough_Wet"},
    {141, 1, 0, "Dirt_Medium_Dry"},
    {142, 1, 1, "Dirt_Medium_Damp"},
    {143, 1, 2, "Dirt_Medium_Wet"},
    {149, 1, 0, "Dirt_Soft_Dry"},
    {150, 1, 1, "Dirt_Soft_Damp"},
    {151, 1, 2, "Dirt_Soft_Wet"},
    {153, 1, 0, "Gravel_Fine_Shallow_Dry_Clone"},
    {154, 1, 1, "Gravel_Fine_Shallow_Damp_Clone"},
    {155, 1, 2, "Gravel_Fine_Shallow_Wet_Clone"},
    {157, 1, 0, "Water_Surface"},
    {161, 1, 0, "Dirt_Hard_Dry_Clone"},
    {162, 1, 1, "Dirt_Hard_Damp_Clone"},
    {163, 1, 2, "Dirt_Hard_Wet_Clone"},
    {165, -1, 0, "Very_Sparse_Bush"},
    {166, -1, 0, "Sparse_Bush"},
    {167, -1, 0, "Dense_Bush"},
    {168, -1, 0, "Very_Dense_Bush"},
    {177, 1, 0, "US2_Gravel_Fine_Shallow_Dry"},
    {178, 1, 1, "US2_Gravel_Fine_Shallow_Damp"},
    {179, 1, 2, "US2_Gravel_Fine_Shallow_Wet"},
    {181, 1, 0, "BR2_Gravel_Coarse_Shallow_Dry"},
    {182, 1, 1, "BR2_Gravel_Coarse_Shallow_Damp"},
    {183, 1, 2, "BR2_Gravel_Coarse_Shallow_Wet"},
    {185, 1, 0, "US2_Gravel_Fine_Deep_Dry"},
    {186, 1, 1, "US2_Gravel_Fine_Deep_Damp"},
    {187, 1, 2, "US2_Gravel_Fine_Deep_Wet"},
    {189, 1, 0, "BR2_Gravel_Coarse_Deep_Dry"},
    {190, 1, 1, "BR2_Gravel_Coarse_Deep_Damp"},
    {191, 1, 2, "BR2_Gravel_Coarse_Deep_Wet"},
    {193, 1, 0, "US2_Gravel_Fine_Deeper_Dry"},
    {194, 1, 1, "US2_Gravel_Fine_Deeper_Damp"},
    {195, 1, 2, "US2_Gravel_Fine_Deeper_Wet"},
    {197, 1, 0, "BR2_Gravel_Coarse_Deeper_Dry"},
    {198, 1, 1, "BR2_Gravel_Coarse_Deeper_Damp"},
    {199, 1, 2, "BR2_Gravel_Coarse_Deeper_Wet"},
    {255, -1, 0, "Spectator"},
};


// id -> class lookup with optional runtime overrides. Unknown ids report cls = -2 (not in the table).
class MaterialTable {
public:
    MaterialTable() {
        for (int i = 0; i < 256; ++i) { cls_[i] = -2; wet_[i] = 0; name_[i] = "unknown"; }
        for (const MaterialInfo& m : kMaterials)
            if (m.id >= 0 && m.id < 256) { cls_[m.id] = m.cls; wet_[m.id] = m.wet; name_[m.id] = m.name; }
    }
    // Lines "id,class,wet"; class -1..3, wet 0..2; '#' or ';' comments. Returns false if the file is missing.
    bool loadOverrides(const std::string& path) {
        std::ifstream f(path);
        if (!f) return false;
        std::string line;
        while (std::getline(f, line)) {
            size_t c = line.find_first_of(";#"); if (c != std::string::npos) line.resize(c);
            std::stringstream ss(line); std::string a, b, w;
            if (!std::getline(ss, a, ',') || !std::getline(ss, b, ',')) continue;
            std::getline(ss, w, ',');
            int id = std::atoi(a.c_str()), cl = std::atoi(b.c_str()), we = std::atoi(w.c_str());
            if (id < 0 || id > 255 || cl < -1 || cl > 3) continue;
            cls_[id] = cl; wet_[id] = we < 0 ? 0 : (we > 2 ? 2 : we);
            if (std::string(name_[id]) == "unknown") name_[id] = "override";
        }
        return true;
    }
    // How much loose material (stones, grit, clods, slush) this ground has for a tyre to throw: 0 = none, 1 = deep loose gravel.
    // It comes from the game's own material name, so a clean tarmac road throws nothing, a road with gravel sprinkled on it a
    // little, and one covered in gravel a lot. Ids added by a materials.csv override use their class instead. -1 = not ground.
    double looseness(int id) const {
        if (id < 0 || id > 255 || cls_[id] < 0) return -1.0;
        const std::string n = name_[id];
        auto has = [&n](const char* w) { return n.find(w) != std::string::npos; };
        if (n == "override" || n == "unknown") { static const double byClass[4] = {0.1, 0.85, 0.5, 0.4}; return byClass[cls_[id]]; }
        double v;
        if (has("Tarmac")) v = has("Clean") ? 0.0 : (has("Sprinkled") ? 0.35 : (has("Covered") ? 0.75 : 0.0));
        else if (has("Gravel")) v = has("Deeper") ? 1.0 : (has("Deep") ? 0.9 : (has("Shallow") ? 0.75 : 0.85));
        else if (has("SnowOnGravel")) v = has("Deep") ? 0.7 : (has("Medium") ? 0.6 : 0.5);
        else if (has("SnowOnIce")) v = 0.2;
        else if (has("Dirt")) v = has("Soft") ? 0.75 : (has("Medium") ? 0.6 : 0.45);
        else if (has("Rough")) v = has("VeryRough") ? 0.8 : 0.65;
        else if (has("Grass")) v = has("Soft") ? 0.45 : (has("Medium") ? 0.3 : 0.2);
        else if (has("Water")) v = 0.25;
        else { static const double byClass[4] = {0.1, 0.85, 0.5, 0.4}; v = byClass[cls_[id]]; }
        return v;
    }
    int cls(int id) const { return (id < 0 || id > 255) ? -2 : cls_[id]; }
    int wet(int id) const { return (id < 0 || id > 255) ? 0 : wet_[id]; }
    const char* name(int id) const { return (id < 0 || id > 255) ? "out of range" : name_[id]; }
private:
    int cls_[256]; int wet_[256]; const char* name_[256];
};

}  // namespace ffx
