# Surfaces and sound file names

RBR/NGP has 107 ground materials. Every tyre plays the sound set of the
exact material under it. For each material the plugin looks for sample files in the order shown below and uses the **first that
exists**, separately for each kind: `<name>_loop`, `<name>_squeal`, `<name>_slide`, `<name>_hit` (any number of variants: `_1`, `_2`...).
The last entry is the plain class name from the ini (default tarmac / gravel / grass / snow), so a four-surface pack works as-is and
each extra file you add makes just those surfaces more specific. If the plugin can't read the ground (the Diagnostics page says so), only the four class names are used.

Handy shortcuts: `tarmac_wet_loop` covers every wet tarmac, `gravel_wet_loop` every wet gravel, `dirt_loop`, `ice_loop` (snow on ice),
`water_loop`, `rough_loop`. Material names are not case sensitive.

| ID | Material | Class | Wet | Files searched (most specific first) |
|---|---|---|---|---|
| 1 | Gravel_Fine_Deeper_Dry | loose | dry | `gravel_fine_deeper_dry` , `gravel_fine_deeper` , `gravel_fine` , `gravel` |
| 2 | Gravel_Fine_Deeper_Damp | loose | damp | `gravel_fine_deeper_damp` , `gravel_fine_deeper` , `gravel_fine` , `gravel_damp` , `gravel` |
| 3 | Gravel_Fine_Deeper_Wet | loose | wet | `gravel_fine_deeper_wet` , `gravel_fine_deeper` , `gravel_fine` , `gravel_wet` , `gravel` |
| 5 | Gravel_Medium_Deeper_Dry | loose | dry | `gravel_medium_deeper_dry` , `gravel_medium_deeper` , `gravel_medium` , `gravel` |
| 6 | Gravel_Medium_Deeper_Damp | loose | damp | `gravel_medium_deeper_damp` , `gravel_medium_deeper` , `gravel_medium` , `gravel_damp` , `gravel` |
| 7 | Gravel_Medium_Deeper_Wet | loose | wet | `gravel_medium_deeper_wet` , `gravel_medium_deeper` , `gravel_medium` , `gravel_wet` , `gravel` |
| 9 | Gravel_Fine_Deep_Dry | loose | dry | `gravel_fine_deep_dry` , `gravel_fine_deep` , `gravel_fine` , `gravel` |
| 10 | Gravel_Fine_Deep_Damp | loose | damp | `gravel_fine_deep_damp` , `gravel_fine_deep` , `gravel_fine` , `gravel_damp` , `gravel` |
| 11 | Gravel_Fine_Deep_Wet | loose | wet | `gravel_fine_deep_wet` , `gravel_fine_deep` , `gravel_fine` , `gravel_wet` , `gravel` |
| 13 | Gravel_Medium_Deep_Dry | loose | dry | `gravel_medium_deep_dry` , `gravel_medium_deep` , `gravel_medium` , `gravel` |
| 14 | Gravel_Medium_Deep_Damp | loose | damp | `gravel_medium_deep_damp` , `gravel_medium_deep` , `gravel_medium` , `gravel_damp` , `gravel` |
| 15 | Gravel_Medium_Deep_Wet | loose | wet | `gravel_medium_deep_wet` , `gravel_medium_deep` , `gravel_medium` , `gravel_wet` , `gravel` |
| 17 | Gravel_Fine_Shallow_Dry | loose | dry | `gravel_fine_shallow_dry` , `gravel_fine_shallow` , `gravel_fine` , `gravel` |
| 18 | Gravel_Fine_Shallow_Damp | loose | damp | `gravel_fine_shallow_damp` , `gravel_fine_shallow` , `gravel_fine` , `gravel_damp` , `gravel` |
| 19 | Gravel_Fine_Shallow_Wet | loose | wet | `gravel_fine_shallow_wet` , `gravel_fine_shallow` , `gravel_fine` , `gravel_wet` , `gravel` |
| 21 | Gravel_Medium_Shallow_Dry | loose | dry | `gravel_medium_shallow_dry` , `gravel_medium_shallow` , `gravel_medium` , `gravel` |
| 22 | Gravel_Medium_Shallow_Damp | loose | damp | `gravel_medium_shallow_damp` , `gravel_medium_shallow` , `gravel_medium` , `gravel_damp` , `gravel` |
| 23 | Gravel_Medium_Shallow_Wet | loose | wet | `gravel_medium_shallow_wet` , `gravel_medium_shallow` , `gravel_medium` , `gravel_wet` , `gravel` |
| 33 | Gravel_Coarse_Deeper_Dry | loose | dry | `gravel_coarse_deeper_dry` , `gravel_coarse_deeper` , `gravel_coarse` , `gravel` |
| 34 | Gravel_Coarse_Deeper_Damp | loose | damp | `gravel_coarse_deeper_damp` , `gravel_coarse_deeper` , `gravel_coarse` , `gravel_damp` , `gravel` |
| 35 | Gravel_Coarse_Deeper_Wet | loose | wet | `gravel_coarse_deeper_wet` , `gravel_coarse_deeper` , `gravel_coarse` , `gravel_wet` , `gravel` |
| 37 | Tarmac_Fine_Clean_Dry | tarmac | dry | `tarmac_fine_clean_dry` , `tarmac_fine_clean` , `tarmac_fine` , `tarmac` |
| 38 | Tarmac_Fine_Clean_Damp | tarmac | damp | `tarmac_fine_clean_damp` , `tarmac_fine_clean` , `tarmac_fine` , `tarmac_damp` , `tarmac` |
| 39 | Tarmac_Fine_Clean_Wet | tarmac | wet | `tarmac_fine_clean_wet` , `tarmac_fine_clean` , `tarmac_fine` , `tarmac_wet` , `tarmac` |
| 41 | Gravel_Coarse_Deep_Dry | loose | dry | `gravel_coarse_deep_dry` , `gravel_coarse_deep` , `gravel_coarse` , `gravel` |
| 42 | Gravel_Coarse_Deep_Damp | loose | damp | `gravel_coarse_deep_damp` , `gravel_coarse_deep` , `gravel_coarse` , `gravel_damp` , `gravel` |
| 43 | Gravel_Coarse_Deep_Wet | loose | wet | `gravel_coarse_deep_wet` , `gravel_coarse_deep` , `gravel_coarse` , `gravel_wet` , `gravel` |
| 45 | Tarmac_Fine_Sprinkled_Dry | tarmac | dry | `tarmac_fine_sprinkled_dry` , `tarmac_fine_sprinkled` , `tarmac_fine` , `tarmac` |
| 46 | Tarmac_Fine_Sprinkled_Damp | tarmac | damp | `tarmac_fine_sprinkled_damp` , `tarmac_fine_sprinkled` , `tarmac_fine` , `tarmac_damp` , `tarmac` |
| 47 | Tarmac_Fine_Sprinkled_Wet | tarmac | wet | `tarmac_fine_sprinkled_wet` , `tarmac_fine_sprinkled` , `tarmac_fine` , `tarmac_wet` , `tarmac` |
| 49 | Gravel_Coarse_Shallow_Dry | loose | dry | `gravel_coarse_shallow_dry` , `gravel_coarse_shallow` , `gravel_coarse` , `gravel` |
| 50 | Gravel_Coarse_Shallow_Damp | loose | damp | `gravel_coarse_shallow_damp` , `gravel_coarse_shallow` , `gravel_coarse` , `gravel_damp` , `gravel` |
| 51 | Gravel_Coarse_Shallow_Wet | loose | wet | `gravel_coarse_shallow_wet` , `gravel_coarse_shallow` , `gravel_coarse` , `gravel_wet` , `gravel` |
| 53 | Tarmac_Fine_Covered_Dry | tarmac | dry | `tarmac_fine_covered_dry` , `tarmac_fine_covered` , `tarmac_fine` , `tarmac` |
| 54 | Tarmac_Fine_Covered_Damp | tarmac | damp | `tarmac_fine_covered_damp` , `tarmac_fine_covered` , `tarmac_fine` , `tarmac_damp` , `tarmac` |
| 55 | Tarmac_Fine_Covered_Wet | tarmac | wet | `tarmac_fine_covered_wet` , `tarmac_fine_covered` , `tarmac_fine` , `tarmac_wet` , `tarmac` |
| 65 | Tarmac_Medium_Clean_Dry | tarmac | dry | `tarmac_medium_clean_dry` , `tarmac_medium_clean` , `tarmac_medium` , `tarmac` |
| 66 | Tarmac_Medium_Clean_Damp | tarmac | damp | `tarmac_medium_clean_damp` , `tarmac_medium_clean` , `tarmac_medium` , `tarmac_damp` , `tarmac` |
| 67 | Tarmac_Medium_Clean_Wet | tarmac | wet | `tarmac_medium_clean_wet` , `tarmac_medium_clean` , `tarmac_medium` , `tarmac_wet` , `tarmac` |
| 69 | Tarmac_Coarse_Clean_Dry | tarmac | dry | `tarmac_coarse_clean_dry` , `tarmac_coarse_clean` , `tarmac_coarse` , `tarmac` |
| 70 | Tarmac_Coarse_Clean_Damp | tarmac | damp | `tarmac_coarse_clean_damp` , `tarmac_coarse_clean` , `tarmac_coarse` , `tarmac_damp` , `tarmac` |
| 71 | Tarmac_Coarse_Clean_Wet | tarmac | wet | `tarmac_coarse_clean_wet` , `tarmac_coarse_clean` , `tarmac_coarse` , `tarmac_wet` , `tarmac` |
| 73 | Tarmac_Medium_Sprinkled_Dry | tarmac | dry | `tarmac_medium_sprinkled_dry` , `tarmac_medium_sprinkled` , `tarmac_medium` , `tarmac` |
| 74 | Tarmac_Medium_Sprinkled_Damp | tarmac | damp | `tarmac_medium_sprinkled_damp` , `tarmac_medium_sprinkled` , `tarmac_medium` , `tarmac_damp` , `tarmac` |
| 75 | Tarmac_Medium_Sprinkled_Wet | tarmac | wet | `tarmac_medium_sprinkled_wet` , `tarmac_medium_sprinkled` , `tarmac_medium` , `tarmac_wet` , `tarmac` |
| 77 | Tarmac_Coarse_Sprinkled_Dry | tarmac | dry | `tarmac_coarse_sprinkled_dry` , `tarmac_coarse_sprinkled` , `tarmac_coarse` , `tarmac` |
| 78 | Tarmac_Coarse_Sprinkled_Damp | tarmac | damp | `tarmac_coarse_sprinkled_damp` , `tarmac_coarse_sprinkled` , `tarmac_coarse` , `tarmac_damp` , `tarmac` |
| 79 | Tarmac_Coarse_Sprinkled_Wet | tarmac | wet | `tarmac_coarse_sprinkled_wet` , `tarmac_coarse_sprinkled` , `tarmac_coarse` , `tarmac_wet` , `tarmac` |
| 81 | Tarmac_Medium_Covered_Dry | tarmac | dry | `tarmac_medium_covered_dry` , `tarmac_medium_covered` , `tarmac_medium` , `tarmac` |
| 82 | Tarmac_Medium_Covered_Damp | tarmac | damp | `tarmac_medium_covered_damp` , `tarmac_medium_covered` , `tarmac_medium` , `tarmac_damp` , `tarmac` |
| 83 | Tarmac_Medium_Covered_Wet | tarmac | wet | `tarmac_medium_covered_wet` , `tarmac_medium_covered` , `tarmac_medium` , `tarmac_wet` , `tarmac` |
| 85 | Tarmac_Coarse_Covered_Dry | tarmac | dry | `tarmac_coarse_covered_dry` , `tarmac_coarse_covered` , `tarmac_coarse` , `tarmac` |
| 86 | Tarmac_Coarse_Covered_Damp | tarmac | damp | `tarmac_coarse_covered_damp` , `tarmac_coarse_covered` , `tarmac_coarse` , `tarmac_damp` , `tarmac` |
| 87 | Tarmac_Coarse_Covered_Wet | tarmac | wet | `tarmac_coarse_covered_wet` , `tarmac_coarse_covered` , `tarmac_coarse` , `tarmac_wet` , `tarmac` |
| 97 | Grass_Hard_Dry | rough | dry | `grass_hard_dry` , `grass_hard` , `grass` |
| 98 | Grass_Hard_Damp | rough | damp | `grass_hard_damp` , `grass_hard` , `grass_damp` , `grass` |
| 99 | Grass_Hard_Wet | rough | wet | `grass_hard_wet` , `grass_hard` , `grass_wet` , `grass` |
| 101 | SnowOnGravel_Shallow | slippery | dry | `snowongravel_shallow` , `snowongravel` , `snow` |
| 102 | SnowOnGravel_Medium | slippery | dry | `snowongravel_medium` , `snowongravel` , `snow` |
| 103 | SnowOnGravel_Deep | slippery | dry | `snowongravel_deep` , `snowongravel` , `snow` |
| 105 | Grass_Medium_Dry | rough | dry | `grass_medium_dry` , `grass_medium` , `grass` |
| 106 | Grass_Medium_Damp | rough | damp | `grass_medium_damp` , `grass_medium` , `grass_damp` , `grass` |
| 107 | Grass_Medium_Wet | rough | wet | `grass_medium_wet` , `grass_medium` , `grass_wet` , `grass` |
| 113 | Grass_Soft_Dry | rough | dry | `grass_soft_dry` , `grass_soft` , `grass` |
| 114 | Grass_Soft_Damp | rough | damp | `grass_soft_damp` , `grass_soft` , `grass_damp` , `grass` |
| 115 | Grass_Soft_Wet | rough | wet | `grass_soft_wet` , `grass_soft` , `grass_wet` , `grass` |
| 117 | SnowOnIce_Shallow | slippery | dry | `snowonice_shallow` , `snowonice` , `ice` , `snow` |
| 118 | SnowOnIce_Medium | slippery | dry | `snowonice_medium` , `snowonice` , `ice` , `snow` |
| 119 | SnowOnIce_Deep | slippery | dry | `snowonice_deep` , `snowonice` , `ice` , `snow` |
| 129 | Rough_Rough_Dry | rough | dry | `rough_rough_dry` , `rough_rough` , `rough` , `grass` |
| 130 | Rough_Rough_Wet | rough | wet | `rough_rough_wet` , `rough_rough` , `rough_wet` , `rough` , `grass` |
| 133 | Dirt_Hard_Dry | loose | dry | `dirt_hard_dry` , `dirt_hard` , `dirt` , `gravel` |
| 134 | Dirt_Hard_Damp | loose | damp | `dirt_hard_damp` , `dirt_hard` , `dirt_damp` , `dirt` , `gravel` |
| 135 | Dirt_Hard_Wet | loose | wet | `dirt_hard_wet` , `dirt_hard` , `dirt_wet` , `dirt` , `gravel` |
| 137 | Rough_VeryRough_Dry | rough | dry | `rough_veryrough_dry` , `rough_veryrough` , `rough` , `grass` |
| 138 | Rough_VeryRough_Wet | rough | wet | `rough_veryrough_wet` , `rough_veryrough` , `rough_wet` , `rough` , `grass` |
| 141 | Dirt_Medium_Dry | loose | dry | `dirt_medium_dry` , `dirt_medium` , `dirt` , `gravel` |
| 142 | Dirt_Medium_Damp | loose | damp | `dirt_medium_damp` , `dirt_medium` , `dirt_damp` , `dirt` , `gravel` |
| 143 | Dirt_Medium_Wet | loose | wet | `dirt_medium_wet` , `dirt_medium` , `dirt_wet` , `dirt` , `gravel` |
| 149 | Dirt_Soft_Dry | loose | dry | `dirt_soft_dry` , `dirt_soft` , `dirt` , `gravel` |
| 150 | Dirt_Soft_Damp | loose | damp | `dirt_soft_damp` , `dirt_soft` , `dirt_damp` , `dirt` , `gravel` |
| 151 | Dirt_Soft_Wet | loose | wet | `dirt_soft_wet` , `dirt_soft` , `dirt_wet` , `dirt` , `gravel` |
| 153 | Gravel_Fine_Shallow_Dry_Clone | loose | dry | `gravel_fine_shallow_dry` , `gravel_fine_shallow` , `gravel_fine` , `gravel` |
| 154 | Gravel_Fine_Shallow_Damp_Clone | loose | damp | `gravel_fine_shallow_damp` , `gravel_fine_shallow` , `gravel_fine` , `gravel_damp` , `gravel` |
| 155 | Gravel_Fine_Shallow_Wet_Clone | loose | wet | `gravel_fine_shallow_wet` , `gravel_fine_shallow` , `gravel_fine` , `gravel_wet` , `gravel` |
| 157 | Water_Surface | loose | dry | `water_surface` , `water` , `gravel` |
| 161 | Dirt_Hard_Dry_Clone | loose | dry | `dirt_hard_dry` , `dirt_hard` , `dirt` , `gravel` |
| 162 | Dirt_Hard_Damp_Clone | loose | damp | `dirt_hard_damp` , `dirt_hard` , `dirt_damp` , `dirt` , `gravel` |
| 163 | Dirt_Hard_Wet_Clone | loose | wet | `dirt_hard_wet` , `dirt_hard` , `dirt_wet` , `dirt` , `gravel` |
| 177 | US2_Gravel_Fine_Shallow_Dry | loose | dry | `us2_gravel_fine_shallow_dry` , `us2_gravel_fine_shallow` , `us2_gravel_fine` , `us2_gravel` , `us2` , `gravel_fine_shallow_dry` , `gravel_fine_shallow` , `gravel_fine` , `gravel` |
| 178 | US2_Gravel_Fine_Shallow_Damp | loose | damp | `us2_gravel_fine_shallow_damp` , `us2_gravel_fine_shallow` , `us2_gravel_fine` , `us2_gravel` , `us2` , `gravel_fine_shallow_damp` , `gravel_fine_shallow` , `gravel_fine` , `gravel_damp` , `gravel` |
| 179 | US2_Gravel_Fine_Shallow_Wet | loose | wet | `us2_gravel_fine_shallow_wet` , `us2_gravel_fine_shallow` , `us2_gravel_fine` , `us2_gravel` , `us2` , `gravel_fine_shallow_wet` , `gravel_fine_shallow` , `gravel_fine` , `gravel_wet` , `gravel` |
| 181 | BR2_Gravel_Coarse_Shallow_Dry | loose | dry | `br2_gravel_coarse_shallow_dry` , `br2_gravel_coarse_shallow` , `br2_gravel_coarse` , `br2_gravel` , `br2` , `gravel_coarse_shallow_dry` , `gravel_coarse_shallow` , `gravel_coarse` , `gravel` |
| 182 | BR2_Gravel_Coarse_Shallow_Damp | loose | damp | `br2_gravel_coarse_shallow_damp` , `br2_gravel_coarse_shallow` , `br2_gravel_coarse` , `br2_gravel` , `br2` , `gravel_coarse_shallow_damp` , `gravel_coarse_shallow` , `gravel_coarse` , `gravel_damp` , `gravel` |
| 183 | BR2_Gravel_Coarse_Shallow_Wet | loose | wet | `br2_gravel_coarse_shallow_wet` , `br2_gravel_coarse_shallow` , `br2_gravel_coarse` , `br2_gravel` , `br2` , `gravel_coarse_shallow_wet` , `gravel_coarse_shallow` , `gravel_coarse` , `gravel_wet` , `gravel` |
| 185 | US2_Gravel_Fine_Deep_Dry | loose | dry | `us2_gravel_fine_deep_dry` , `us2_gravel_fine_deep` , `us2_gravel_fine` , `us2_gravel` , `us2` , `gravel_fine_deep_dry` , `gravel_fine_deep` , `gravel_fine` , `gravel` |
| 186 | US2_Gravel_Fine_Deep_Damp | loose | damp | `us2_gravel_fine_deep_damp` , `us2_gravel_fine_deep` , `us2_gravel_fine` , `us2_gravel` , `us2` , `gravel_fine_deep_damp` , `gravel_fine_deep` , `gravel_fine` , `gravel_damp` , `gravel` |
| 187 | US2_Gravel_Fine_Deep_Wet | loose | wet | `us2_gravel_fine_deep_wet` , `us2_gravel_fine_deep` , `us2_gravel_fine` , `us2_gravel` , `us2` , `gravel_fine_deep_wet` , `gravel_fine_deep` , `gravel_fine` , `gravel_wet` , `gravel` |
| 189 | BR2_Gravel_Coarse_Deep_Dry | loose | dry | `br2_gravel_coarse_deep_dry` , `br2_gravel_coarse_deep` , `br2_gravel_coarse` , `br2_gravel` , `br2` , `gravel_coarse_deep_dry` , `gravel_coarse_deep` , `gravel_coarse` , `gravel` |
| 190 | BR2_Gravel_Coarse_Deep_Damp | loose | damp | `br2_gravel_coarse_deep_damp` , `br2_gravel_coarse_deep` , `br2_gravel_coarse` , `br2_gravel` , `br2` , `gravel_coarse_deep_damp` , `gravel_coarse_deep` , `gravel_coarse` , `gravel_damp` , `gravel` |
| 191 | BR2_Gravel_Coarse_Deep_Wet | loose | wet | `br2_gravel_coarse_deep_wet` , `br2_gravel_coarse_deep` , `br2_gravel_coarse` , `br2_gravel` , `br2` , `gravel_coarse_deep_wet` , `gravel_coarse_deep` , `gravel_coarse` , `gravel_wet` , `gravel` |
| 193 | US2_Gravel_Fine_Deeper_Dry | loose | dry | `us2_gravel_fine_deeper_dry` , `us2_gravel_fine_deeper` , `us2_gravel_fine` , `us2_gravel` , `us2` , `gravel_fine_deeper_dry` , `gravel_fine_deeper` , `gravel_fine` , `gravel` |
| 194 | US2_Gravel_Fine_Deeper_Damp | loose | damp | `us2_gravel_fine_deeper_damp` , `us2_gravel_fine_deeper` , `us2_gravel_fine` , `us2_gravel` , `us2` , `gravel_fine_deeper_damp` , `gravel_fine_deeper` , `gravel_fine` , `gravel_damp` , `gravel` |
| 195 | US2_Gravel_Fine_Deeper_Wet | loose | wet | `us2_gravel_fine_deeper_wet` , `us2_gravel_fine_deeper` , `us2_gravel_fine` , `us2_gravel` , `us2` , `gravel_fine_deeper_wet` , `gravel_fine_deeper` , `gravel_fine` , `gravel_wet` , `gravel` |
| 197 | BR2_Gravel_Coarse_Deeper_Dry | loose | dry | `br2_gravel_coarse_deeper_dry` , `br2_gravel_coarse_deeper` , `br2_gravel_coarse` , `br2_gravel` , `br2` , `gravel_coarse_deeper_dry` , `gravel_coarse_deeper` , `gravel_coarse` , `gravel` |
| 198 | BR2_Gravel_Coarse_Deeper_Damp | loose | damp | `br2_gravel_coarse_deeper_damp` , `br2_gravel_coarse_deeper` , `br2_gravel_coarse` , `br2_gravel` , `br2` , `gravel_coarse_deeper_damp` , `gravel_coarse_deeper` , `gravel_coarse` , `gravel_damp` , `gravel` |
| 199 | BR2_Gravel_Coarse_Deeper_Wet | loose | wet | `br2_gravel_coarse_deeper_wet` , `br2_gravel_coarse_deeper` , `br2_gravel_coarse` , `br2_gravel` , `br2` , `gravel_coarse_deeper_wet` , `gravel_coarse_deeper` , `gravel_coarse` , `gravel_wet` , `gravel` |
