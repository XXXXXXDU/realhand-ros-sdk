# hand_config_const.py
from typing import Dict, List, Optional
from dataclasses import dataclass, field
from types import MappingProxyType

@dataclass(frozen=True)        # frozen=True makes the instance truly read-only
class HandConfig:
    joint_names: List[str] = field(default_factory=list)
    joint_names_en: Optional[List[str]] = None
    init_pos: List[int] = field(default_factory=list)
    preset_actions: Optional[Dict[str, List[int]]] = None

# ------------------------------------------------------------------
# Example action constant dictionary (can be added according to the rules)
# ------------------------------------------------------------------
_HAND_CONFIGS: Dict[str, HandConfig] = {
    "L25": HandConfig(
        joint_names=["Thumb Root", "Index Root", "Middle Root", "Ring Root", "Little Root",
                     "Thumb Side", "Index Side", "Middle Side", "Ring Side", "Little Side",
                     "Thumb Roll", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb Middle", "Index Middle",
                     "Middle Middle", "Ring Middle", "Little Middle", "Thumb Tip", "Index Tip",
                     "Middle Tip", "Ring Tip", "Little Tip"],
        init_pos=[255] * 25,
        preset_actions={
            "Fist": [0] * 25,
            "Open": [255] * 25,
            "OK": [255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                   255, 255, 255, 255, 255, 255, 0, 0, 255, 255,
                   0, 0, 0, 255, 255]
        }
    ),
    "L21": HandConfig(
        joint_names=["Thumb Root", "Index Root", "Middle Root", "Ring Root", "Little Root",
                     "Thumb Side", "Index Side", "Middle Side", "Ring Side", "Little Side",
                     "Thumb Roll", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb Middle", "Reserved",
                     "Reserved", "Reserved", "Reserved", "Thumb Tip", "Index Tip", "Middle Tip",
                     "Ring Tip", "Little Tip"],
        init_pos=[255] * 25
    ),
    "L20": HandConfig(
        joint_names=["Thumb Root", "Index Root", "Middle Root", "Ring Root", "Little Root",
                     "Thumb Side", "Index Side", "Middle Side", "Ring Side", "Little Side",
                     "Thumb Yaw", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb Tip", "Index Tip",
                     "Middle Tip", "Ring Tip", "Little Tip"],
        init_pos=[255, 255, 255, 255, 255, 255, 10, 100, 180, 240, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255],
        preset_actions={
            "Fist": [40, 0, 0, 0, 0, 131, 10, 100, 180, 240, 19, 255, 255, 255, 255, 135, 0, 0, 0, 0],
            "Open": [255, 255, 255, 255, 255, 255, 10, 100, 180, 240, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "OK": [191, 95, 255, 255, 255, 136, 107, 100, 180, 240, 72, 255, 255, 255, 255, 116, 99, 255, 255, 255],
            "Thumbs Up": [255, 0, 0, 0, 0, 127, 10, 100, 180, 240, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0],
            "Thumb to Index": [0, 0, 255, 255, 255, 186, 10, 100, 180, 240, 0, 255, 255, 255, 255, 255, 183, 255, 255, 255],
            "Thumb to Middle": [0, 255, 0, 255, 255, 145, 10, 100, 180, 240, 0, 255, 255, 255, 255, 255, 255, 202, 255, 255],
            "Thumb to Ring": [0, 255, 255, 0, 255, 108, 10, 100, 180, 240, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "Thumb to Little": [0, 255, 255, 255, 0, 70, 10, 100, 180, 240, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "Ready1": [255, 255, 255, 255, 255, 255, 10, 100, 180, 240, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "One": [40, 255, 0, 0, 0, 131, 125, 100, 180, 240, 19, 255, 255, 255, 255, 135, 255, 0, 0, 0],
            "Two": [40, 255, 255, 0, 0, 81, 35, 177, 180, 240, 19, 255, 255, 255, 255, 135, 255, 255, 0, 0],
            "Three": [40, 255, 255, 255, 0, 161, 62, 123, 180, 240, 13, 255, 255, 255, 255, 0, 255, 255, 255, 0],
            "Four": [40, 255, 255, 255, 255, 161, 62, 123, 180, 242, 13, 255, 255, 255, 255, 0, 255, 255, 255, 255],
            "Five": [255, 255, 255, 255, 255, 255, 10, 100, 180, 240, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "Six": [255, 0, 0, 0, 255, 220, 10, 100, 180, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 255],
            "Seven": [0, 0, 0, 0, 0, 161, 10, 127, 180, 219, 18, 255, 255, 255, 255, 255, 195, 205, 0, 0],
            "Eight": [255, 255, 0, 0, 0, 202, 104, 100, 180, 240, 233, 255, 255, 255, 255, 255, 255, 0, 0, 0],
            "Nine": [40, 255, 0, 0, 0, 131, 103, 100, 180, 240, 19, 255, 255, 255, 255, 135, 47, 0, 0, 0],
        }
    ),
    "G20": HandConfig(
        joint_names=["Thumb Root", "Index Root", "Middle Root", "Ring Root", "Little Root",
                     "Thumb Side", "Index Side", "Middle Side", "Ring Side", "Little Side",
                     "Thumb Yaw", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb Tip", "Index Tip",
                     "Middle Tip", "Ring Tip", "Little Tip"],
        init_pos=[255, 255, 255, 255, 255, 255, 193, 148, 105, 42, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255],
        preset_actions={
            "Thumbs Up": [255, 0, 0, 0, 0, 255, 162, 162, 144, 100, 210, 255, 255, 255, 255, 255, 0, 0, 0, 0],
            "Fist": [96, 0, 0, 0, 0, 0, 193, 158, 128, 91, 132, 255, 255, 255, 255, 144, 0, 0, 0, 0],
            "Open": [255, 255, 255, 255, 255, 255, 193, 148, 105, 42, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "OK": [148, 110, 255, 255, 255, 44, 164, 100, 114, 127, 178, 255, 255, 255, 255, 94, 71, 255, 255, 255],
            "Thumb to Middle": [191, 255, 55, 255, 255, 96, 95, 100, 114, 127, 105, 255, 255, 255, 255, 94, 255, 108, 255, 255],
            "Thumb to Ring": [191, 255, 255, 72, 255, 115, 95, 100, 114, 127, 60, 255, 255, 255, 255, 94, 255, 255, 97, 255],
            "Thumb to Little": [191, 255, 255, 255, 55, 0, 95, 100, 114, 121, 70, 255, 255, 255, 255, 94, 255, 255, 255, 100],
            "Ready1": [255, 0, 0, 0, 0, 255, 162, 162, 144, 100, 210, 255, 255, 255, 255, 255, 0, 0, 0, 0],
            "One": [96, 255, 0, 0, 0, 0, 190, 161, 127, 80, 68, 255, 255, 255, 255, 144, 255, 0, 0, 0],
            "Two": [96, 255, 255, 0, 0, 0, 190, 66, 127, 80, 68, 255, 255, 255, 255, 144, 255, 255, 0, 0],
            "Three": [96, 255, 255, 255, 0, 0, 200, 132, 76, 80, 68, 255, 255, 255, 255, 144, 255, 255, 255, 0],
            "Four": [80, 255, 255, 255, 255, 78, 200, 132, 114, 48, 129, 255, 255, 255, 255, 64, 255, 255, 255, 255],
            "Five": [255, 255, 255, 255, 255, 255, 193, 148, 105, 42, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "Six": [255, 0, 0, 0, 255, 255, 156, 126, 125, 42, 245, 255, 255, 255, 255, 255, 0, 0, 0, 255],
            "Seven": [38, 0, 0, 0, 0, 55, 156, 126, 125, 117, 145, 255, 255, 255, 255, 255, 164, 163, 0, 0],
            "Eight": [255, 255, 0, 0, 0, 255, 162, 162, 144, 100, 210, 255, 255, 255, 255, 255, 255, 0, 0, 0],
            "Nine": [67, 255, 0, 0, 0, 37, 162, 162, 144, 100, 85, 255, 255, 255, 255, 169, 0, 0, 0, 0],
            "Action1": [255, 255, 255, 255, 255, 255, 193, 148, 105, 42, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "Action2": [255, 255, 255, 255, 255, 255, 125, 129, 125, 130, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "Action3": [255, 255, 255, 255, 255, 255, 193, 148, 105, 42, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "Action4": [255, 255, 255, 255, 255, 255, 125, 129, 125, 130, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "Action5": [255, 255, 255, 255, 255, 255, 255, 255, 247, 255, 210, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "Action6": [255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 210, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "Action7": [255, 255, 255, 255, 255, 255, 255, 255, 247, 255, 210, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "Action8": [255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 210, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "Action9": [255, 255, 255, 255, 255, 255, 125, 129, 125, 130, 210, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "Root1": [0, 0, 0, 0, 0, 255, 125, 129, 125, 130, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "Root2": [255, 255, 255, 255, 255, 255, 125, 129, 125, 130, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "Tip1": [6, 0, 0, 0, 0, 255, 125, 129, 125, 130, 219, 255, 255, 255, 255, 125, 0, 0, 0, 0],
            "Tip2": [6, 0, 0, 0, 0, 255, 125, 129, 125, 130, 219, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "Tip3": [6, 0, 0, 0, 0, 255, 125, 129, 125, 130, 219, 255, 255, 255, 255, 125, 0, 0, 0, 0],
            "Tip4": [6, 0, 0, 0, 0, 255, 125, 129, 125, 130, 219, 255, 255, 255, 255, 255, 255, 255, 255, 255],
            "Default": [255, 255, 255, 255, 255, 255, 193, 148, 105, 42, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255]


        }
    ),
    # L10 with spherical thumb joint
    # "L10": HandConfig(
    #     joint_names_en=["thumb_cmc_pitch", "thumb_cmc_roll", "index_mcp_pitch", "middle_mcp_pitch", "ring_mcp_pitch", "pinky_mcp_pitch",
    #                     "index_mcp_roll", "ring_mcp_roll", "pinky_mcp_roll", "thumb_cmc_yaw"],
    #     joint_names=["Thumb Root", "Thumb Side", "Index Root", "Middle Root", "Ring Root",
    #                  "Little Root", "Index Side", "Ring Side", "Little Side", "Thumb Roll"],
    #     init_pos=[255] * 10,
    #     preset_actions={
    #         "Fist": [75, 128, 0, 0, 0, 0, 128, 128, 128, 57],
    #         "Open": [255, 128, 255, 255, 255, 255, 128, 128, 128, 128],
    #         "OK": [110, 128, 75, 255, 255, 255, 128, 128, 128, 68],
    #         "Thumbs Up": [255, 145, 0, 0, 0, 0, 0, 255, 255, 65]
    #     }
    # ),
    # L10 with gear thumb joint
    "L10": HandConfig(
        joint_names_en=["thumb_cmc_pitch", "thumb_cmc_roll", "index_mcp_pitch", "middle_mcp_pitch", "ring_mcp_pitch", "pinky_mcp_pitch",
                        "index_mcp_roll", "ring_mcp_roll", "pinky_mcp_roll", "thumb_cmc_yaw"],
        joint_names=["Thumb Root", "Thumb Side", "Index Root", "Middle Root", "Ring Root",
                     "Little Root", "Index Side", "Ring Side", "Little Side", "Thumb Roll"],
        init_pos=[255] * 10,
        preset_actions={
            "Open": [255, 255, 255, 255, 255, 255, 128, 67, 89, 255],
            "Thumbs Up": [255, 255, 0, 0, 0, 0, 128, 67, 89, 255],
            "Fist": [90, 0, 0, 0, 0, 0, 128, 67, 89, 197],
            "One": [55, 0, 255, 0, 0, 0, 128, 67, 89, 124],
            "Two": [55, 0, 255, 255, 0, 0, 128, 67, 89, 124],
            "Three": [116, 255, 255, 255, 255, 0, 128, 67, 89, 255],
            "Four": [0, 0, 255, 255, 255, 255, 128, 67, 89, 255],
            "Five": [255, 255, 255, 255, 255, 255, 128, 67, 89, 255],
            "Six": [255, 255, 0, 0, 0, 255, 128, 67, 89, 255],
            "Seven1": [255, 37, 119, 112, 0, 0, 128, 67, 89, 211],
            "Seven2": [91, 37, 119, 112, 0, 0, 128, 67, 89, 211],
            "Eight": [255, 255, 255, 0, 0, 0, 128, 67, 89, 255],
            "Nine": [59, 0, 134, 0, 0, 0, 128, 67, 89, 153],
            "Side_Swing0": [255, 0, 255, 255, 255, 255, 128, 67, 89, 153],
            "Side_Swing1": [0, 0, 255, 255, 255, 255, 255, 255, 255, 255],
            "Side_Swing2": [0, 0, 255, 255, 255, 255, 0, 0, 0, 255],
            "Side_Swing3": [0, 0, 255, 255, 255, 255, 255, 255, 255, 255],
            "Side_Swing4": [0, 0, 255, 255, 255, 255, 0, 0, 0, 255],
            "Side_Swing5": [0, 0, 255, 255, 255, 255, 255, 255, 255, 255],
            "Side_Swing6": [0, 0, 255, 255, 255, 255, 128, 67, 89, 255],
            "Flex1": [209, 0, 124, 122, 123, 122, 128, 67, 89, 255],
            "Flex2": [0, 0, 255, 255, 255, 255, 128, 67, 89, 255],
            "Flex3": [209, 0, 124, 122, 123, 122, 128, 67, 89, 255],
            "Flex4": [0, 0, 255, 255, 255, 255, 128, 67, 89, 255],
            "Flex5": [209, 0, 124, 122, 123, 122, 128, 67, 89, 255],
            "Flex6": [0, 0, 255, 255, 255, 255, 128, 67, 89, 255],
            "OK": [84, 39, 122, 255, 255, 255, 128, 67, 89, 255],
            "Thumb_Pressure1": [134, 39, 99, 255, 255, 255, 128, 67, 89, 255],
            "Thumb_Pressure2": [73, 39, 91, 255, 255, 255, 128, 67, 89, 255],
            "Index_Pressure1": [151, 39, 190, 255, 255, 255, 128, 67, 89, 255],
            "Index_Pressure2": [58, 39, 103, 255, 255, 255, 128, 67, 89, 255],
            "Middle_Pressure1": [40, 39, 255, 128, 255, 255, 128, 67, 89, 209],
            "Middle_Pressure2": [40, 39, 255, 89, 255, 255, 128, 67, 89, 209],
            "Ring_Pressure1": [51, 39, 255, 255, 139, 255, 128, 67, 89, 154],
            "Ring_Pressure2": [51, 39, 255, 255, 83, 255, 128, 67, 89, 154],
            "Little_Pressure1": [62, 39, 255, 255, 255, 155, 128, 67, 89, 101],
            "Little_Pressure2": [62, 39, 255, 255, 255, 75, 128, 67, 89, 101],
        }
    ),
    # L7 with spherical thumb joint
    # "L7": HandConfig(
    #     joint_names=["Thumb Flexion", "Thumb Yaw", "Index Flexion", "Middle Flexion", "Ring Flexion",
    #                  "Little Flexion", "Thumb Roll"],
    #     init_pos=[250] * 7,
    #     preset_actions={
    #         "Thumbs Up": [255, 111, 0, 0, 0, 0, 86],
    #         "Fist": [71, 79, 0, 0, 0, 0, 64],
    #         "Open": [255, 111, 250, 250, 250, 250, 55],
    #         "OK": [141, 111, 168, 250, 250, 250, 86],
            
    #     }
    # ),
    # L7 with gear thumb joint
    "L7": HandConfig(
        joint_names=["Thumb Flexion", "Thumb Yaw", "Index Flexion", "Middle Flexion", "Ring Flexion",
                     "Little Flexion", "Thumb Roll"],
        init_pos=[250] * 7,
        preset_actions={
            "Open": [255, 111, 250, 250, 250, 250, 55],
            "Thumbs Up": [255, 255, 0, 0, 0, 0, 255],
            "Thumbs Up 2": [255, 0, 0, 0, 0, 0, 255],
            "Fist": [65, 0, 0, 0, 0, 0, 93],
            "One": [66, 0, 255, 0, 0, 0, 93],
            "One-1": [61, 0, 255, 0, 0, 0, 255],
            "Two": [0, 0, 255, 255, 0, 0, 255],
            "Three": [0, 0, 255, 255, 255, 0, 255],
            "Four": [0, 0, 255, 255, 255, 255, 119],
            "Five": [255, 111, 250, 250, 250, 250, 55],
            "OK": [99, 15, 146, 250, 250, 250, 206],
            "Thumb_Pressure1": [99, 15, 206, 250, 250, 250, 206],
            "Thumb_Pressure2": [109, 15, 70, 250, 250, 250, 206],
            "Index_Pressure1": [99, 15, 206, 250, 250, 250, 206],
            "Index_Pressure2": [69, 15, 140, 250, 250, 250, 206],
            "Middle_Pressure": [82, 15, 255, 136, 250, 250, 170],
            "Ring_Pressure": [70, 15, 255, 255, 141, 250, 125],
            "Little_Pressure": [70, 15, 255, 255, 255, 120, 78],
            "Ready1": [70, 15, 255, 255, 255, 255, 78]
            
        }
    ),
    "O6": HandConfig(
        joint_names_en=["thumb_cmc_pitch", "thumb_cmc_yaw", "index_mcp_pitch", "middle_mcp_pitch", "pinky_mcp_pitch", "ring_mcp_pitch"],
        joint_names=["Thumb Flexion", "Thumb Yaw", "Index Flexion", "Middle Flexion", "Ring Flexion", "Little Flexion"],
        init_pos=[250] * 6,
        preset_actions={
            "Open": [250, 250, 250, 250, 250, 250],
            "One": [125, 18, 255, 0, 0, 0],
            "Two": [92, 87, 255, 255, 0, 0],
            "Three": [92, 87, 255, 255, 255, 0],
            "Four": [92, 87, 255, 255, 255, 255],
            "Five": [255, 255, 255, 255, 255, 255],
            "OK": [96, 100, 118, 250, 250, 250],
            "Thumbs Up": [250, 79, 0, 0, 0, 0],
            "Fist": [102, 18, 0, 0, 0, 0],
        }
    ),
    "L6": HandConfig(
        joint_names_en=["thumb_cmc_pitch", "thumb_cmc_yaw", "index_mcp_pitch", "middle_mcp_pitch", "pinky_mcp_pitch", "ring_mcp_pitch"],
        joint_names=["Thumb Flexion", "Thumb Yaw", "Index Flexion", "Middle Flexion", "Ring Flexion", "Little Flexion"],
        init_pos=[250] * 6,
        preset_actions={
            "Open": [250, 250, 250, 250, 250, 250],
            "One": [0, 18, 255, 0, 0, 0],
            "Two": [0, 39, 255, 255, 0, 0],
            "Three": [0, 39, 255, 255, 255, 0],
            "Four": [0, 0, 255, 255, 255, 255],
            "Five": [255, 255, 255, 255, 255, 255],
            "OK": [74, 13, 153, 255, 255, 255],
            "Thumbs Up": [255, 255, 0, 0, 0, 0],
            "Fist": [79, 11, 0, 0, 0, 0],
            "Sequence1": [250, 250, 250, 250, 250, 250],
            "Sequence2": [250, 250, 0, 250, 250, 0],
            "Sequence3": [250, 250, 0, 0, 0, 0],
            "Sequence4": [250, 250, 0, 0, 0, 255],
            "Sequence5": [250, 250, 0, 0, 255, 255],
            "Sequence6": [250, 250, 0, 255, 255, 255],
            "Sequence7": [250, 250, 250, 250, 250, 250],
            "Index_Pressure_Ready1": [0, 18, 255, 0, 0, 0],
            "Index_Pressure_Test": [9, 42, 55, 250, 250, 250],
            "Index_Pressure_Ready2": [0, 18, 255, 0, 0, 0],
            "Thumb_Pressure_Ready1": [139, 18, 130, 0, 0, 0],
            "Thumb_Pressure_Test": [39, 30, 122, 250, 250, 250],
            "Thumb_Pressure_Ready2": [139, 18, 130, 0, 0, 0]
        }
    ),
}
HAND_CONFIGS = MappingProxyType(_HAND_CONFIGS)
