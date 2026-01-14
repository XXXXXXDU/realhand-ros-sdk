CONFIG = {
    # Default values (if needed, but the default in the code is 5 elements, special values are 6, 7, 10, 20, 21/25 elements)
    # "DEFAULT": {
    #     "pose": None,
    #     "torque": [200, 200, 200, 200, 200],
    #     "speed": [80, 200, 200, 200, 200]
    # },
    
    # 6-axis/joint configuration
    "O6": {
        "pose": [255, 200, 255, 255, 255, 255],
        "torque": [250, 250, 250, 250, 250, 250],
        "speed": [255, 255, 255, 255, 255, 255]
    },
    "L6": { # Same as O6, a function can be used to handle the repetitive logic
        "pose": [255, 200, 255, 255, 255, 255],
        "torque": [250, 250, 250, 250, 250, 250],
        "speed": [255, 255, 255, 255, 255, 255]
    },
    "L6P": { # 与 O6 相同
        "pose": [255, 200, 255, 255, 255, 255],
        "torque": [250, 250, 250, 250, 250, 250],
        "speed": [255, 255, 255, 255, 255, 255]
    },

    # 7-axis/joint configuration
    "L7": {
        "pose": [255, 200, 255, 255, 255, 255, 180],
        "torque": [250, 250, 250, 250, 250, 250, 250],
        "speed": [120, 180, 180, 180, 180, 180, 180]
    },

    # 10-axis/joint configuration
    "L10": {
        "pose": [255, 200, 255, 255, 255, 255, 180, 180, 180, 41],
        "torque": [250, 250, 250, 250, 250, 250, 250, 250, 250, 250],
        "speed": [120, 180, 180, 180, 180, 180, 180, 180, 180, 180]
    },

    # 20-axis/joint configuration (Note: torque and speed for L20 and G20 are not explicitly defined in the original code, the default values from the original code are kept here)
    "L20": {
        "pose": [255, 255, 255, 255, 255, 255, 10, 100, 180, 240, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255],
    },
    "G20": {
        "pose": [241, 255, 255, 255, 255, 255, 141, 134, 149, 137, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255],
    },

    # 21/25-axis/joint configuration (L21 and L25 have the same pose)
    "L21": {
        "pose": [75, 255, 255, 255, 255, 176, 97, 81, 114, 147, 202, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255],
    },
    "L25": {
        "pose": [75, 255, 255, 255, 255, 176, 97, 81, 114, 147, 202, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255],
    }
}