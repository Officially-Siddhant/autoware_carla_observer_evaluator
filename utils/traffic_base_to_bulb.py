import numpy as np

def local_to_global(x_local, y_local, z_local, x_origin, y_origin, z_origin, yaw_deg):
    # Convert yaw to radians
    yaw_rad = np.deg2rad(yaw_deg)
    
    # Rotation matrix for yaw around Z-axis
    R = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [np.sin(yaw_rad),  np.cos(yaw_rad), 0],
        [0,                0,               1]
    ])
    
    # Local position as vector
    local_pos = np.array([x_local, y_local, z_local])
    
    # Rotate and translate
    global_pos = R @ local_pos + np.array([x_origin, y_origin, z_origin])

    # right hand to left hand

 
    return global_pos

if 1 == 1:
    # Example usage
    bulb_locals = [[0.519171, 0.049951, 3.275746],
                    [0.235171,	0.049951,	3.275746],
                    [-9.157929, 0.43005, 4.485746],
                    [-9.441929, 0.43005, 4.485746]              
                    ]

    x_origin, y_origin, z_origin = -119.230034, 5.093152, 0.262770
    yaw_deg = -90

    for bulb in bulb_locals:
        x_local, y_local, z_local = bulb
        global_position = local_to_global(x_local, y_local, z_local, x_origin, y_origin, z_origin, yaw_deg)
        print("Global Position:", global_position)


while 0==1:
    bulb_locals = [[-0.519171, 0.049951, 3.275746],
                    [-0.235171,	0.049951,	3.275746],
                    [9.157929, 0.43005, 4.485746],
                    [9.441929, 0.43005, 4.485746]              
                    ]
    origin_input = input("Enter x_origin, y_origin, z_origin, yaw_deg (comma-separated): ")
    
    try:
        x_origin, y_origin, z_origin, yaw_deg = map(float, origin_input.strip().split(","))
    except ValueError:
        print("Invalid input. Please enter four comma-separated values.")
        continue

    print(f"Using origin ({x_origin}, {y_origin}, {z_origin}) and yaw {yaw_deg}°")

    for bulb in bulb_locals:
        x_local, y_local, z_local = bulb
        global_position = local_to_global(x_local, y_local, z_local, x_origin, y_origin, z_origin, yaw_deg)
        print("  Global Position:", global_position)