import numpy as np
import cv2


#      Perspective Transform FUNCTIONS

def perspect_transform(img, src, dst):

    M = cv2.getPerspectiveTransform(src, dst)
    # keep same size as input image
    wr_pt = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))

    return wr_pt



#  Color Thresholding

def color_thresh(img, rgb_thresh=(160, 160, 160)):
    
    # Select all values from the RGB Red channel from the input img
    # Create a zero NP array with the same dementions as the Red channel
    color_select = np.zeros_like(img[:, :, 0])

    # Assign True to each pixel above input threshold values in RGB
    # Assign False to all pixels below the threshold
    in_thresh = (img[:, :, 0] > rgb_thresh[0]) \
        & (img[:, :, 1] > rgb_thresh[1]) \
        & (img[:, :, 2] > rgb_thresh[2])

    # Assign 1 to all True values in above_thresh
    color_select[in_thresh] = 1

    # Return binary img
    return color_select


# COORDINATES FUNCTIONS

def rover_coords(binary_img):

    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()

    # Calculate pixel positions with reference to the rover position
    # being at the center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1] / 2).astype(np.float)

    return x_pixel, y_pixel


def to_polar_coords(x_pixel, y_pixel):
    """Convert the [x_pixel, y_pixel] from rover_coords() to [distance, angle].
    Where each pixel position is represented by its distance from the origin
    and counterclockwise angle from the positive x-direction."""

    # Calculate distance from Rover to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)

    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)

    return dist, angles


def rotate_pix(xpix, ypix, yaw):
    """Maps rover space pixels to world space.
    Used for turning the Rover."""

    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180

    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))

    return xpix_rotated, ypix_rotated


def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    """Translate rover-centric coordinates back into world coordinates."""

    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos

    return xpix_translated, ypix_translated


def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    """Performs the rotation and translation actions required
    for Rover movement & mapping the environment."""

    # Apply rotation using the rotate_pix() function
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)

    # Apply translation using the translate_pix() function
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)

    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)

    return x_pix_world, y_pix_world


# ROCKS PART

def find_rocks(img, levels=(110, 110, 50)):
    # since rocks are yellow, they have high Red and Green & low Blue
    rockpix = ((img[:, :, 0] > levels[0])
               & (img[:, :, 1] > levels[1])
               & (img[:, :, 2] < levels[2]))

    color_select = np.zeros_like(img[:, :, 0])
    color_select[rockpix] = 1

    return color_select


# The MAIN function

def perception_step(Rover):

    dst_size, bottom_offset, image = 5, 6, Rover.img

    # Saving Start Position of Rover
    if(Rover.FirstFrame == True):
        Rover.FirstFrame = False
        Rover.startpos = Rover.pos
        print(f"X Start: {Rover.startpos[0]}")
        print(f"Y Start: {Rover.startpos[1]}")

    # Perform perception steps to update Rover()
    # 1) Define src and destination points for perspective transform
    src = np.float32([[14, 140], [300, 140], [200, 95], [120, 95]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                              [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                              [image.shape[1]/2 + dst_size, image.shape[0] - 2 * dst_size - bottom_offset],
                              [image.shape[1]/2 - dst_size, image.shape[0] - 2 * dst_size - bottom_offset]])
    
    # 2) Apply perspective transform
    wr_pt = perspect_transform(image, src, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    terrain_img = color_thresh(wr_pt, (150, 150, 150))
    rock_map = find_rocks(wr_pt, levels=(110, 110, 50))  
      
    # Old Implentation for Rock Detection
    
    # lower_yellow = np.array([24 - 5, 100, 100])
    # upper_yellow = np.array([24 + 5, 255, 255])
    #         # Convert BGR to HSV
    # hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    #         # Threshold the HSV image to get only upper_yellow colors
    # rock_samples = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # rock_samples = perspect_transform(rock_samples, src, destination)
    
    white = cv2.bitwise_not(np.zeros_like(image))
    warped_white = perspect_transform(white, src, destination)
    warped_white_threshed = color_thresh(warped_white, (1,1,1))  
    
    not_terrain = cv2.bitwise_not(terrain_img) # invert the terrain image 
    obstacle = cv2.bitwise_and(warped_white_threshed,not_terrain)
      
    # 4) Update Rover.vision_image (this will be displayed on left side of screen) 
    Rover.vision_image[:,:,0] = obstacle * 255 
    Rover.vision_image[:,:,2] = terrain_img * 255
    
    # 5) Convert map image pixel values to rover-centric coords
    x_pixel_rover, y_pixel_rover = rover_coords(terrain_img)  # terrain rover 
    x_pixel_obstacle,y_pixel_obstacle = rover_coords(obstacle) # obstacle rover
    x_pixel_rock,y_pixel_rock = rover_coords(rock_map)
    
    # 6) Convert rover-centric pixel values to world coordinates
    world_size = Rover.worldmap.shape[0]
    # scale = 2 * dst_size 
    scale = 3 * dst_size # Better fidelty
    
    navigable_x_world,navigable_y_world = pix_to_world(x_pixel_rover,
                                                      y_pixel_rover,Rover.pos[0],Rover.pos[1],
                                                      Rover.yaw,Rover.worldmap.shape[0],scale) 
                               
    obstacle_x_world,obstacle_navigable_y_world = pix_to_world(x_pixel_obstacle,
                                                               y_pixel_obstacle,Rover.pos[0],Rover.pos[1],
                                                               Rover.yaw,Rover.worldmap.shape[0],scale)

    rock_x_world,rock_y_world = pix_to_world(x_pixel_rock,y_pixel_rock,Rover.pos[0],
                                            Rover.pos[1],Rover.yaw,
                                            Rover.worldmap.shape[0],scale)
     
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    if (Rover.roll < 0.5):
        if(Rover.pitch < 0.5):
            Rover.worldmap[obstacle_navigable_y_world, obstacle_x_world, 0] = 255
            Rover.worldmap[rock_y_world, rock_x_world,1] = 255
            Rover.worldmap[navigable_y_world, navigable_x_world, 2] = 255
            nav_pix = Rover.worldmap[:, :, 2] > 0 # remove overlap mesurements
            Rover.worldmap[nav_pix, 0] = 0
            Rover.worldmap = np.clip(Rover.worldmap, 0, 255) # clip to avoid overflow
    
    # Find Rover Coordinates
    rock_x, rock_y = rover_coords(rock_map)

    # Find closest Rock to Rover
    rock_dist, rock_angles = to_polar_coords(rock_x, rock_y)

    if rock_map.any():
        # At Rock to World Map
        rock_x_world, rock_y_world = pix_to_world(
            rock_x, rock_y, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 255
        Rover.vision_image[:, :, 1] = rock_map * 225

    else:
        Rover.vision_image[:, :, 1] = 0

    if len(rock_dist) > 0:
        if Rover.mode == 'reverse':
            Rover.mode = 'reverse'
        else:
            Rover.mode = 'going_to_rock'
            Rover.rock_dists = rock_dist
            Rover.rock_angle = rock_angles
    else:
        Rover.nav_dists, Rover.nav_angles = to_polar_coords(x_pixel_rover, y_pixel_rover)

    return Rover
