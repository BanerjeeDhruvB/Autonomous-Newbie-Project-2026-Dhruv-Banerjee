# controller.py
#
# Faulty decision logic for the 2026 Autonomous Newbie Project.
# Recruits will mainly modify this file.
#
# Sign convention:
# lane_offset_m:
#   negative = vehicle is left of lane center
#   positive = vehicle is right of lane center
#
# heading_error_deg:
#   negative = vehicle heading points left of desired direction
#   positive = vehicle heading points right of desired direction
#
# Steering output semantics:
# "LEFT" means command the vehicle to steer / move left.
# "RIGHT" means command the vehicle to steer / move right.
# Therefore:
# - positive lane_offset_m means vehicle is right of center, so LEFT is corrective
# - positive heading_error_deg means vehicle points right of desired direction, so LEFT is corrective
import math

VALID_STEERING = {"LEFT", "RIGHT", "STRAIGHT"}
VALID_SPEED = {"ACCELERATE", "SLOW", "STOP"}


def controller(
    obstacle_distance_m,
    lane_offset_m,
    heading_error_deg,
    speed_mps,
    e_stop,
    left_clear,
    right_clear,
    sensor_valid
):
    """
    Returns:
        (steering, speed_action)

        steering:
            "LEFT", "RIGHT", "STRAIGHT"

        speed_action:
            "ACCELERATE", "SLOW", "STOP"
    """

    DANGER_OBSTACLE_M = 1.0
    CAUTION_OBSTACLE_M = 2.0

    MILD_HEADING_DEG = 3.0
    LARGE_HEADING_DEG = 15.0

    MILD_OFFSET_M = 0.15
    LARGE_OFFSET_M = 0.40

    HIGH_SPEED_MPS = 3.0

    #Constants for turning radius check
    OBS_WIDTH_M = 0.714
    OBS_LENGTH_M = 0.286

    TURN_SPEED_MPS = 1.5
    TURNING_RATE_DEG_PER_S = 35
    TURNING_RADIUS_APPROX = (TURN_SPEED_MPS)/(35*(3.14159/180))
    
    #Safety margin for staying away from obstacle. Increases with heading error
    SAFETY_MARGIN_M = obstacle_distance_m*math.sin(math.radians(abs(heading_error_deg))) + 0.1

    #Check if turning radius clears obstacle
    LEFT_TURN_CLEARS_OBS = False
    RIGHT_TURN_CLEARS_OBS = False
    if TURNING_RADIUS_APPROX**2-(obstacle_distance_m-OBS_LENGTH_M/2)**2 < 0:
        LEFT_TURN_CLEARS_OBS = False
    else: LEFT_TURN_CLEARS_OBS = math.sqrt(TURNING_RADIUS_APPROX**2-(obstacle_distance_m-OBS_LENGTH_M/2)**2)-TURNING_RADIUS_APPROX < -(lane_offset_m + OBS_WIDTH_M/2 + SAFETY_MARGIN_M)

    centered = abs(lane_offset_m) <= MILD_OFFSET_M
    small_heading_error = abs(heading_error_deg) <= MILD_HEADING_DEG

    steering = "STRAIGHT"
    speed_action = "ACCELERATE"

    if not sensor_valid:
        return "STRAIGHT", "STOP"

    if centered and small_heading_error:
        steering = "STRAIGHT"
        speed_action = "ACCELERATE"

    elif speed_mps >= HIGH_SPEED_MPS:
        if heading_error_deg > LARGE_HEADING_DEG or lane_offset_m > LARGE_OFFSET_M:
            steering = "LEFT"
            speed_action = "SLOW"

        elif heading_error_deg < -LARGE_HEADING_DEG or lane_offset_m < -LARGE_OFFSET_M:
            steering = "RIGHT"
            speed_action = "SLOW"

    if heading_error_deg > LARGE_HEADING_DEG or lane_offset_m > LARGE_OFFSET_M:
        steering = "LEFT"
        speed_action = "ACCELERATE"

    if heading_error_deg < -LARGE_HEADING_DEG or lane_offset_m < -LARGE_OFFSET_M:
        steering = "RIGHT"
        speed_action = "ACCELERATE"

    elif obstacle_distance_m <= DANGER_OBSTACLE_M:
        if not left_clear and not right_clear:
            steering = "STRAIGHT"
            speed_action = "STOP"

        elif left_clear and not right_clear:
            steering = "LEFT"
            speed_action = "SLOW"

        elif right_clear and not left_clear:
            steering = "RIGHT"
            speed_action = "SLOW"

        elif heading_error_deg > MILD_HEADING_DEG or lane_offset_m > MILD_OFFSET_M:
            steering = "LEFT"
            speed_action = "SLOW"

        elif heading_error_deg < -MILD_HEADING_DEG or lane_offset_m < -MILD_OFFSET_M:
            steering = "RIGHT"
            speed_action = "SLOW"

        else:
            steering = "LEFT"
            speed_action = "SLOW"

    elif obstacle_distance_m <= CAUTION_OBSTACLE_M:
        print(math.sqrt(TURNING_RADIUS_APPROX**2-(obstacle_distance_m-OBS_LENGTH_M/2)**2)-TURNING_RADIUS_APPROX)
        print(-(lane_offset_m + OBS_WIDTH_M/2 + SAFETY_MARGIN_M))
        print(LEFT_TURN_CLEARS_OBS)
        if not left_clear and not right_clear:
            steering = "STRAIGHT"
            speed_action = "STOP"

        elif (left_clear and not right_clear) and LEFT_TURN_CLEARS_OBS:
            steering = "LEFT"
            speed_action = "SLOW"

        elif right_clear and not left_clear:
            steering = "RIGHT"
            speed_action = "SLOW"

        elif heading_error_deg > MILD_HEADING_DEG or lane_offset_m > MILD_OFFSET_M:
            steering = "LEFT"
            speed_action = "SLOW"

        elif heading_error_deg < -MILD_HEADING_DEG or lane_offset_m < -MILD_OFFSET_M:
            steering = "RIGHT"
            speed_action = "SLOW"

        else:
            steering = "STRAIGHT"
            speed_action = "SLOW"

    if e_stop:
        steering = "STRAIGHT"
        speed_action = "STOP"

    return steering, speed_action
