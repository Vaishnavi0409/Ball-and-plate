import time
import cv2
import numpy as np
import serial
from collections import deque
from PID_controller import PIDController 

BOARD_DIMENSIONS = (500, 625)

SERIAL_ENABLED = True

#Constants for visual offset
PIXELS_PER_CM = 15
OFFSET_CM = 1
OFFSET_PX = int(OFFSET_CM * PIXELS_PER_CM)

#Predefined Color Range for the Board (UPDATED TO MATCH IMAGE) 
#These values are derived from the 'Color Tuning' window in the provided image.
LOWER_YELLOW = np.array([7, 63, 121]) 
UPPER_YELLOW = np.array([32, 255, 255]) 


#Initialize PID controllers
pid_x = PIDController(Kp=1.2, Ki=0.05, Kd=0.2, setpoint=(BOARD_DIMENSIONS[0] / 2) + OFFSET_PX)
pid_y = PIDController(Kp=1.2, Ki=0.05, Kd=0.2, setpoint=(BOARD_DIMENSIONS[1] / 2) + OFFSET_PX)


#Temporal smoothing for board corners 
corner_history = deque(maxlen=5)
last_valid_corners = None

#Ball HSV Color Range (will be updated by trackbars) ---
ball_hsv_lower = np.array([25, 0, 80])
ball_hsv_upper = np.array([105, 83, 255])


if SERIAL_ENABLED:
    try:
        ser = serial.Serial('/dev/cu.usbmodem12301', 9600, timeout=1)
        print("Serial connection established")
    except Exception as e:
        print(f"Failed to connect to serial port: {e}")
        SERIAL_ENABLED = False

def nothing(x):
    pass

def update_pid_from_sliders():
    
    pid_x.Kp = cv2.getTrackbarPos("Kp_X", "PID Tuning") / 100.0
    pid_x.Ki = cv2.getTrackbarPos("Ki_X", "PID Tuning") / 1000.0
    pid_x.Kd = cv2.getTrackbarPos("Kd_X", "PID Tuning") / 100.0

    pid_y.Kp = cv2.getTrackbarPos("Kp_Y", "PID Tuning") / 100.0
    pid_y.Ki = cv2.getTrackbarPos("Ki_Y", "PID Tuning") / 1000.0
    pid_y.Kd = cv2.getTrackbarPos("Kd_Y", "PID Tuning") / 100.0

def update_ball_hsv_from_sliders():
    global ball_hsv_lower, ball_hsv_upper
    h_min = cv2.getTrackbarPos("H_min_ball", "PID Tuning")
    s_min = cv2.getTrackbarPos("S_min_ball", "PID Tuning")
    v_min = cv2.getTrackbarPos("V_min_ball", "PID Tuning")
    h_max = cv2.getTrackbarPos("H_max_ball", "PID Tuning")
    s_max = cv2.getTrackbarPos("S_max_ball", "PID Tuning")
    v_max = cv2.getTrackbarPos("V_max_ball", "PID Tuning")
    ball_hsv_lower = np.array([h_min, s_min, v_min])
    ball_hsv_upper = np.array([h_max, s_max, v_max])


def smooth_corners(new_corners):
    """Apply temporal smoothing to corner positions to reduce jitter."""
    global corner_history, last_valid_corners
    
    if new_corners is None:
        return last_valid_corners
    
    corner_history.append(new_corners)
    
    smoothed = np.mean(corner_history, axis=0)
    
    last_valid_corners = smoothed.astype(np.float32)
    return last_valid_corners

def order_points_improved(pts):
    """Order points: top-left, top-right, bottom-right, bottom-left"""
    pts = pts.astype(np.float32)
    center = np.mean(pts, axis=0)

    rect = np.zeros((4, 2), dtype=np.float32)
    top_pts = pts[pts[:, 1] < center[1]]
    bottom_pts = pts[pts[:, 1] >= center[1]]

    if len(top_pts) == 2 and len(bottom_pts) == 2:
        top_sorted = top_pts[np.argsort(top_pts[:, 0])]
        bottom_sorted = bottom_pts[np.argsort(bottom_pts[:, 0])]
        rect[0], rect[1], rect[2], rect[3] = top_sorted[0], top_sorted[1], bottom_sorted[1], bottom_sorted[0]
    else:
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)
        rect[0], rect[2], rect[1], rect[3] = pts[np.argmin(s)], pts[np.argmax(s)], pts[np.argmin(diff)], pts[np.argmax(diff)]

    return rect

def detect_yellow_board_robust(frame, lower_bound, upper_bound, output_size=BOARD_DIMENSIONS, debug=False):
    if frame is None:
        return None, None, None, None

    original_frame = frame.copy()
    mask = None

    try:
        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        current_mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        mask = cv2.morphologyEx(current_mask, cv2.MORPH_CLOSE, kernel, iterations=4)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return original_frame, None, None, mask
        
        largest_contour = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest_contour) < 5000:
             return original_frame, None, None, mask

        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        raw_corners = order_points_improved(box).astype(np.float32)
        
        corners = smooth_corners(raw_corners)
        
        if corners is None:
            return original_frame, None, None, mask
        
        out_w, out_h = output_size
        expanded_w, expanded_h = out_w + 2 * OFFSET_PX, out_h + 2 * OFFSET_PX
        dst = np.array([
            [OFFSET_PX, OFFSET_PX], [out_w - 1 + OFFSET_PX, OFFSET_PX],
            [out_w - 1 + OFFSET_PX, out_h - 1 + OFFSET_PX], [OFFSET_PX, out_h - 1 + OFFSET_PX]
        ], dtype=np.float32)

        M = cv2.getPerspectiveTransform(corners, dst)
        warped = cv2.warpPerspective(original_frame, M, (expanded_w, expanded_h), 
                                     flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
        return warped, corners, None, mask
    except Exception as e:
        return original_frame, None, None, mask

def detect_yellow_board_with_validation(frame, lower_bound, upper_bound, output_size=BOARD_DIMENSIONS):
    if frame is None: return frame, None, None
    warped, corners, _, mask = detect_yellow_board_robust(frame, lower_bound, upper_bound, output_size, debug=False)
    if corners is not None and warped is not None:
        try:
            gray_warped = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
            if cv2.countNonZero(gray_warped) > 0.1 * gray_warped.size:
                return warped, corners, mask
        except Exception as e: print(f"Validation failed: {e}")
    return frame, None, mask

def detect_ball(frame, lower_bound, upper_bound, debug=False):
    """
    Detects the ball using a robust hybrid method: a user-tunable color filter
    followed by a rigid shape (circularity) analysis.
    """
    if frame is None: return None, None, None
    try:
        output_frame = frame.copy()
        
        # 1. Blur and convert to HSV color space
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # 2. Use the HSV range provided by the trackbars
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # 3. Clean the mask to remove noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Show the mask for debugging
        cv2.imshow("Ball Mask", mask)

        # 4. Find contours in the cleaned mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_candidate = None
        best_circularity = 0
        
        if contours:
            for cnt in contours:
                # 5. Apply a strict area filter
                area = cv2.contourArea(cnt)
                if not (300 < area < 15000):
                    continue

                # 6. Calculate circularity
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue
                
                circularity = 4 * np.pi * (area / (perimeter * perimeter))
                
                # 7. Only consider highly circular contours (relaxed slightly to 0.75)
                if circularity > 0.75:
                    if circularity > best_circularity:
                        ((x, y), radius) = cv2.minEnclosingCircle(cnt)
                        best_circularity = circularity
                        best_candidate = ((int(x), int(y)), int(radius))

        ball_center, ball_radius = best_candidate if best_candidate else (None, None)
        
        # Draw the target (blue dot) at the center of the board
        center_x, center_y = output_frame.shape[1] // 2, output_frame.shape[0] // 2
        cv2.circle(output_frame, (center_x, center_y), 8, (255, 100, 0), -1)
        
        return output_frame, ball_center, ball_radius
    except Exception as e:
        print(f"Ball detection error: {e}")
        return frame, None, None


def ball_position_pid(ball_center, frame_shape, pid_x, pid_y, dt=0.033):
    """Apply PID control to ball position"""
    control_x, control_y = 0.0, 0.0
    if ball_center and dt > 0:
        x, y = ball_center
        control_x, control_y = pid_x.update(x, dt), pid_y.update(y, dt)
        control_x, control_y = max(-100, min(100, control_x)), max(-100, min(100, control_y))
    return control_x, control_y

def calculate_circular_setpoint(start_time, board_shape):
    """Calculates the target (x, y) point for the ball to trace a circle."""
    center_x = (board_shape[0] / 2) + OFFSET_PX
    center_y = (board_shape[1] / 2) + OFFSET_PX
    RADIUS = 140  
    SPEED = 0.2   

    elapsed_time = time.time() - start_time
    angle = elapsed_time * SPEED * 2 * np.pi

    target_x = center_x + RADIUS * np.cos(angle)
    target_y = center_y + RADIUS * np.sin(angle)

    return int(target_x), int(target_y)

def calculate_figure_eight_setpoint(start_time, board_shape):
    """Calculates the target (x, y) point for the ball to trace a figure-eight."""
    center_x = (board_shape[0] / 2) + OFFSET_PX
    center_y = (board_shape[1] / 2) + OFFSET_PX
    RADIUS_X = 140  
    RADIUS_Y = 180  
    SPEED = 0.15

    elapsed_time = time.time() - start_time
    angle = elapsed_time * SPEED * np.pi

    # Parametric equations for a Lissajous curve shaped like a figure-eight
    target_x = center_x + RADIUS_X * np.cos(angle)
    target_y = center_y + RADIUS_Y * np.sin(2 * angle)

    return int(target_x), int(target_y)

def main():
    global last_valid_corners

    cap = cv2.VideoCapture(0)
    # Give the camera time to initialize
    time.sleep(1.0) 
    
    if not cap.isOpened():
        print("Error: Could not open video stream.")
        print("Please check the following:")
        print("1. Is the camera connected and turned on?")
        print("2. Is another application (e.g., Photo Booth, Zoom) using the camera?")
        print("3. On macOS, has the terminal been granted camera permissions?")
        print("4. Try changing cv2.VideoCapture(0) to cv2.VideoCapture(1) or higher.")
        return
        
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    cv2.namedWindow("PID Tuning")
    cv2.createTrackbar("Kp_X", "PID Tuning", 80, 500, nothing)
    cv2.createTrackbar("Ki_X", "PID Tuning", 80, 100, nothing)
    cv2.createTrackbar("Kd_X", "PID Tuning", 25, 200, nothing)
    cv2.createTrackbar("Kp_Y", "PID Tuning", 80, 500, nothing)
    cv2.createTrackbar("Ki_Y", "PID Tuning", 80, 100, nothing)
    cv2.createTrackbar("Kd_Y", "PID Tuning", 25, 200, nothing)
    cv2.createTrackbar("Circle Mode", "PID Tuning", 0, 1, nothing)
    cv2.createTrackbar("Figure 8 Mode", "PID Tuning", 0, 1, nothing)

    # Ball HSV Color Tuning Controls
    cv2.createTrackbar("H_min_ball", "PID Tuning", 25, 179, nothing)
    cv2.createTrackbar("S_min_ball", "PID Tuning", 0, 255, nothing)
    cv2.createTrackbar("V_min_ball", "PID Tuning", 80, 255, nothing)
    cv2.createTrackbar("H_max_ball", "PID Tuning", 105, 179, nothing)
    cv2.createTrackbar("S_max_ball", "PID Tuning", 83, 255, nothing)
    cv2.createTrackbar("V_max_ball", "PID Tuning", 255, 255, nothing)


    last_time = time.time()
    last_print_time = time.time()
    circle_mode_start_time = 0
    figure_eight_mode_start_time = 0 
    
    print("Starting ball tracking system...")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: 
            print("Error: Could not read frame. Exiting.")
            break
            
        now, dt = time.time(), time.time() - last_time
        last_time = now

        cropped_frame, corners, board_mask = detect_yellow_board_with_validation(frame, LOWER_YELLOW, UPPER_YELLOW)
        
        display_frame = frame.copy()
        if corners is not None:
            cv2.polylines(display_frame, [corners.astype(int)], True, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow('Original Frame', display_frame)
        if board_mask is not None: cv2.imshow("Board Mask", board_mask)

        if cropped_frame is not None:
            cv2.imshow('Warped Board', cropped_frame)
        else:
            expanded_w, expanded_h = BOARD_DIMENSIONS[0] + 2 * OFFSET_PX, BOARD_DIMENSIONS[1] + 2 * OFFSET_PX
            cv2.imshow('Warped Board', np.zeros((expanded_h, expanded_w, 3), dtype=np.uint8))

        update_pid_from_sliders()
        update_ball_hsv_from_sliders()

        balled_frame, final_ball_pos, ball_radius = detect_ball(cropped_frame, ball_hsv_lower, ball_hsv_upper)

        circle_mode_on = cv2.getTrackbarPos("Circle Mode", "PID Tuning") == 1
        figure_eight_mode_on = cv2.getTrackbarPos("Figure 8 Mode", "PID Tuning") == 1
        
        
        if figure_eight_mode_on:
            if figure_eight_mode_start_time == 0:
                figure_eight_mode_start_time = time.time()
            circle_mode_start_time = 0  

            target_x, target_y = calculate_figure_eight_setpoint(figure_eight_mode_start_time, BOARD_DIMENSIONS)
            pid_x.setpoint = target_x
            pid_y.setpoint = target_y
            
            if balled_frame is not None:
                 cv2.circle(balled_frame, (target_x, target_y), 10, (255, 0, 255), 2) 

        elif circle_mode_on:
            if circle_mode_start_time == 0:
                circle_mode_start_time = time.time()
            figure_eight_mode_start_time = 0 
            
            target_x, target_y = calculate_circular_setpoint(circle_mode_start_time, BOARD_DIMENSIONS)
            pid_x.setpoint = target_x
            pid_y.setpoint = target_y
            
            if balled_frame is not None:
                 cv2.circle(balled_frame, (target_x, target_y), 10, (0, 0, 255), 2) 

        else:
           
            circle_mode_start_time = 0
            figure_eight_mode_start_time = 0
            pid_x.setpoint = (BOARD_DIMENSIONS[0] / 2) + OFFSET_PX
            pid_y.setpoint = (BOARD_DIMENSIONS[1] / 2) + OFFSET_PX

        # Unified PID control logic
        control_x, control_y = 0, 0
        distance_from_center = None

        if final_ball_pos:
            control_x, control_y = ball_position_pid(final_ball_pos, BOARD_DIMENSIONS, pid_x, pid_y, max(dt, 0.001))
            # Calculate distance from the current setpoint (which could be moving)
            current_setpoint = (pid_x.setpoint, pid_y.setpoint)
            distance_from_center = np.linalg.norm(np.array(final_ball_pos) - np.array(current_setpoint))
        else:
            # If the ball is lost, reset the PID to avoid windup
            pid_x.reset()
            pid_y.reset()
        
        NEUTRAL_ANGLE = 90
        MAX_TILT_ANGLE = 30
        servo_angle_x = NEUTRAL_ANGLE + (control_x / 100.0) * MAX_TILT_ANGLE
        servo_angle_y = NEUTRAL_ANGLE + (control_y / 100.0) * MAX_TILT_ANGLE

        servo_angle_x = int(np.clip(servo_angle_x, 0, 180))
        servo_angle_y = int(np.clip(servo_angle_y, 0, 180))
        
        if balled_frame is not None:
            if final_ball_pos:
                cv2.circle(balled_frame, final_ball_pos, ball_radius or 20, (0, 255, 0), 3)
            cv2.imshow('Ball Detection', balled_frame)

        if SERIAL_ENABLED:
            try:
                command = f"<{servo_angle_x},{servo_angle_y}>\n"
                ser.write(command.encode('utf-8'))
            except Exception as e:
                print(f"Serial write error: {e}")

        
        if now - last_print_time > 0.1:
            ball_pos_str = f"({final_ball_pos[0]}, {final_ball_pos[1]})" if final_ball_pos else "Not detected"
            dist_str = f"{distance_from_center:.2f}" if distance_from_center is not None else "N/A"
            print(f"Ball: {ball_pos_str}, Distance: {dist_str}, Servos: <{servo_angle_x},{servo_angle_y}>")
            last_print_time = now

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break
        elif key == ord('r'):
            pid_x.reset(); pid_y.reset()
            corner_history.clear()
            last_valid_corners = None
            print("\nPID and history reset")

    cap.release()
    cv2.destroyAllWindows()
    if SERIAL_ENABLED and 'ser' in globals() and ser.is_open:
        ser.write(b"<90,90>\n")
        ser.close()
        print("\nSerial connection closed")

if __name__ == "__main__":
    main()
