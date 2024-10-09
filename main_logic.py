import time
import opencv as cv2
import numpy as np
import requests
import os
import warnings
import math

# ESP32 URL
URL = "http://172.20.10.12"
CAR_URL = "http://172.20.10.13"

# Initialize video capture from ESP32-CAM
cap = cv2.VideoCapture(URL + ":81/stream")

# Global variables
middle_lane = None
left_lane = None
right_lane = None

def set_resolution(url: str, index: int = 1, verbose: bool = False):
    try:
        if verbose:
            resolutions = ("10: UXGA(1600x1200)\n9: SXGA(1280x1024)\n8: XGA(1024x768)\n"
                           "7: SVGA(800x600)\n6: VGA(640x480)\n5: CIF(400x296)\n4: QVGA(320x240)\n"
                           "3: HQVGA(240x176)\n0: QQVGA(160x120)")
            print(f"Available resolutions\n{resolutions}")

        if index in [10, 9, 8, 7, 6, 5, 4, 3, 0]:
            requests.get(url + f"/control?var=framesize&val={index}")
        else:
            print("Wrong index")
    except Exception as e:
        print(f"SET_RESOLUTION: something went wrong - {e}")

def set_quality(url: str, value: int = 4, verbose: bool = False):
    try:
        if 10 <= value <= 63:
            requests.get(url + f"/control?var=quality&val={value}")
    except Exception as e:
        print(f"SET_QUALITY: something went wrong - {e}")

def set_awb(url: str, awb: int = 1):
    try:
        awb = not awb
        requests.get(url + f"/control?var=awb&val={1 if awb else 0}")
    except Exception as e:
        print(f"SET_AWB: something went wrong - {e}")
    return awb

def save_image(frame, save_dir="images", img_format="jpg"):
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    filename = os.path.join(save_dir, f"frame_{cv2.getTickCount()}.{img_format}")
    cv2.imwrite(filename, frame)
    print(f"Image saved: {filename}")

def extrapolate_lines(lines, img_height):
    left_lines = []
    right_lines = []

    min_slope_threshold = 0.1  # Define a minimum slope threshold

    for line in lines:
        x1, y1, x2, y2 = line[0]
        try:
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', np.RankWarning)
                slope, intercept = np.polyfit((x1, x2), (y1, y2), 1)

            print(f"Line detected with slope: {slope}")

            if abs(slope) < min_slope_threshold:
                print(f"Ignoring line with slope: {slope} (too flat)")
                continue

            if slope < 0:  # Left lane (negative slope)
                left_lines.append((slope, intercept))
                print(f"Added to left lines: slope={slope}, intercept={intercept}")
            elif slope > 0:  # Right lane (positive slope)
                right_lines.append((slope, intercept))
                print(f"Added to right lines: slope={slope}, intercept={intercept}")

        except np.linalg.LinAlgError as e:
            print(f"Error fitting line: {e}")
            continue

    if left_lines:
        left_slope, left_intercept = np.mean(left_lines, axis=0)
        print(f"Left lane slope: {left_slope}, intercept: {left_intercept}")
    else:
        left_slope, left_intercept = None, None
        print("No left lanes detected.")

    if right_lines:
        right_slope, right_intercept = np.mean(right_lines, axis=0)
        print(f"Right lane slope: {right_slope}, intercept: {right_intercept}")
    else:
        right_slope, right_intercept = None, None
        print("No right lanes detected.")

    if left_slope is not None:
        left_x1 = int((img_height - left_intercept) / left_slope)
        left_x2 = int((int(img_height * 0.6) - left_intercept) / left_slope)
        left_lane = ((left_x1, img_height), (left_x2, int(img_height * 0.6)))
    else:
        left_lane = None

    if right_slope is not None:
        right_x1 = int((img_height - right_intercept) / right_slope)
        right_x2 = int((int(img_height * 0.6) - right_intercept) / right_slope)
        right_lane = ((right_x1, img_height), (right_x2, int(img_height * 0.6)))
    else:
        right_lane = None

    return left_lane, right_lane

def calculate_middle_lane(left_lane, right_lane):
    """
    Calculate the middle lane between the left and right lanes.
    Make it a vertical line at the average x-coordinate.
    """
    if left_lane is not None and right_lane is not None:
        # Average the x-coordinates of the bottom points of the left and right lanes
        middle_x = (left_lane[0][0] + right_lane[0][0]) // 2

        # Create a vertical line for the middle lane from the bottom to the top of the image
        middle_lane = ((middle_x, left_lane[0][1]), (middle_x, 0))  # Extend it vertically
        return middle_lane
    return None


def calculate_angle(left_lane, right_lane, img_height):
    """
    Calculate the angle of the middle lane relative to the vertical axis by fitting a curve.
    """
    if left_lane is not None and right_lane is not None:
        # Extract points from left and right lanes
        left_x1, left_y1 = left_lane[0]
        left_x2, left_y2 = left_lane[1]
        right_x1, right_y1 = right_lane[0]
        right_x2, right_y2 = right_lane[1]

        # Find the middle points along the lane
        middle_x1 = (left_x1 + right_x1) // 2
        middle_x2 = (left_x2 + right_x2) // 2

        middle_lane_points = np.array([[middle_x1, img_height], [middle_x2, int(img_height * 0.6)]])

        # Fit a 2nd degree polynomial (quadratic) to the middle points
        x_vals = [middle_x1, middle_x2]
        y_vals = [img_height, int(img_height * 0.6)]

        # Fit a quadratic curve (degree 2)
        poly_fit = np.polyfit(y_vals, x_vals, 2)  # Fit to y (height) and x (width) coordinates

        # Use the derivative of the polynomial to get the slope at the bottom of the image (car's position)
        poly_derivative = np.polyder(poly_fit)

        # Calculate the slope at the bottom (y = img_height)
        slope_at_bottom = np.polyval(poly_derivative, img_height)

        # Convert slope to angle in degrees
        angle_radians = math.atan(slope_at_bottom)
        angle_degrees = math.degrees(angle_radians)

        # Adjust to get the angle relative to the vertical axis (90 degrees)
        angle_relative_to_vertical = 90 - angle_degrees

        return angle_relative_to_vertical
    return None


# Modify the draw_lane_lines function to correctly draw the middle lane
def draw_lane_lines(img, left_lane, right_lane, middle_lane, img_center_x):
    img_with_lines = np.copy(img)

    if left_lane is not None:
        cv2.line(img_with_lines, left_lane[0], left_lane[1], (0, 255, 0), 5)

    if right_lane is not None:
        cv2.line(img_with_lines, right_lane[0], right_lane[1], (0, 255, 0), 5)

    if middle_lane is not None:
        cv2.line(img_with_lines, middle_lane[0], middle_lane[1], (0, 0, 255), 5)

        # Calculate lane angle and control the car
        lane_angle = calculate_angle(left_lane, right_lane, img.shape[0])

        # Display the angle, direction, and turn force on the image
        angle_text = f"Angle: {int(lane_angle)} degrees"


        # Set positions for text
        cv2.putText(img_with_lines, angle_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    return img_with_lines

def draw_roi(img, polygon):
    roi_img = img.copy()
    cv2.polylines(roi_img, [polygon], isClosed=True, color=(0, 255, 255), thickness=2)
    return roi_img


def control_car(left_lane, right_lane, middle_lane_angle):
    """
    Control the car based on the detected lanes.
    - If only the left lane is detected, turn right.
    - If only the right lane is detected, turn left.
    - If both lanes are detected, follow the middle lane angle to go forward-left or forward-right.
    """
    target_angle = 90  # Ideal angle, representing a straight path.
    tolerance = 5      # Tolerance range for going straight

    # Handle cases where only one lane is detected
    if left_lane is not None and right_lane is None:
        # Only left lane detected, so turn right
        direction = "Right"
        turn_force = 5  # Set a fixed turn force
        print("Only left lane detected, turning right")
    elif right_lane is not None and left_lane is None:
        # Only right lane detected, so turn left
        direction = "Left"
        turn_force = 5  # Set a fixed turn force
        print("Only right lane detected, turning left")
    else:
        # Both lanes detected, follow middle lane angle
        angle_difference = target_angle - middle_lane_angle

        if abs(angle_difference) <= tolerance:
            # Angle is within tolerance, go straight
            direction = "Straight"
            turn_force = 0
            print("Both lanes detected, moving straight")
        elif angle_difference > tolerance:
            # Middle lane angle is too far right, move left slightly
            direction = "ForwardLeft"
            turn_force = angle_difference  # Adjust force based on angle
            print(f"Both lanes detected, moving forward-left, angle difference: {angle_difference}")
        else:
            # Middle lane angle is too far left, move right slightly
            direction = "ForwardRight"
            turn_force = abs(angle_difference)  # Adjust force based on angle
            print(f"Both lanes detected, moving forward-right, angle difference: {-angle_difference}")

    # Send control commands to the car based on the determined direction
    if direction == "Straight":
        requests.get(CAR_URL + "/control?cmd=STRAIGHT")
    elif direction == "Left":
        requests.get(CAR_URL + f"/control?cmd=RIGHT&force={turn_force}")
    elif direction == "Right":
        requests.get(CAR_URL + f"/control?cmd=LEFT&force={turn_force}")
    elif direction == "ForwardLeft":
        requests.get(CAR_URL + f"/control?cmd=FORWARDLEFT&force={turn_force}")
    elif direction == "ForwardRight":
        requests.get(CAR_URL + f"/control?cmd=FORWARDRIGHT&force={turn_force}")

    return direction, turn_force


# Modify the on_change function to pass lanes to control_car
def on_change(val):
    global middle_lane, left_lane, right_lane
    try:

        low_threshold = cv2.getTrackbarPos('Low Threshold', 'Lane Detection')
        high_threshold = cv2.getTrackbarPos('High Threshold', 'Lane Detection')
        min_line_length = cv2.getTrackbarPos('Min Line Length', 'Lane Detection')
        max_line_gap = cv2.getTrackbarPos('Max Line Gap', 'Lane Detection')

        blur = cv2.GaussianBlur(img_gray, (5, 5), 0)
        edges = cv2.Canny(blur, low_threshold, high_threshold)
        masked_edges = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, threshold=15, minLineLength=min_line_length,
                                maxLineGap=max_line_gap)

        if lines is not None:
            left_lane, right_lane = extrapolate_lines(lines, img_gray.shape[0])

            # Calculate the middle lane as a vertical line based on left and right lanes
            middle_lane = calculate_middle_lane(left_lane, right_lane)

            # Draw lanes on the image
            img_with_lines = draw_lane_lines(frame, left_lane, right_lane, middle_lane, img_gray.shape[1] // 2)
            img_with_roi = draw_roi(img_with_lines, polygon)
            cv2.imshow('Lane Detection', img_with_roi)

    except Exception as e:
        print(f"ON_CHANGE: something went wrong - {e}")


def apply_sharpening_filter(img):
    # Sharpening kernel (can be tuned for better results)
    kernel = np.array([[0, -1, 0],
                       [-1, 5, -1],
                       [0, -1, 0]])

    # Apply the sharpening filter
    sharpened = cv2.filter2D(img, -1, kernel)
    return sharpened


if __name__ == '__main__':
    set_resolution(URL, index=8)

    # Create window and trackbars
    cv2.namedWindow('Lane Detection')
    cv2.createTrackbar('Low Threshold', 'Lane Detection', 129, 255, on_change)
    cv2.createTrackbar('High Threshold', 'Lane Detection', 220, 255, on_change)
    cv2.createTrackbar('Min Line Length', 'Lane Detection', 86, 300, on_change)
    cv2.createTrackbar('Max Line Gap', 'Lane Detection', 50, 300, on_change)

    # Initialize variables
    img_gray = None
    middle_lane = None
    left_lane = None
    right_lane = None
    last_request_time = time.time()

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Failed to grab frame")
            break

        # Apply sharpening filter to the frame
        sharpened_frame = apply_sharpening_filter(frame)

        # Convert to grayscale and equalize histogram
        img_gray = cv2.cvtColor(sharpened_frame, cv2.COLOR_BGR2GRAY)
        img_gray = cv2.equalizeHist(img_gray)

        # Create mask for region of interest (ROI)
        height, width = img_gray.shape
        polygon = np.array([
            [(int(width * 0.001), height),
             (int(width), height),
             (int(width), int(height * 0.5)),
             (int(width * 0.001), int(height * 0.5))]])
        mask = np.zeros_like(img_gray)
        cv2.fillPoly(mask, [polygon], 255)

        # Manually call on_change to update based on current trackbar positions
        on_change(0)

        # Draw ROI on the frame
        frame_with_roi = draw_roi(sharpened_frame, polygon)
        cv2.imshow("frame", frame_with_roi)

        # Control the car based on lane position
        if left_lane or right_lane:
            current_time = time.time()
            if current_time - last_request_time >= 3:
                print('CONTROL')
                lane_angle = calculate_angle(left_lane, right_lane, img_gray.shape[0])
                control_car(left_lane, right_lane, lane_angle)
                last_request_time = current_time  # Reset timer

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    cap.release()
