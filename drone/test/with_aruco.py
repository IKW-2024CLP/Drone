from drone.drone import *
import cv2
from cv2 import aruco

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters_create()


def draw_text(img, text,
              font=cv2.FONT_HERSHEY_PLAIN,
              pos=(0, 0),
              font_scale=3,
              font_thickness=2,
              text_color=(0, 255, 0),
              text_color_bg=(0, 0, 0)
              ):
    x, y = pos
    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_w, text_h = text_size
    cv2.rectangle(img, pos, (x + text_w, y + text_h), text_color_bg, -1)
    cv2.putText(img, text, (x, y + text_h + font_scale - 1), font, font_scale, text_color, font_thickness)

    return text_size


def cam(cap, width, height) -> Tuple[int, int]:
    line_color = (0, 255, 0)
    line_thickness = 2
    x = y = 0

    ret, frame = cap.read()
    if not ret:
        cap.release()
        cv2.destroyAllWindows()
        exit("Camera error")

    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = aruco.detectMarkers(gray_img, aruco_dict, parameters=parameters)
    # Show center
    center_x, center_y = width // 2, height // 2
    cv2.rectangle(frame, (center_x - 50, center_y + 50), (center_x + 50, center_y - 50), (255,0,255), line_thickness)
    cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), line_color, line_thickness)
    cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), line_color, line_thickness)

    if ids is not None:
        for i, corner in enumerate(corners):
            pixel_coords = corner[0]

            # Aruco position
            aruco_x = int(pixel_coords[:, 0].mean())
            aruco_y = int(pixel_coords[:, 1].mean())

            x = aruco_x - center_x
            y = center_y - aruco_y

            cv2.line(frame, (aruco_x - 20, aruco_y), (aruco_x + 20, aruco_y), (255,0,255), line_thickness)
            cv2.line(frame, (aruco_x, aruco_y - 20), (aruco_x, aruco_y + 20), (255,0,255), line_thickness)
            cv2.line(frame, (center_x, center_y), (aruco_x, aruco_y), line_color, line_thickness)
            draw_text(frame, f"  dist : (x={x}, y={y})", pos=(aruco_x - 30, aruco_y), font_scale=1)

    # 화면에 출력
    cv2.imshow('ArUco Marker Detection', frame)
    return x, y


if __name__ == '__main__':
    vehicle = None
    try:
        vehicle = drone()
    except Exception as e:
        print("Connection error\n", e)
    finally:
        cap = cv2.VideoCapture(0)
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

        while True:
            x, y = cam(cap, int(width), int(height))

            # 50보다 멀리 있을 때
            #   2차함수 방적식을 따르며 접근해보기.
            if vehicle is not None:
                if 50 < abs(x) < 500:
                    vehicle.move_velo(x / 7.2, y / 7.2, 0)
                elif 50 < abs(y) < 500:
                    vehicle.move_velo(x / 7.2, y / 7.2, 0)
            else:
                if 50 < abs(y) < 500:
                    print(f"{x / 7.2:.1f}, {y / 7.2:.1f}")
                elif 50 < abs(y) < 500:
                    print(f"{x / 7.2:.1f}, {y / 7.2:.1f}")

            # 'q' to EXIT.
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
