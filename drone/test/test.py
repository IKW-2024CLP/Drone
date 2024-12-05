from drone.drone import *
# Test one by one!
if __name__ == '__main__':
    vehicle = drone("172.30.208.1:14550")
    # Velocity 기반 비행 테스트
    print("Test here")

    # TODO:
    #   Test the move based velocity code.
    #   If you can, try move with image sensors.(e.g. Camera with aruco)
    if vehicle.altitude < 2:
        vehicle.takeoff()
    print("x = 1")
    print(f"current yaw : {vehicle.yaw}")
    vehicle.move_velo(0,0,0,0.78,0)