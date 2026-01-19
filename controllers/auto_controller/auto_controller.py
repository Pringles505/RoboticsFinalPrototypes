from controller import Robot, Keyboard, Motor
import base64
import sys
import os
import time
import cv2
import numpy as np
from openai import OpenAI

# Configuration
OPENAI_API_KEY = "mete aqui el openai key"


def set_motor_position_safe(motor: Motor, target: float):
    mn = motor.getMinPosition()
    mx = motor.getMaxPosition()

    # Clamp to min if defined
    if mn != float("-inf") and target < mn:
        target = mn
    # Clamp to max if defined
    if mx != float("inf") and target > mx:
        target = mx

    motor.setPosition(target)


class WebcamRoboticPicker:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.client = OpenAI(api_key=OPENAI_API_KEY)

        # Webcam
        print("Initializing webcam...")
        self.webcam = cv2.VideoCapture(0)
        if not self.webcam.isOpened():
            print("❌ Could not open webcam")
            sys.exit(1)

        self.webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        print("✓ Webcam initialized")

        # Keyboard
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.timestep)

        # Discover motors
        print("\n" + "=" * 60)
        print("AUTO-DISCOVERING MOTORS...")
        print("=" * 60)

        all_motors = []
        device_count = self.robot.getNumberOfDevices()

        for i in range(device_count):
            dev = self.robot.getDeviceByIndex(i)
            if isinstance(dev, Motor):
                all_motors.append(dev)

        if len(all_motors) < 6:
            print("❌ ERROR: Less than 6 motors found")
            sys.exit(1)

        self.arm_motors = all_motors[:6]
        self.gripper_motors = all_motors[6:]

        print(f"✓ ARM motors: {len(self.arm_motors)}")
        print(f"✓ GRIPPER motors: {len(self.gripper_motors)}")

        # Velocities
        for m in self.arm_motors:
            m.setVelocity(1.0)
        for g in self.gripper_motors:
            g.setVelocity(0.5)

        # Sensors
        self.sensors = []
        for m in self.arm_motors:
            ps = m.getPositionSensor()
            if ps:
                ps.enable(self.timestep)
                self.sensors.append(ps)

        print(f"✓ Position sensors: {len(self.sensors)}")

        # Print gripper limits once
        print("\nGRIPPER MOTOR LIMITS:")
        for g in self.gripper_motors:
            print(
                f"{g.getName():30} min={g.getMinPosition():.4f} max={g.getMaxPosition():.4f}"
            )

        # Gripper positions fucking headache finding these values, use the position finder script
        self.GRIPPER_OPEN = -0.1
        self.GRIPPER_CLOSE = 0.6

        # Arm positions more manual bullshit cube, sphere and cylinder they are all cubes but fuck replacing them names
        self.positions = {
            "home": [0, -1.57, 1.57, -1.57, -1.57, 0],
            # Box positions where the objects are 
            "red_cube_end": [-3.898, -1.109, 1.37, -1.87, -1.57, -0.0],
            "green_sphere_end": [-3.398, -1.109, 1.37, -1.87, -1.57, -0.0],
            "blue_cylinder_end": [-2.899, -1.109, 1.37, -1.87, -1.57, 0.0],
            # Above mid position so it doesnt hit anything like an animation frame
            "blue_cylinder_above": [-0.6, -1.37, 1.37, -1.57, -1.57, 0.0],
            "green_sphere_above": [-0.1, -1.47, 1.67, -1.87, -1.57, 0.0],
            "red_cube_above": [0.5, -1.27, 1.57, -2.07, -1.57, -0.0],
            # Object positions where to drop
            "red_cube": [0.5, -1.269, 1.871, -2.27, -1.57, 0.0],
            "green_sphere": [-0.1, -1.07, 1.67, -2.07, -1.57, -0.0],
            "blue_cylinder": [-0.9, -1.27, 1.87, -2.27, -1.57, 0.1],
        }

        self.color_to_object = {
            "red": "red_cube",
            "green": "green_sphere",
            "blue": "blue_cylinder",
        }

        print("\n✓ INITIALIZATION COMPLETE")
        print("Press SPACEBAR to capture\n")

    # Robot Controls
    def wait(self, steps=150):
        for _ in range(steps):
            self.robot.step(self.timestep)

    def move_arm(self, joint_positions):
        for m, p in zip(self.arm_motors, joint_positions):
            m.setPosition(p)

    def open_gripper(self):
        for g in self.gripper_motors:
            set_motor_position_safe(g, self.GRIPPER_OPEN)
        self.wait(80)

    def close_gripper(self):
        for g in self.gripper_motors:
            set_motor_position_safe(g, self.GRIPPER_CLOSE)
        self.wait(80)

    # Vision and AI prompt
    def capture_image(self):
        ret, frame = self.webcam.read()
        if not ret:
            return None, None
        _, buf = cv2.imencode(".jpg", frame)
        return base64.b64encode(buf).decode(), frame

    def detect_color(self, image_b64):
        prompt = "What is the MAIN COLOR? Reply with one word: red, green, blue, none"
        response = self.client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{image_b64}",
                                "detail": "low",
                            },
                        },
                    ],
                }
            ],
            max_tokens=5,
            temperature=0,
        )
        text = response.choices[0].message.content.lower()
        for c in self.color_to_object:
            if c in text:
                return c
        return None

    # Pick and place
    def pick_and_place(self, obj):
        print(f"\nPicking {obj}")
        
        box = obj.replace("cube", "box").replace("sphere", "box").replace(
            "cylinder", "box"
        )

        self.move_arm(self.positions["home"])
        self.wait(30)

        self.move_arm(self.positions[f"{obj}_above"])
        self.wait(30)

        self.open_gripper()

        self.move_arm(self.positions[obj]) 
        self.wait(30)

        self.close_gripper()

        self.move_arm(self.positions[f"{obj}_end"])
        self.wait(150)

        self.open_gripper()
        self.move_arm(self.positions["home"])
        self.wait(150)

        print("✓ Done")

    # Main running loop
    def run(self):
        self.move_arm(self.positions["home"])
        self.wait(100)

        while self.robot.step(self.timestep) != -1:
            ret, frame = self.webcam.read()
            if ret:
                cv2.imshow("Webcam", frame)
                cv2.waitKey(1)
            key = self.keyboard.getKey()
            if key == 32: 
                img_b64, _ = self.capture_image()
                color = self.detect_color(img_b64)
                if color:
                    self.pick_and_place(self.color_to_object[color])

            if cv2.waitKey(1) & 0xFF == 27:
                break

        self.webcam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    picker = WebcamRoboticPicker()
    picker.run()
