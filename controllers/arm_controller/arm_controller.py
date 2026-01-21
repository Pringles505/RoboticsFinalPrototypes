import os
import sys
import json
import math
import time
from pathlib import Path
from typing import Optional
from dataclasses import dataclass
from controller import Robot, Motor, PositionSensor

# Add parent directory for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from controllers.arm_controller.ik_solver import IKSolver, IKResult


@dataclass
class RobotConfig:
    # Link lengths (meters)
    link1: float = 0.4
    link2: float = 0.3
    link3: float = 0.2

    # Shoulder pivot height above world z=0
    # Base at z=0, base height=0.1, shoulder housing=0.025
    shoulder_height: float = 0.125

    # Control parameters
    # 3cm - tighter for better accuracy
    position_tolerance: float = 0.03  
    max_velocity: float = 1.0  # rad/s

    @property
    def total_reach(self) -> float:
        return self.link1 + self.link2 + self.link3


CONFIG = RobotConfig()
# Maximum recursive verification attempts
MAX_ATTEMPTS = 5  

# Paths
CONTROLLER_DIR = Path(__file__).parent
PROJECT_DIR = CONTROLLER_DIR.parent.parent
FRAMES_DIR = PROJECT_DIR / "frames"
COMMAND_FILE = CONTROLLER_DIR / "command.json"
STATUS_FILE = CONTROLLER_DIR / "status.json"

AI_COUNCIL_URL = os.getenv("AI_COUNCIL_URL", "http://localhost:8000")


class ArmController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # IK solver
        self.ik = IKSolver(
            link_lengths=[CONFIG.link1, CONFIG.link2, CONFIG.link3],
            base_height=CONFIG.shoulder_height
        )

        # Motors and sensors
        self.motors: dict[str, Motor] = {}
        self.sensors: dict[str, PositionSensor] = {}

        motor_names = ["shoulder_motor", "elbow_motor", "wrist_motor", "end_motor"]
        sensor_names = ["shoulder_sensor", "elbow_sensor", "wrist_sensor", "end_sensor"]

        for motor_name, sensor_name in zip(motor_names, sensor_names):
            motor = self.robot.getDevice(motor_name)
            sensor = self.robot.getDevice(sensor_name)
            if motor and sensor:
                self.motors[motor_name] = motor
                sensor.enable(self.timestep)
                self.sensors[sensor_name] = sensor
                motor.setVelocity(CONFIG.max_velocity)

        # State
        self.current_command_id: Optional[str] = None
        self.waypoints: list[dict] = []
        self.waypoint_index: int = 0
        self.is_executing: bool = False
        self.last_status_time: float = 0
        self.waypoint_start_time: float = 0
        
        # Seconds per waypoint before timeout
        self.waypoint_timeout: float = 5.0  

        # Recursive verification state
        self.current_command_text: str = ""
        self.current_attempt: int = 0
        self.pending_verification: bool = False
        self.verification_delay_start: float = 0
        # Brief wait after completion before verifying
        self.verification_delay: float = 0.2  

        print(f"[Arm] Initialized. Reach: {CONFIG.total_reach:.2f}m, Shoulder height: {CONFIG.shoulder_height:.3f}m")
        print(f"[Arm] Max attempts per command: {MAX_ATTEMPTS}")

    def get_joint_angles(self) -> dict[str, float]:
        return {
            "shoulder": self.sensors["shoulder_sensor"].getValue(),
            "elbow": self.sensors["elbow_sensor"].getValue(),
            "wrist": self.sensors["wrist_sensor"].getValue(),
            "end": self.sensors["end_sensor"].getValue()
        }

    def set_joint_angles(self, shoulder: float, elbow: float, wrist: float, end: float = 0.0):
        self.motors["shoulder_motor"].setPosition(shoulder)
        self.motors["elbow_motor"].setPosition(elbow)
        self.motors["wrist_motor"].setPosition(wrist)
        self.motors["end_motor"].setPosition(end)

    def move_to_position(self, x: float, y: float, z: float) -> IKResult:
        result = self.ik.solve(x, y, z)
        if result.success:
            # end_motor keeps end effector horizontal
            # Total angle should be 0: elbow + wrist + end = 0
            end_angle = -(result.elbow + result.wrist)
            self.set_joint_angles(result.shoulder, result.elbow, result.wrist, end_angle)
            print(f"[IK] Target ({x:.2f},{y:.2f},{z:.2f}) -> angles: "
                  f"sh={math.degrees(result.shoulder):.1f}°, "
                  f"el={math.degrees(result.elbow):.1f}°, "
                  f"wr={math.degrees(result.wrist):.1f}°, "
                  f"end={math.degrees(end_angle):.1f}°")
        return result

    def get_end_effector_position(self) -> tuple[float, float, float]:
        angles = self.get_joint_angles()
        return self.ik.forward_kinematics(
            angles["shoulder"],
            angles["elbow"],
            angles["wrist"]
        )

    def distance_to_target(self, target: tuple[float, float, float]) -> float:
        current = self.get_end_effector_position()
        return math.sqrt(
            (current[0] - target[0])**2 +
            (current[1] - target[1])**2 +
            (current[2] - target[2])**2
        )

    def has_reached_target(self, target: tuple[float, float, float]) -> bool:
        return self.distance_to_target(target) < CONFIG.position_tolerance

    def call_ai_council(self, command: str, is_verification: bool = False) -> Optional[dict]:
        import urllib.request
        import urllib.parse

        try:
            url = f"{AI_COUNCIL_URL}/api/council"

            # For verification, modify the command to ask for verification
            if is_verification:
                verify_command = f"VERIFY: {command} - Check if the action was successful. If not, provide a new plan to complete it."
            else:
                verify_command = command

            data = urllib.parse.urlencode({
                "command": verify_command,
                "recognition_json": "{}"
            }).encode()

            req = urllib.request.Request(url, data=data, method="POST")
            req.add_header("Content-Type", "application/x-www-form-urlencoded")

            with urllib.request.urlopen(req, timeout=60) as response:
                result = json.loads(response.read().decode())

            if result.get("success"):
                print(f"[AI] Vision: {result['vision']['scene_summary']}")
                print(f"[AI] Plan: {result['plan']['reasoning']}")
                return result

            return None

        except Exception as e:
            print(f"[AI] Connection failed: {e}, using mock")
            return self._mock_response(command, is_verification)

    def call_verification(self, original_command: str) -> dict:
        import urllib.request
        import urllib.parse

        try:
            url = f"{AI_COUNCIL_URL}/api/verify"
            data = urllib.parse.urlencode({
                "original_command": original_command,
                "attempt": self.current_attempt,
                "max_attempts": MAX_ATTEMPTS
            }).encode()

            req = urllib.request.Request(url, data=data, method="POST")
            req.add_header("Content-Type", "application/x-www-form-urlencoded")

            with urllib.request.urlopen(req, timeout=60) as response:
                result = json.loads(response.read().decode())

            return result

        except Exception as e:
            print(f"[Verify] API failed: {e}, using mock verification")
            return self._mock_verification(original_command)

    def _mock_verification(self, command: str) -> dict:
        # For mock, always say success after attempt 2
        if self.current_attempt >= 2:
            return {
                "success": True,
                "action_completed": True,
                "reasoning": "Mock: Action appears complete after multiple attempts"
            }

        # 50% chance of success on first attempt for testing to just see if the verification works
        import random
        if random.random() > 0.5:
            return {
                "success": True,
                "action_completed": True,
                "reasoning": "Mock: Action appears successful"
            }
        else:
            # Need to retry
            cmd = command.lower()
            cubes = {
                "blue": (0.5, 0.1),
                "red": (0.7, -0.15),
                "green": (0.55, -0.05),
                "yellow": (0.45, 0.2)
            }

            target_x, target_y = 0.5, 0.0
            for color, (cx, cy) in cubes.items():
                if color in cmd:
                    target_x, target_y = cx, cy
                    break

            return {
                "success": True,
                "action_completed": False,
                "reasoning": "Mock: Action not complete, cube not pushed far enough",
                "plan": {
                    "intent": command,
                    "action": "push",
                    "waypoints": [
                        {"x": target_x, "y": target_y, "z": 0.35, "description": "re-approach"},
                        {"x": target_x + 0.15, "y": target_y, "z": 0.23, "description": "push further"},
                        {"x": target_x + 0.15, "y": target_y, "z": 0.40, "description": "retract"},
                    ],
                    "reasoning": "Retry: pushing cube further"
                }
            }

    def _mock_response(self, command: str, is_verification: bool = False) -> dict:
        cmd = command.lower()

        # Cube positions on table
        cubes = {
            "blue": (0.5, 0.1),
            "red": (0.7, -0.15),
            "green": (0.55, -0.05),
            "yellow": (0.45, 0.2)
        }

        target_color = None
        target_x, target_y = 0.5, 0.0

        for color, (cx, cy) in cubes.items():
            if color in cmd:
                target_color = color
                target_x, target_y = cx, cy
                break

        waypoints = []
        if "push" in cmd and target_color:
            # Higher Z values to avoid table collision (table at Z=0.16)
            waypoints = [
                {"x": target_x - 0.10, "y": target_y, "z": 0.35, "description": "approach high"},
                {"x": target_x - 0.03, "y": target_y, "z": 0.23, "description": "lower to cube top"},
                {"x": target_x + 0.12, "y": target_y, "z": 0.23, "description": "push through"},
                {"x": target_x + 0.12, "y": target_y, "z": 0.40, "description": "retract up"},
            ]
            print(f"[Mock] Push plan for {target_color} cube at ({target_x}, {target_y})")
        elif "home" in cmd:
            waypoints = [{"x": 0.5, "y": 0.0, "z": 0.40, "description": "home"}]
        else:
            waypoints = [{"x": target_x, "y": target_y, "z": 0.35, "description": "move to target"}]

        return {
            "success": True,
            "vision": {"scene_summary": f"Mock: {target_color or 'objects'} detected"},
            "plan": {
                "intent": command,
                "action": "push" if "push" in cmd else "move",
                "waypoints": waypoints,
                "reasoning": f"Mock plan for {target_color or 'target'}"
            }
        }

    def check_for_command(self) -> Optional[dict]:
        if not COMMAND_FILE.exists():
            return None

        try:
            with open(COMMAND_FILE, "r") as f:
                data = json.load(f)

            cmd_id = data.get("id")
            if cmd_id and cmd_id != self.current_command_id:
                self.current_command_id = cmd_id
                return data
        except:
            pass
        return None

    def start_command(self, command_text: str):
        self.current_command_text = command_text
        self.current_attempt = 1

        print(f"\n{'='*60}")
        print(f"[Command] '{command_text}' - Attempt {self.current_attempt}/{MAX_ATTEMPTS}")
        print(f"{'='*60}")

        response = self.call_ai_council(command_text)
        if response and response.get("success"):
            self.execute_plan(response["plan"])

    def execute_plan(self, plan: dict):
        waypoints = plan.get("waypoints", [])
        if not waypoints:
            print("[Exec] No waypoints!")
            self.on_execution_complete()
            return

        self.waypoints = waypoints
        self.waypoint_index = 0
        self.is_executing = True
        self.waypoint_start_time = self.robot.getTime()

        print(f"[Exec] Starting plan: {len(waypoints)} waypoints")
        self._go_to_waypoint()

    def _go_to_waypoint(self):
        if self.waypoint_index >= len(self.waypoints):
            self.on_execution_complete()
            return

        wp = self.waypoints[self.waypoint_index]
        x, y, z = wp["x"], wp["y"], wp["z"]
        desc = wp.get("description", f"wp{self.waypoint_index}")

        print(f"[Exec] -> {desc}: ({x:.3f}, {y:.3f}, {z:.3f})")

        result = self.move_to_position(x, y, z)
        self.waypoint_start_time = self.robot.getTime()

        if not result.success:
            print(f"[Exec] IK issue: {result.message}")

    def on_execution_complete(self):
        print(f"[Exec] Plan execution complete! Verifying...")
        self.is_executing = False
        self.waypoints = []

        # Start verification 
        self.pending_verification = True
        self.verification_delay_start = self.robot.getTime()

    def check_verification(self):
        if not self.pending_verification:
            return

        elapsed = self.robot.getTime() - self.verification_delay_start
        if elapsed < self.verification_delay:
            return

        self.pending_verification = False
        self.run_verification()

    def run_verification(self):
        print(f"\n[Verify] Checking if action was successful (attempt {self.current_attempt}/{MAX_ATTEMPTS})...")

        result = self.call_verification(self.current_command_text)

        if result.get("action_completed", False):
            print(f"[Verify] SUCCESS! Action completed successfully.")
            print(f"[Verify] Reason: {result.get('reasoning', 'N/A')}")
            self.current_command_text = ""
            self.current_attempt = 0
            return

        # Action not complete - check if we can retry
        if self.current_attempt >= MAX_ATTEMPTS:
            print(f"[Verify] FAILED after {MAX_ATTEMPTS} attempts. Giving up.")
            print(f"[Verify] Reason: {result.get('reasoning', 'N/A')}")
            self.current_command_text = ""
            self.current_attempt = 0
            return

        # Retry
        self.current_attempt += 1
        print(f"\n{'='*60}")
        print(f"[Retry] Action not complete. Attempting retry {self.current_attempt}/{MAX_ATTEMPTS}")
        print(f"[Retry] Reason: {result.get('reasoning', 'N/A')}")
        print(f"{'='*60}")

        # Check if verification returned a new plan
        if result.get("plan"):
            print("[Retry] Using plan from verification response")
            self.execute_plan(result["plan"])
        else:
            # Get a new plan from the council
            print("[Retry] Requesting new plan from AI Council")
            response = self.call_ai_council(self.current_command_text, is_verification=True)
            if response and response.get("success"):
                self.execute_plan(response["plan"])
            else:
                print("[Retry] Failed to get new plan")
                self.current_command_text = ""
                self.current_attempt = 0

    def update_execution(self):
        if not self.is_executing or not self.waypoints:
            return

        wp = self.waypoints[self.waypoint_index]
        target = (wp["x"], wp["y"], wp["z"])

        dist = self.distance_to_target(target)
        elapsed = self.robot.getTime() - self.waypoint_start_time

        # Check if reached or timed out
        if dist < CONFIG.position_tolerance:
            print(f"[Exec] Reached waypoint {self.waypoint_index} (dist={dist:.3f}m)")
            self.waypoint_index += 1
            self._go_to_waypoint()
        elif elapsed > self.waypoint_timeout:
            print(f"[Exec] Timeout at waypoint {self.waypoint_index} (dist={dist:.3f}m), advancing...")
            self.waypoint_index += 1
            self._go_to_waypoint()

    def write_status(self):
        now = self.robot.getTime()
        if now - self.last_status_time < 0.5:
            return
        self.last_status_time = now

        angles = self.get_joint_angles()
        ee = self.get_end_effector_position()

        target = None
        if self.is_executing and self.waypoint_index < len(self.waypoints):
            wp = self.waypoints[self.waypoint_index]
            target = {"x": wp["x"], "y": wp["y"], "z": wp["z"]}

        status = {
            "time": now,
            "executing": self.is_executing,
            "waypoint": f"{self.waypoint_index}/{len(self.waypoints)}",
            "ee": {"x": round(ee[0], 3), "y": round(ee[1], 3), "z": round(ee[2], 3)},
            "target": target,
            "angles_deg": {
                "shoulder": round(math.degrees(angles["shoulder"]), 1),
                "elbow": round(math.degrees(angles["elbow"]), 1),
                "wrist": round(math.degrees(angles["wrist"]), 1)
            },
            "current_command": self.current_command_text,
            "attempt": f"{self.current_attempt}/{MAX_ATTEMPTS}" if self.current_command_text else "0/0",
            "pending_verification": self.pending_verification
        }

        try:
            with open(STATUS_FILE, "w") as f:
                json.dump(status, f, indent=2)
        except:
            pass

    def run(self):
        print("[Arm] Starting...")
        print(f"[Arm] At motors (0,0,0,0), EE is at {self.ik.forward_kinematics(0,0,0)}")

        # Initial position - safely above table (table at Z=0.16)
        print("[Arm] Moving to initial position...")
        self.move_to_position(0.5, 0.0, 0.40)

        while self.robot.step(self.timestep) != -1:
            # Check for new commands (only if not currently processing one)
            if not self.is_executing and not self.pending_verification and not self.current_command_text:
                cmd = self.check_for_command()
                if cmd:
                    text = cmd.get("text", "")
                    self.start_command(text)

            # Update execution
            self.update_execution()

            # Check for pending verification
            self.check_verification()

            # Write status
            self.write_status()


if __name__ == "__main__":
    ArmController().run()
