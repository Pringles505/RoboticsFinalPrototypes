import json
from pathlib import Path
from controller import Robot, Camera

# Paths
CONTROLLER_DIR = Path(__file__).parent
PROJECT_DIR = CONTROLLER_DIR.parent.parent
FRAMES_DIR = PROJECT_DIR / "frames"
COMMAND_FILE = PROJECT_DIR / "controllers" / "arm_controller" / "command.json"

# Ensure frames directory exists
FRAMES_DIR.mkdir(parents=True, exist_ok=True)


class CameraController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Initialize cameras
        self.cameras: dict[str, Camera] = {}
        camera_names = ["top_camera", "front_camera", "side_camera"]

        for cam_name in camera_names:
            camera = self.robot.getDevice(cam_name)
            if camera:
                camera.enable(self.timestep)
                if hasattr(camera, 'recognitionEnable'):
                    camera.recognitionEnable(self.timestep)
                self.cameras[cam_name] = camera
                print(f"[CameraRig] Enabled {cam_name}")

        self.last_command_id = None
        print(f"[CameraRig] Initialized with {len(self.cameras)} cameras")
        print(f"[CameraRig] Saving frames to {FRAMES_DIR}")

    def capture_all(self):
        for cam_name, camera in self.cameras.items():
            filepath = FRAMES_DIR / f"{cam_name}.png"
            camera.saveImage(str(filepath), 100)
        print(f"[CameraRig] Captured {len(self.cameras)} frames")

    def check_for_capture_request(self) -> bool:
        if not COMMAND_FILE.exists():
            return False

        try:
            with open(COMMAND_FILE, "r") as f:
                cmd_data = json.load(f)

            cmd_id = cmd_data.get("id")
            if cmd_id and cmd_id != self.last_command_id:
                self.last_command_id = cmd_id
                return True
        except (json.JSONDecodeError, IOError):
            pass

        return False

    def run(self):
        print("[CameraRig] Starting main loop")

        # Capture initial frames
        # Wait a few steps for cameras to initialize
        for _ in range(10):
            self.robot.step(self.timestep)

        self.capture_all()

        while self.robot.step(self.timestep) != -1:
            # Check for new command and capture if needed
            if self.check_for_capture_request():
                # Small delay to ensure scene is rendered
                for _ in range(3):
                    self.robot.step(self.timestep)
                self.capture_all()


if __name__ == "__main__":
    controller = CameraController()
    controller.run()
