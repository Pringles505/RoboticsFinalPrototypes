# Apollo Command Terminal - send natural language commands to robot arm
import os
import sys
import json
import time
from pathlib import Path

# Paths
PROJECT_DIR = Path(__file__).parent
COMMAND_FILE = PROJECT_DIR / "controllers" / "arm_controller" / "command.json"
STATUS_FILE = PROJECT_DIR / "controllers" / "arm_controller" / "status.json"


def send_command(text: str) -> str:
    command_id = str(int(time.time() * 1000))

    command_data = {
        "id": command_id,
        "text": text,
        "timestamp": time.time()
    }

    # Ensure directory exists
    COMMAND_FILE.parent.mkdir(parents=True, exist_ok=True)

    with open(COMMAND_FILE, "w") as f:
        json.dump(command_data, f, indent=2)

    return command_id


def get_status() -> dict:
    if not STATUS_FILE.exists():
        return {"error": "No status file found. Is the simulation running?"}

    try:
        with open(STATUS_FILE, "r") as f:
            return json.load(f)
    except (json.JSONDecodeError, IOError) as e:
        return {"error": f"Failed to read status: {e}"}


def format_status(status: dict) -> str:
    if "error" in status:
        return f"Error: {status['error']}"

    lines = [
        f"Time: {status.get('time', 0):.2f}s",
        f"Executing: {status.get('executing', False)}",
    ]

    if status.get("executing"):
        lines.append(f"Waypoint: {status.get('waypoint_index', 0) + 1}/{status.get('waypoint_count', 0)}")

    ee = status.get("end_effector", {})
    lines.append(f"End Effector: ({ee.get('x', 0):.3f}, {ee.get('y', 0):.3f}, {ee.get('z', 0):.3f})")

    target = status.get("target")
    if target:
        lines.append(f"Target: ({target.get('x', 0):.3f}, {target.get('y', 0):.3f}, {target.get('z', 0):.3f})")

    angles = status.get("angles_deg", {})
    lines.append(f"Angles: shoulder={angles.get('shoulder', 0):.1f}°, "
                 f"elbow={angles.get('elbow', 0):.1f}°, "
                 f"wrist={angles.get('wrist', 0):.1f}°")

    return "\n".join(lines)


def print_header():
    print("\n" + "=" * 60)
    print("  APOLLO Command Terminal")
    print("  AI Council Robot Control")
    print("=" * 60)


def main():
    print_header()

    while True:
        try:
            user_input = input("apollo> ").strip()

            if not user_input:
                continue

            lower_input = user_input.lower()

            if lower_input in ("quit", "exit", "q"):
                print("Goodbye!")
                break

            elif lower_input == "status":
                status = get_status()
                print("\n" + format_status(status) + "\n")

            else:
                # Send as natural language command
                cmd_id = send_command(user_input)
                print(f"Sent: '{user_input}' (ID: {cmd_id})")
                print("Waiting for AI Council response...")

        except KeyboardInterrupt:
            print("\n\nInterrupted. Goodbye!")
            break

        except EOFError:
            print("\nGoodbye!")
            break


if __name__ == "__main__":
    main()
