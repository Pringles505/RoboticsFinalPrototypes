# APOLLO AI Council Server
import os
import base64
import json
import re
from pathlib import Path
from typing import Optional
from fastapi import FastAPI, HTTPException, Form
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
import httpx

# Configuration
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")
OPENAI_BASE_URL = os.getenv("OPENAI_BASE_URL", "https://api.openai.com/v1")
VISION_MODEL = os.getenv("VISION_MODEL", "gpt-4o")
ORCHESTRATOR_MODEL = os.getenv("ORCHESTRATOR_MODEL", "gpt-4o")
FRAMES_DIR = Path(__file__).parent.parent / "frames"
TIMEOUT_SECONDS = 60

# Model Prompts
VISION_SYSTEM_PROMPT = (Path(__file__).parent.parent / 'model_prompts' / 'vision_system_prompt.txt').read_text()
ORCHESTRATOR_SYSTEM_PROMPT = (Path(__file__).parent.parent / 'model_prompts' / 'orchestrator_system_prompt.txt').read_text()
VERIFICATION_SYSTEM_PROMPT = (Path(__file__).parent.parent / 'model_prompts' / 'verification_system_prompt.txt').read_text()

# FastAPI app setup and middleware
app = FastAPI(
    title="APOLLO AI Council",
    description="Vision and Orchestrator agents for robot control",
    version="1.0.0"
)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# An object detected by the Vision Agent
class DetectedObject(BaseModel):
    id: str = Field(..., description="Unique identifier like 'blue_cube'")
    label: str = Field(..., description="Object type like 'cube'")
    color: Optional[str] = Field(None, description="Color if identifiable")
    position: Optional[dict] = Field(None, description="Estimated x, y, z position")
    confidence: float = Field(..., ge=0.0, le=1.0)
    notes: Optional[str] = None

# Result from vision
class VisionResult(BaseModel):
    scene_summary: str = Field(..., description="Brief description of the scene")
    objects: list[DetectedObject] = Field(default_factory=list)
    robot_visible: bool = Field(True, description="Whether robot arm is visible")
    table_visible: bool = Field(True, description="Whether work surface is visible")
    uncertainties: list[str] = Field(default_factory=list)

# A waypoint the roboto should move/pass to
class Waypoint(BaseModel):
    x: float
    y: float
    z: float
    description: Optional[str] = None

# Orch model will plan motion based on vision
class MotionPlan(BaseModel):
    intent: str = Field(..., description="What the user wants to accomplish")
    action: str = Field(..., description="Type: push, move, point, pick, unknown")
    target_object: Optional[str] = Field(None, description="Target object ID")
    waypoints: list[Waypoint] = Field(default_factory=list)
    approach_direction: Optional[str] = Field(None, description="Direction to approach from")
    speed: str = Field("normal", description="slow, normal, or fast")
    confidence: float = Field(..., ge=0.0, le=1.0)
    reasoning: str = Field(..., description="Explanation of the plan")
    warnings: list[str] = Field(default_factory=list)

# Cocatenated response from both agents
class CouncilResponse(BaseModel):
    vision: VisionResult
    plan: MotionPlan
    success: bool = True
    error: Optional[str] = None

async def call_vision_agent(images: list[tuple[str, bytes]]) -> VisionResult:
    if not OPENAI_API_KEY:
        # Return mock data for testing without API key do NOT want to be 100 euros in debt to openai lol
        return VisionResult(
            scene_summary="Mock vision: Table with colored cubes visible",
            objects=[
                DetectedObject(
                    id="blue_cube",
                    label="cube",
                    color="blue",
                    position={"x": 0.5, "y": 0.1, "z": 0.2},
                    confidence=0.85
                ),
                DetectedObject(
                    id="red_cube",
                    label="cube",
                    color="red",
                    position={"x": 0.7, "y": -0.15, "z": 0.2},
                    confidence=0.85
                ),
                DetectedObject(
                    id="green_cube",
                    label="cube",
                    color="green",
                    position={"x": 0.55, "y": -0.05, "z": 0.2},
                    confidence=0.85
                ),
                DetectedObject(
                    id="yellow_cube",
                    label="cube",
                    color="yellow",
                    position={"x": 0.45, "y": 0.2, "z": 0.2},
                    confidence=0.85
                )
            ],
            robot_visible=True,
            table_visible=True,
            uncertainties=["Using mock data - no API key configured"]
        )

    # Build message content with images
    content = [{"type": "text", "text": """Analyze these camera views and identify all objects.
        Use the camera positions from my system prompt to triangulate accurate 3D coordinates.
        Cross-reference positions across all views for best accuracy."""}]

    camera_descriptions = {
        "top_camera": "TOP CAMERA - Bird's eye view from (0.55, 0, 0.7), looking down. Use for X,Y estimation.",
        "front_camera": "FRONT CAMERA - From (1.13, 0, 0.26), looking back at table. Use for Y,Z estimation.",
        "side_camera": "SIDE CAMERA - From (0.55, -0.7, 0.3), looking sideways at table. Use for X,Z estimation."
    }

    for cam_name, img_bytes in images:
        b64_data = base64.b64encode(img_bytes).decode("utf-8")
        cam_desc = camera_descriptions.get(cam_name, f"[{cam_name}]")
        content.append({"type": "text", "text": f"\n{cam_desc}"})
        content.append({
            "type": "image_url",
            "image_url": {
                "url": f"data:image/png;base64,{b64_data}",
                "detail": "high"
            }
        })

    async with httpx.AsyncClient(timeout=TIMEOUT_SECONDS) as client:
        response = await client.post(
            f"{OPENAI_BASE_URL}/chat/completions",
            headers={
                "Authorization": f"Bearer {OPENAI_API_KEY}",
                "Content-Type": "application/json"
            },
            json={
                "model": VISION_MODEL,
                "messages": [
                    {"role": "system", "content": VISION_SYSTEM_PROMPT},
                    {"role": "user", "content": content}
                ],
                "max_tokens": 1500,
                "response_format": {"type": "json_object"}
            }
        )

        if response.status_code != 200:
            raise HTTPException(500, f"Vision API error: {response.text}")

        result = response.json()
        raw_text = result["choices"][0]["message"]["content"]

        # Parse JSON response
        try:
            data = json.loads(raw_text)
            return VisionResult(**data)
        except (json.JSONDecodeError, ValueError) as e:
            # Try to extract JSON from markdown code blocks
            json_match = re.search(r"```(?:json)?\s*([\s\S]*?)\s*```", raw_text)
            if json_match:
                data = json.loads(json_match.group(1))
                return VisionResult(**data)
            raise HTTPException(500, f"Failed to parse vision response: {e}")

async def call_orchestrator_agent(
    command: str,
    vision_result: VisionResult,
    recognition_data: Optional[dict] = None
    ) -> MotionPlan:

    if not OPENAI_API_KEY:
        # Return mock plan for testing
        target = None
        waypoints = []

        # Parse command for target object
        cmd_lower = command.lower()
        for obj in vision_result.objects:
            if obj.color and obj.color.lower() in cmd_lower:
                target = obj.id
                pos = obj.position or {"x": 0.5, "y": 0.0, "z": 0.2}

                # Softcoded push commands, this can be replaced with prompt training or local model
                if "push" in cmd_lower:
                    # Create push waypoints
                    waypoints = [
                        Waypoint(x=pos["x"]-0.1, y=pos["y"], z=0.25, description="approach"),
                        Waypoint(x=pos["x"], y=pos["y"], z=0.23, description="contact"),
                        Waypoint(x=pos["x"]+0.15, y=pos["y"], z=0.23, description="push through"),
                    ]
                else:
                    # Simple move to target
                    waypoints = [
                        Waypoint(x=pos["x"], y=pos["y"], z=0.25, description="target position")
                    ]
                break

        if not waypoints:
            waypoints = [Waypoint(x=0.5, y=0.0, z=0.3, description="default position")]

        return MotionPlan(
            intent=command,
            action="push" if "push" in cmd_lower else "move",
            target_object=target,
            waypoints=waypoints,
            approach_direction="from_behind",
            speed="normal",
            confidence=0.7,
            reasoning="Mock plan generated without API",
            warnings=["Using mock orchestrator - no API key configured"]
        )

    # Build context message debug info
    vision_json = vision_result.model_dump_json(indent=2)

    user_message = f"""User command: "{command}" Vision Agent analysis: {vision_json}"""

    if recognition_data:
        user_message += f"\nWebots recognition data: {json.dumps(recognition_data, indent=2)}"

    async with httpx.AsyncClient(timeout=TIMEOUT_SECONDS) as client:
        response = await client.post(
            f"{OPENAI_BASE_URL}/chat/completions",
            headers={
                "Authorization": f"Bearer {OPENAI_API_KEY}",
                "Content-Type": "application/json"
            },
            json={
                "model": ORCHESTRATOR_MODEL,
                "messages": [
                    {"role": "system", "content": ORCHESTRATOR_SYSTEM_PROMPT},
                    {"role": "user", "content": user_message}
                ],
                "max_tokens": 1000,
                "response_format": {"type": "json_object"}
            }
        )

        if response.status_code != 200:
            raise HTTPException(500, f"Orchestrator API error: {response.text}")

        result = response.json()
        raw_text = result["choices"][0]["message"]["content"]

        try:
            data = json.loads(raw_text)
            # Convert waypoints list of dicts to list of Waypoint objects
            if "waypoints" in data:
                data["waypoints"] = [Waypoint(**wp) for wp in data["waypoints"]]
            return MotionPlan(**data)
        except (json.JSONDecodeError, ValueError) as e:
            json_match = re.search(r"```(?:json)?\s*([\s\S]*?)\s*```", raw_text)
            if json_match:
                data = json.loads(json_match.group(1))
                if "waypoints" in data:
                    data["waypoints"] = [Waypoint(**wp) for wp in data["waypoints"]]
                return MotionPlan(**data)
            raise HTTPException(500, f"Failed to parse orchestrator response: {e}")

# API Endpoints

# Test server status, api key availability, and model info
@app.get("/health")
async def health_check():
    return {
        "status": "ok",
        "api_configured": bool(OPENAI_API_KEY),
        "vision_model": VISION_MODEL,
        "orchestrator_model": ORCHESTRATOR_MODEL
    }

# Main endpoint to run both agents
@app.post("/api/council", response_model=CouncilResponse)
async def run_council(
    command: str = Form(..., description="User's natural language command"),
    recognition_json: Optional[str] = Form(None, description="Webots recognition data as JSON")
):
    try:
        # Load camera images
        images = []
        for cam_name in ["top_camera", "front_camera", "side_camera"]:
            img_path = FRAMES_DIR / f"{cam_name}.png"
            if img_path.exists():
                images.append((cam_name, img_path.read_bytes()))

        if not images:
            # No images available, use recognition data only
            print("Warning: No camera images found, using default positions")

        # Parse recognition data if provided
        recognition_data = None
        if recognition_json:
            try:
                recognition_data = json.loads(recognition_json)
            except json.JSONDecodeError:
                pass

        # Run Vision Agent
        vision_result = await call_vision_agent(images)

        # Run Orchestrator Agent
        motion_plan = await call_orchestrator_agent(command, vision_result, recognition_data)

        return CouncilResponse(
            vision=vision_result,
            plan=motion_plan,
            success=True
        )

    except HTTPException:
        raise
    except Exception as e:
        return CouncilResponse(
            vision=VisionResult(
                scene_summary="Error during analysis",
                objects=[],
                robot_visible=False,
                table_visible=False,
                uncertainties=[str(e)]
            ),
            plan=MotionPlan(
                intent=command,
                action="unknown",
                waypoints=[],
                confidence=0.0,
                reasoning=f"Error: {e}",
                warnings=[str(e)]
            ),
            success=False,
            error=str(e)
        )

# Endpoint to run only Vision Agent
@app.post("/api/vision", response_model=VisionResult)
async def analyze_vision():
    images = []
    for cam_name in ["top_camera", "front_camera", "side_camera"]:
        img_path = FRAMES_DIR / f"{cam_name}.png"
        if img_path.exists():
            images.append((cam_name, img_path.read_bytes()))

    return await call_vision_agent(images)

# Endpoint to run only Orchestrator Agent
@app.post("/api/plan", response_model=MotionPlan)
async def create_plan(
    command: str = Form(...),
    vision_json: str = Form(...)
):
    vision_data = json.loads(vision_json)
    vision_result = VisionResult(**vision_data)
    return await call_orchestrator_agent(command, vision_result)

# Verify so it can recursively fix failed actions
class VerificationResult(BaseModel):
    success: bool = True
    action_completed: bool = Field(..., description="Whether the original action was completed")
    reasoning: str = Field(..., description="Explanation of why action succeeded or failed")
    vision: Optional[VisionResult] = None
    plan: Optional[MotionPlan] = Field(None, description="New plan if action not completed")

# Call verification agent
async def call_verification_agent(
    original_command: str,
    images: list[tuple[str, bytes]]
) -> dict:
    if not OPENAI_API_KEY:
        # Mock verification
        return {
            "action_completed": True,
            "reasoning": "Mock verification: Assuming action completed (no API key)",
            "confidence": 0.5
        }

    # Build message content with images
    content = [{"type": "text", "text": f"""Original command: "{original_command}"
    Please analyze the current scene and determine if this action was completed successfully.
    Look at the cube positions relative to the colored reference markers to determine if cubes have moved."""}]

    camera_descriptions = {
        "top_camera": "TOP CAMERA - Bird's eye view from above",
        "front_camera": "FRONT CAMERA - View from in front of the table",
        "side_camera": "SIDE CAMERA - View from the side"
    }

    for cam_name, img_bytes in images:
        b64_data = base64.b64encode(img_bytes).decode("utf-8")
        cam_desc = camera_descriptions.get(cam_name, f"[{cam_name}]")
        content.append({"type": "text", "text": f"\n{cam_desc}"})
        content.append({
            "type": "image_url",
            "image_url": {
                "url": f"data:image/png;base64,{b64_data}",
                "detail": "high"
            }
        })

    async with httpx.AsyncClient(timeout=TIMEOUT_SECONDS) as client:
        response = await client.post(
            f"{OPENAI_BASE_URL}/chat/completions",
            headers={
                "Authorization": f"Bearer {OPENAI_API_KEY}",
                "Content-Type": "application/json"
            },
            json={
                "model": VISION_MODEL,
                "messages": [
                    {"role": "system", "content": VERIFICATION_SYSTEM_PROMPT},
                    {"role": "user", "content": content}
                ],
                "max_tokens": 1000,
                "response_format": {"type": "json_object"}
            }
        )

        if response.status_code != 200:
            raise HTTPException(500, f"Verification API error: {response.text}")

        result = response.json()
        raw_text = result["choices"][0]["message"]["content"]

        try:
            return json.loads(raw_text)
        except json.JSONDecodeError:
            json_match = re.search(r"```(?:json)?\s*([\s\S]*?)\s*```", raw_text)
            if json_match:
                return json.loads(json_match.group(1))
            return {
                "action_completed": False,
                "reasoning": f"Failed to parse verification response: {raw_text[:200]}",
                "confidence": 0.0
            }

# Endpoint to verify action completion
@app.post("/api/verify", response_model=VerificationResult)
async def verify_action(
    original_command: str = Form(..., description="The original command to verify")
):
    try:
        # Load current camera images
        images = []
        for cam_name in ["top_camera", "front_camera", "side_camera"]:
            img_path = FRAMES_DIR / f"{cam_name}.png"
            if img_path.exists():
                images.append((cam_name, img_path.read_bytes()))

        # Run verification
        verify_result = await call_verification_agent(original_command, images)

        action_completed = verify_result.get("action_completed", False)
        reasoning = verify_result.get("reasoning", "No reasoning provided")

        print(f"[Verify] Command: '{original_command}'")
        print(f"[Verify] Completed: {action_completed}")
        print(f"[Verify] Reason: {reasoning}")

        # If action not completed, get a new plan
        new_plan = None
        vision_result = None

        if not action_completed:
            # Get fresh vision analysis
            vision_result = await call_vision_agent(images)

            # Get a new plan with context about the failure
            retry_command = f"RETRY: {original_command} - Previous attempt failed. Reason: {reasoning}. Please create a new plan to complete this action."
            new_plan = await call_orchestrator_agent(retry_command, vision_result)

        return VerificationResult(
            success=True,
            action_completed=action_completed,
            reasoning=reasoning,
            vision=vision_result,
            plan=new_plan
        )

    except Exception as e:
        print(f"[Verify] Error: {e}")
        return VerificationResult(
            success=False,
            action_completed=False,
            reasoning=f"Verification error: {str(e)}",
            vision=None,
            plan=None
        )


if __name__ == "__main__":
    import uvicorn
    print("Starting Apollo AI Council Server...")
    print(f"API Key configured: {bool(OPENAI_API_KEY)}")
    print(f"Frames directory: {FRAMES_DIR}")
    uvicorn.run(app, host="0.0.0.0", port=8000)
