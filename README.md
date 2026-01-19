### Prototype for AI-Council driven robotic arm
## Use

1. Add openAI API key to hardcoded line 11 in the controller file, fuck safety in future replace with .env file.

2. Open the world file in webots. If the simulation is running a python script will trigger the webcam to open.

3. !IMPORTANT WHILE MOUSE FOCUS IS ON WEBOTS press space to take a picture. The picture is saved in a temp file and packaged in the API call to openAI with the prompt on line 153.

Notes:

Robot movement is currently hardcoded. With a pickup position for each robot, a mid-pickup keyframe to ensure robot movements doesnt hinder the world and a place keyframe. 
