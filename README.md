# The A.P.O.LL.O Arm Project- Asimovs Program Operated LLM-Driven Omnidirectional Arm

A Webots robot simulation controlled by an AI council divided into **Vision** and **Orchestrator** agents. Send  language commands like "push the blue cube" and watch the robot execute them.

## Setup

### Prerequisites

- [Webots R2025a](https://cyberbotics.com/)
- Python 3.10+
- OpenAI API key (for AI council)

### Installation

1. **Clone/copy the project** to your Webots projects directory

2. **Set up Python environment**:
   ```bash
   cd Nexus
   python -m venv .venv

   # Windows
   .venv\Scripts\activate

   pip install -r ai_council/requirements.txt
   ```

3. **Configure environment**:
   ```bash
   # .env and add your OpenAI API key
   # OPENAI_API_KEY=sk-your-key-here
   ```

## Running

### 1. Start the AI Council Server

```bash
cd Nexus/ai_council
python server.py
```

Or with uvicorn (recommended):
```bash
uvicorn server:app --host 0.0.0.0 --port 8000 --reload
```

The server will be available at `http://localhost:8000`

### 2. Open Webots Simulation

1. Open Webots
2. File → Open World
3. Navigate to `Apollo/worlds/arena.wbt`
4. The simulation should start automatically

tip: Pause and restart it

### 3. Send Commands

In a new terminal:
```bash
cd Apollo
python command_terminal.py
```

Then type commands:
```
apollo> push the blue cube
apollo> move to the red cube
apollo> point at the green cube
apollo> status
```