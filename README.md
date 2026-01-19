# AI-Vision Robotic Arm – Color Picker Prototype

> **Prototipo de brazo robótico UR5e con visión artificial y clasificación de objetos mediante GPT-4o Vision en simulación Webots.**

---

##  Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Authors](#authors)
- [Core Logic / Mechanics](#core-logic--mechanics)
- [System Architecture](#system-architecture)
  - [Directory Structure](#directory-structure)
  - [Component Configuration](#component-configuration)
- [Technical Specifications](#technical-specifications)
  - [Robot Specifications](#robot-specifications)
  - [Behavioral States](#behavioral-states)
  - [Motor Configuration](#motor-configuration)
- [Technical Implementation](#technical-implementation)
  - [Vision Pipeline](#vision-pipeline)
  - [Motion Control Algorithm](#motion-control-algorithm)
  - [Position Clamping Function](#position-clamping-function)
- [Installation & Setup](#installation--setup)
  - [Prerequisites](#prerequisites)
  - [Installation Steps](#installation-steps)
  - [Execution](#execution)
- [Configuration](#configuration)
  - [Adjustable Parameters](#adjustable-parameters)
  - [Tuning Guide](#tuning-guide)
- [Known Limitations](#known-limitations)
- [Contributing & Development Guidelines](#contributing--development-guidelines)

---

## Overview

Este proyecto implementa un **sistema robótico autónomo de pick-and-place** que integra visión artificial con inteligencia artificial generativa para la clasificación de objetos por color. El sistema utiliza:

| Tecnología | Propósito |
|------------|-----------|
| **Webots R2025a** | Simulación física 3D del entorno robótico |
| **Universal Robots UR5e** | Brazo robótico industrial de 6 ejes |
| **Robotiq 3F Gripper** | Pinza de 3 dedos para manipulación de objetos |
| **OpenAI GPT-4o Vision** | Reconocimiento y clasificación de colores mediante IA |
| **OpenCV** | Captura y procesamiento de imágenes desde webcam |
| **Python 3.x** | Lenguaje de implementación del controlador |

El flujo operacional consiste en capturar una imagen de un objeto físico mediante la webcam del usuario, enviarla a la API de OpenAI para identificación del color dominante, y ejecutar una secuencia de movimientos predefinidos para transportar el objeto virtual correspondiente a su contenedor designado.

---

## Features

###  Arquitectura
- **Integración Webots-OpenAI**: Pipeline completo desde captura hasta ejecución motora
- **Auto-discovery de motores**: Detección automática de dispositivos del robot
- **Sistema de sensores de posición**: Habilitación dinámica de encoders

###  Comunicación
- **API REST OpenAI**: Envío de imágenes codificadas en Base64
- **Protocolo de Webots**: Comunicación síncrona con timestep configurable
- **Interfaz HID**: Captura de eventos de teclado para trigger manual

###  Mecánicas
- **Gripper adaptativo**: Control seguro con límites de posición
- **Keyframe animation**: Trayectorias intermedias para evitar colisiones
- **Multi-object support**: Mapeo de colores a objetos/destinos

---

## Authors

| Nombre | Rol |
|--------|-----|
| Mascaró | Developer |
| Gonzalo | Developer |
| Marcos Cabrero | Developer |

---

## Core Logic / Mechanics

### Flujo de Operación Principal

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   Usuario       │────▶│   Webcam         │────▶│   OpenCV        │
│   (SPACEBAR)    │     │   Capture        │     │   Encode B64    │
└─────────────────┘     └──────────────────┘     └────────┬────────┘
                                                          │
                                                          ▼
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   Robot Arm     │◀────│   Controller     │◀────│   OpenAI API    │
│   Pick & Place  │     │   Parse Color    │     │   GPT-4o Vision │
└─────────────────┘     └──────────────────┘     └─────────────────┘
```

### Mapeo de Colores a Objetos

| Color Detectado | Objeto Virtual | Posición Destino |
|-----------------|----------------|------------------|
| `red` | `red_cube` | `red_cube_end` |
| `green` | `green_sphere` | `green_sphere_end` |
| `blue` | `blue_cylinder` | `blue_cylinder_end` |
| `none` | — | No action |

### Secuencia de Pick-and-Place

| Paso | Acción | Wait Steps |
|------|--------|------------|
| 1 | Mover a posición `home` | 30 |
| 2 | Mover a posición `{object}_above` | 30 |
| 3 | Abrir gripper | 80 |
| 4 | Mover a posición `{object}` | 30 |
| 5 | Cerrar gripper | 80 |
| 6 | Mover a posición `{object}_end` | 150 |
| 7 | Abrir gripper | — |
| 8 | Retornar a `home` | 150 |

---

## System Architecture

### Directory Structure

```
RoboticsFinalPrototypes/
│
├── 📄 README.md                          # Documentación del proyecto
├── 📄 .env.example                       # Plantilla de variables de entorno
├── 📄 .gitignore                         # Archivos ignorados por Git
│
├── 📁 controllers/
│   └── 📁 auto_controller/
│       └── 🐍 auto_controller.py         # Controlador principal (215 LOC)
│
└── 📁 worlds/
    └── 🌍 color_picker_robot.wbt         # Mundo de simulación Webots
```

| Directorio | Descripción |
|------------|-------------|
| `controllers/` | Contiene los controladores Python para Webots |
| `controllers/auto_controller/` | Controlador principal del sistema |
| `worlds/` | Archivos de mundo `.wbt` para la simulación |

### Component Configuration

#### Componentes de Hardware Simulado

| Componente | Modelo | Especificación |
|------------|--------|----------------|
| **Brazo Robótico** | UR5e | 6 DOF, alcance 850mm |
| **Gripper** | Robotiq 3F | 3 dedos adaptativos |
| **Cámara** | Virtual Camera | 640×480 px |
| **Arena** | RectangleArena | 3m × 3m |

#### Objetos del Mundo

| Objeto | Color RGB | Tamaño (m) | Masa (kg) |
|--------|-----------|------------|-----------|
| `red_cube` | `(1, 0, 0)` | 0.1 × 0.1 × 0.1 | 0.1 |
| `green_sphere` | `(0, 1, 0)` | 0.1 × 0.1 × 0.1 | 0.1 |
| `blue_cylinder` | `(0, 0, 1)` | 0.1 × 0.1 × 0.1 | 0.1 |
| `box_1`, `box_2`, `box_3` | `(0.8, 0.8, 0.8)` | 0.15 × 0.15 × 0.1 | — |

---

## Technical Specifications

### Robot Specifications

| Parámetro | Valor |
|-----------|-------|
| **Modelo** | Universal Robots UR5e |
| **Grados de Libertad** | 6 (+ gripper) |
| **Controlador** | `auto_controller.py` |
| **Timestep Base** | Dinámico (`robot.getBasicTimeStep()`) |
| **Velocidad Arm Motors** | 1.0 rad/s |
| **Velocidad Gripper Motors** | 0.5 rad/s |

### Behavioral States

| Estado | Descripción | Trigger |
|--------|-------------|---------|
| **IDLE** | Robot en posición home, esperando input | Inicio de simulación |
| **CAPTURING** | Captura de imagen desde webcam | Tecla SPACEBAR |
| **ANALYZING** | Enviando imagen a OpenAI API | Post-captura |
| **PICKING** | Ejecutando secuencia de recogida | Color válido detectado |
| **PLACING** | Ejecutando secuencia de colocación | Objeto agarrado |
| **RETURNING** | Retornando a posición home | Post-colocación |

### Motor Configuration

#### Arm Motors (6 joints)

| Joint | Posición Home (rad) | Descripción |
|-------|---------------------|-------------|
| Joint 0 | `0.00` | Base rotation |
| Joint 1 | `-1.57` | Shoulder |
| Joint 2 | `1.57` | Elbow |
| Joint 3 | `-1.57` | Wrist 1 |
| Joint 4 | `-1.57` | Wrist 2 |
| Joint 5 | `0.00` | Wrist 3 |

#### Gripper Configuration

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `GRIPPER_OPEN` | `-0.1` rad | Posición de apertura |
| `GRIPPER_CLOSE` | `0.6` rad | Posición de cierre |

---

## Technical Implementation

### Vision Pipeline

```python
# 1. Captura de frame desde webcam
ret, frame = self.webcam.read()

# 2. Codificación a Base64
_, buf = cv2.imencode(".jpg", frame)
image_b64 = base64.b64encode(buf).decode()

# 3. Envío a OpenAI GPT-4o Vision
response = self.client.chat.completions.create(
    model="gpt-4o",
    messages=[{
        "role": "user",
        "content": [
            {"type": "text", "text": prompt},
            {"type": "image_url", "image_url": {
                "url": f"data:image/jpeg;base64,{image_b64}",
                "detail": "low"
            }}
        ]
    }],
    max_tokens=5,
    temperature=0
)
```

#### Parámetros de OpenAI API

| Parámetro | Valor | Justificación |
|-----------|-------|---------------|
| `model` | `gpt-4o` | Modelo con capacidades de visión |
| `detail` | `low` | Reducción de tokens/costo |
| `max_tokens` | `5` | Respuesta mínima (1 palabra) |
| `temperature` | `0` | Determinismo máximo |

### Motion Control Algorithm

La secuencia de movimiento utiliza **keyframe interpolation** para evitar colisiones:

```
Position Flow:
home → {object}_above → {object} → {object}_end → home
           │               │            │
           └── clearance ──┴── pickup ──┴── dropoff
```

#### Posiciones Predefinidas (radianes)

| Posición | J0 | J1 | J2 | J3 | J4 | J5 |
|----------|-------|-------|------|-------|-------|------|
| `home` | 0.00 | -1.57 | 1.57 | -1.57 | -1.57 | 0.00 |
| `red_cube` | 0.50 | -1.27 | 1.87 | -2.27 | -1.57 | 0.00 |
| `red_cube_above` | 0.50 | -1.27 | 1.57 | -2.07 | -1.57 | 0.00 |
| `red_cube_end` | -3.90 | -1.11 | 1.37 | -1.87 | -1.57 | 0.00 |
| `green_sphere` | -0.10 | -1.07 | 1.67 | -2.07 | -1.57 | 0.00 |
| `green_sphere_above` | -0.10 | -1.47 | 1.67 | -1.87 | -1.57 | 0.00 |
| `green_sphere_end` | -3.40 | -1.11 | 1.37 | -1.87 | -1.57 | 0.00 |
| `blue_cylinder` | -0.90 | -1.27 | 1.87 | -2.27 | -1.57 | 0.10 |
| `blue_cylinder_above` | -0.60 | -1.37 | 1.37 | -1.57 | -1.57 | 0.00 |
| `blue_cylinder_end` | -2.90 | -1.11 | 1.37 | -1.87 | -1.57 | 0.00 |

### Position Clamping Function

Para garantizar movimientos seguros dentro de los límites físicos del motor:

```python
def set_motor_position_safe(motor: Motor, target: float):
    """
    Aplica clamping a la posición objetivo dentro de los límites del motor.
    
    Args:
        motor: Instancia del motor Webots
        target: Posición objetivo en radianes
    """
    mn = motor.getMinPosition()  # Límite inferior
    mx = motor.getMaxPosition()  # Límite superior

    if mn != float("-inf") and target < mn:
        target = mn
    if mx != float("inf") and target > mx:
        target = mx

    motor.setPosition(target)
```

---

## Installation & Setup

### Prerequisites

| Requisito | Versión | Notas |
|-----------|---------|-------|
| **Webots** | R2025a | Simulador robótico |
| **Python** | ≥ 3.8 | Runtime del controlador |
| **OpenCV** | ≥ 4.5 | Procesamiento de imagen |
| **OpenAI SDK** | ≥ 1.0 | Cliente de API |
| **Webcam** | — | Hardware requerido |
| **API Key OpenAI** | — | Con acceso a GPT-4o |

### Installation Steps

```bash
# 1. Clonar el repositorio
git clone https://github.com/Pringles505/RoboticsFinalPrototypes.git
cd RoboticsFinalPrototypes

# 2. Instalar dependencias Python
pip install opencv-python numpy openai python-dotenv

# 3. Configurar API Key de OpenAI
cp .env.example .env
# Editar .env y añadir tu API Key:
# OPENAI_API_KEY=tu-api-key-aqui
```

> ⚠️ **Importante**: El archivo `.env` está incluido en `.gitignore` para proteger tu API Key. Nunca subas credenciales al repositorio.

### Execution

```bash
# 1. Abrir Webots
webots worlds/color_picker_robot.wbt

# 2. Iniciar simulación (Play button o Ctrl+P)
# 3. La webcam se activará automáticamente
# 4. Presionar SPACEBAR para capturar y procesar
```

---

## Configuration

### Adjustable Parameters

```python
# === OPENAI CONFIGURATION (.env file) ===
# OPENAI_API_KEY=tu-api-key     # API Key (REQUERIDO) - configurar en .env

# === WEBCAM CONFIGURATION ===
self.webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   # Ancho de captura
self.webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Alto de captura

# === MOTOR VELOCITIES ===
for m in self.arm_motors:
    m.setVelocity(1.0)           # Velocidad del brazo (rad/s)
for g in self.gripper_motors:
    g.setVelocity(0.5)           # Velocidad del gripper (rad/s)

# === GRIPPER POSITIONS ===
self.GRIPPER_OPEN = -0.1         # Posición abierta
self.GRIPPER_CLOSE = 0.6         # Posición cerrada

# === TIMING ===
self.wait(steps=150)             # Steps de espera por defecto
self.wait(80)                    # Steps para operaciones de gripper
self.wait(30)                    # Steps para movimientos de brazo
```

### Tuning Guide

#### Ajuste de Velocidad

| Escenario | Arm Velocity | Gripper Velocity | Wait Steps |
|-----------|--------------|------------------|------------|
| **Precisión alta** | 0.5 | 0.3 | 200 |
| **Balanceado** | 1.0 | 0.5 | 150 |
| **Alta velocidad** | 2.0 | 1.0 | 80 |

#### Calibración de Posiciones

Para calibrar nuevas posiciones de objetos:

1. Ejecutar simulación en modo **pausa**
2. Usar el panel de **Position Sensors** de Webots
3. Mover manualmente el robot a la posición deseada
4. Leer valores de los sensores
5. Actualizar diccionario `self.positions`

---

## Known Limitations

| Comportamiento | Causa | Mitigación |
|----------------|-------|------------|
| **Posiciones hardcodeadas** | No hay path planning dinámico | Calibrar manualmente para nuevos objetos |
| **Sin detección de colisiones** | Trayectorias predefinidas | Usar posiciones `_above` intermedias |
| **Dependencia de iluminación** | GPT-4o sensible a condiciones de luz | Usar iluminación consistente |
| **Latencia de API** | Llamada HTTP a OpenAI | ~1-3s por detección |
| **Solo 3 colores** | Mapeo fijo en `color_to_object` | Extender diccionario para más colores |
| **Gripper genérico** | Valores de apertura/cierre fijos | Ajustar `GRIPPER_OPEN`/`GRIPPER_CLOSE` |
| **Webcam index 0** | Asume primera cámara disponible | Cambiar índice en `cv2.VideoCapture(n)` |
| **Sin recuperación de errores** | Fallos de API terminan ejecución | Implementar try/catch con reintentos |

---

## Contributing & Development Guidelines

### Git Flow

```bash
# 1. Fork del repositorio
# 2. Crear rama feature
git checkout -b feature/nueva-funcionalidad

# 3. Commits atómicos
git commit -m "feat: descripción concisa del cambio"

# 4. Push y Pull Request
git push origin feature/nueva-funcionalidad
```

### Convención de Commits

| Prefijo | Uso |
|---------|-----|
| `feat:` | Nueva funcionalidad |
| `fix:` | Corrección de bug |
| `docs:` | Cambios en documentación |
| `refactor:` | Refactorización de código |
| `test:` | Añadir o modificar tests |

### Estilo de Código

- **PEP 8** para Python
- **Type hints** recomendados
- **Docstrings** en formato Google
- **Nombres descriptivos** para variables y funciones

### Estructura de Nuevos Controladores

```python
"""
Docstring del módulo.
"""
from controller import Robot, Motor

# Constants
CONSTANT_NAME = value

# Helper functions
def helper_function():
    """Docstring."""
    pass

# Main class
class ControllerName:
    """Docstring."""
    
    def __init__(self):
        pass
    
    def run(self):
        pass

if __name__ == "__main__":
    controller = ControllerName()
    controller.run()
```

---

## License

Este proyecto se proporciona como **prototipo experimental** con fines educativos bajo licencia **MIT**.

---

<p align="center">
  <b>IMMUNE Technology Institute – Robotics Final Project</b><br>
  <i>AI-Powered Vision System for Robotic Manipulation</i>
</p>
