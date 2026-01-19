# AI-Council Driven Robotic Arm – Prototype

## Descripción general

Este proyecto es un **prototipo de brazo robótico controlado por visión artificial asistida por OpenAI**. El sistema utiliza la **webcam del usuario** para identificar el **color de un objeto físico** colocado frente a la cámara y, en función de ese color, el brazo robótico **selecciona la caja correspondiente y la mueve** dentro de un entorno de simulación en **Webots**.

El reconocimiento del color se realiza enviando una imagen capturada por la webcam a la **API de OpenAI**, que procesa la imagen y devuelve información sobre el color detectado. A partir de esa respuesta, el controlador del robot ejecuta una secuencia de movimientos predefinida.

Este proyecto es un **proof of concept**, no un sistema autónomo completo.

---

## Arquitectura del sistema

El sistema está compuesto por los siguientes elementos:

- **Webots Simulation**
  - Entorno 3D donde se ejecuta el brazo robótico.
  - Incluye:
    - Brazo robótico
    - Cajas de diferentes colores (destinos)
    - Mundo preconfigurado

- **Controller en Python**
  - Controla el brazo robótico dentro de Webots.
  - Gestiona:
    - Captura de imagen desde la webcam
    - Envío de la imagen a OpenAI
    - Interpretación del color detectado
    - Ejecución de secuencias de movimiento

- **Webcam del usuario**
  - Se activa automáticamente cuando la simulación está en ejecución. (Sigue siendo necesario un previo consentimiento por parte del usuario para usar la webcam)
  - Permite mostrar un objeto físico al sistema.

- **OpenAI API**
  - Recibe la imagen capturada.
  - Analiza la imagen según un prompt definido en el controller.
  - Devuelve el color del objeto detectado.

---

## Instalación y requisitos

### Requisitos

- Webots (versión compatible con el proyecto)
- Python 3.x
- Webcam funcional
- API Key de OpenAI

---

## Uso

1. Abre el archivo del mundo (`.wbt`) en Webots.
2. Inicia la simulación.
3. Cuando la simulación esté en ejecución:
   - Se abrirá automáticamente la webcam.
4. Coloca un objeto frente a la cámara.
5. Asegúrate de que el foco del mouse esté en Webots.
6. Presiona la tecla **ESPACIO** para capturar la imagen.
7. El sistema procesará la imagen y el brazo robótico ejecutará la acción correspondiente.

---

## Estado actual del proyecto

- Los movimientos del robot están **hardcodeados**.
- Cada color tiene:
  - Una posición de recogida
  - Un keyframe intermedio para evitar colisiones
  - Una posición final de colocación

---

## Licencia

Este proyecto se proporciona como **prototipo experimental** y con fines educativos.
