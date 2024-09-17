# WALL-E: LLM-based Navigation Robot Using Multimodal Sensory Approach

## Project Overview
WALL-E is an advanced robot that utilizes an LLM (Large Language Model) to navigate through complex environments. Using a multimodal sensory approach that integrates **LIDAR** and **YOLO-based vision inference**, WALL-E is capable of analyzing its surroundings, making decisions, and navigating autonomously. The robot communicates with the LLM (LLAMA 3.1) for inference processing and command generation.

---

## To-do List:

### 1. **LIDAR Addition**
   - Integrate a LIDAR sensor for environmental mapping and obstacle detection.

### 2. **Inferences Messages Service Modification**
   - Modify the service to handle LIDAR inferences along with vision-based inferences (YOLO).

### 3. **LIDAR Data Distance Point Map Broadcasting**
   - Broadcast LIDAR data in the form of a distance point map over a ROS topic for real-time processing.

### 4. **Receiving LIDAR Data and YOLO Inferences, and Conversion to XML**
   - Process the combined LIDAR and YOLO inference data and convert it into an XML format for easy handling and transmission.

### 5. **Publishing XML Over `/scenedata` Topic**
   - Publish the XML-formatted data over the `/scenedata` topic for further processing by the LLM and other modules.

### 6. **LLM API Integration (LLAMA 3.1)**
   - Integrate the **LLAMA 3.1 API** to receive and process data from the `/scenedata` topic.
   - Subscribe to the `/scenedata` topic and use the LLM for intelligent decision-making based on the scene information.

### 7. **Control System Rework**
   - Rework the robot’s control system to integrate LLM-based decision-making and commands for movement.

### 8. **Commands Generation**
   - Develop the logic for generating movement and action commands based on LIDAR and YOLO data processed by the LLM.

### 9. **ATT Integration**
   - Integrate **ATT** (Augmented Telemetry Transfer) for enhanced telemetry and data transmission.

### 10. **Algorithm Design**
   - **Movement Process**:
     1. Upon receiving a command, the robot will rotate **360 degrees**.
     2. After each **30-degree** increment, the robot will send scene inferences to the LLM.
     3. The LLM will then command the robot to move in a specific direction based on inferences.
     4. Repeat the process until the destination is reached.

### 11. **Task Scenario Creation**
   - Create various navigation scenarios for the robot to perform tasks such as object retrieval, pathfinding, and obstacle avoidance.

### 12. **Audio to Text API Integration**
   - Integrate an **Audio to Text API** for voice-based command input, allowing the robot to understand spoken commands and execute actions accordingly.

---

## Project Roadmap

1. **Multimodal Sensor Integration**
   - Combine LIDAR and camera data for a richer understanding of the robot’s surroundings.
   
2. **LLM-Based Decision Making**
   - Use the LLAMA 3.1 LLM for scene analysis, inference, and intelligent decision-making.

3. **Command Execution and Control**
   - Develop the control system to execute the LLM-generated commands and perform autonomous navigation.

4. **Telemetry and Data Transmission**
   - Implement ATT for high-efficiency telemetry and communication with the control systems.

---

## Technologies Used

- **ROS2**: For sensor data collection, publishing, and robot control.
- **LIDAR**: For distance mapping and obstacle detection.
- **YOLO**: For object recognition and scene analysis.
- **LLAMA 3.1 API**: For LLM-based decision-making.
- **Python**: For control algorithms, data processing, and LLM integration.
- **ATT (Augmented Telemetry Transfer)**: For telemetry and communication.

---

## Installation and Setup

sudo apt install ros-humble-desktop  # For ROS2 Humble
pip install openai  # For LLM integration
pip install requests

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/WALL-E.git
   cd WALL-E
