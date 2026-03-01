# Conversational Robotics with LLMs

## Introduction

**Conversational robotics** combines large language models (LLMs) with robotic systems to enable natural language interaction. This allows users to command robots using everyday language instead of programming or specialized interfaces, making robotics accessible to everyone.

## Architecture Overview

A conversational robotics system consists of several components:

```
User Speech → Speech Recognition → LLM → Action Planning → Robot Execution
                                    ↓
                              Context & Memory
                                    ↓
                              Robot Perception
```

**Key components:**
1. **Speech Recognition**: Convert speech to text (Whisper, Google Speech API)
2. **Language Model**: Understand intent and generate plans (GPT-4, Claude)
3. **Action Planner**: Translate high-level commands to robot actions
4. **Execution Engine**: Execute actions on robot hardware
5. **Perception**: Provide environmental context to LLM
6. **Memory**: Maintain conversation history and world state

## Speech Recognition with OpenAI Whisper

### Installation

```bash
pip install openai-whisper
```

### Basic Usage

```python
import whisper
import pyaudio
import wave
import numpy as np

class SpeechRecognizer:
    def __init__(self, model_size='base'):
        """
        model_size: 'tiny', 'base', 'small', 'medium', 'large'
        """
        self.model = whisper.load_model(model_size)
        self.audio = pyaudio.PyAudio()

    def record_audio(self, duration=5, sample_rate=16000):
        """Record audio from microphone"""
        stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=sample_rate,
            input=True,
            frames_per_buffer=1024
        )

        print("Recording...")
        frames = []

        for _ in range(0, int(sample_rate / 1024 * duration)):
            data = stream.read(1024)
            frames.append(data)

        print("Recording finished")

        stream.stop_stream()
        stream.close()

        # Convert to numpy array
        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
        audio_data = audio_data.astype(np.float32) / 32768.0

        return audio_data

    def transcribe(self, audio_data):
        """Transcribe audio to text"""
        result = self.model.transcribe(audio_data)
        return result['text']

    def listen_and_transcribe(self, duration=5):
        """Record and transcribe in one step"""
        audio = self.record_audio(duration)
        text = self.transcribe(audio)
        return text
```

### ROS 2 Integration

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')

        self.recognizer = SpeechRecognizer(model_size='base')

        # Publisher for transcribed text
        self.text_pub = self.create_publisher(
            String,
            '/speech/text',
            10
        )

        # Timer for continuous listening
        self.create_timer(0.1, self.listen_callback)
        self.listening = False

    def listen_callback(self):
        """Continuously listen for speech"""
        if not self.listening:
            self.listening = True

            try:
                text = self.recognizer.listen_and_transcribe(duration=3)

                if text.strip():
                    self.get_logger().info(f'Recognized: {text}')

                    msg = String()
                    msg.data = text
                    self.text_pub.publish(msg)

            except Exception as e:
                self.get_logger().error(f'Recognition error: {e}')

            finally:
                self.listening = False
```

## LLM Integration with OpenAI

### Basic LLM Client

```python
from openai import OpenAI
import json

class RobotLLM:
    def __init__(self, api_key):
        self.client = OpenAI(api_key=api_key)
        self.conversation_history = []

        # System prompt defines robot capabilities
        self.system_prompt = """You are a helpful humanoid robot assistant. You can:
- Navigate to locations (navigate_to)
- Pick up objects (pick_up)
- Place objects (place_at)
- Wave and gesture (wave, nod)
- Answer questions about your environment

When given a command, respond with a JSON action plan.

Example:
User: "Go to the kitchen and bring me a cup"
Response: {
  "actions": [
    {"type": "navigate_to", "location": "kitchen"},
    {"type": "pick_up", "object": "cup"},
    {"type": "navigate_to", "location": "user"},
    {"type": "hand_over", "object": "cup"}
  ],
  "explanation": "I'll go to the kitchen, pick up a cup, and bring it to you."
}
"""

    def process_command(self, user_input, context=None):
        """Process user command and generate action plan"""
        # Add context if provided
        if context:
            context_str = f"\nCurrent context: {json.dumps(context)}"
            user_input = user_input + context_str

        # Add to conversation history
        self.conversation_history.append({
            "role": "user",
            "content": user_input
        })

        # Call LLM
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.system_prompt},
                *self.conversation_history
            ],
            temperature=0.7,
            max_tokens=500
        )

        assistant_message = response.choices[0].message.content

        # Add to history
        self.conversation_history.append({
            "role": "assistant",
            "content": assistant_message
        })

        # Parse action plan
        try:
            action_plan = json.loads(assistant_message)
            return action_plan
        except json.JSONDecodeError:
            # If not JSON, treat as conversational response
            return {
                "actions": [],
                "explanation": assistant_message
            }

    def clear_history(self):
        """Clear conversation history"""
        self.conversation_history = []
```

### Function Calling for Structured Actions

```python
class RobotLLMWithFunctions:
    def __init__(self, api_key):
        self.client = OpenAI(api_key=api_key)

        # Define available functions
        self.functions = [
            {
                "name": "navigate_to",
                "description": "Navigate to a specific location",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "location": {
                            "type": "string",
                            "description": "The target location (e.g., 'kitchen', 'living room')"
                        }
                    },
                    "required": ["location"]
                }
            },
            {
                "name": "pick_up_object",
                "description": "Pick up an object",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "object_name": {
                            "type": "string",
                            "description": "Name of the object to pick up"
                        },
                        "location": {
                            "type": "string",
                            "description": "Where the object is located"
                        }
                    },
                    "required": ["object_name"]
                }
            },
            {
                "name": "place_object",
                "description": "Place a held object at a location",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "object_name": {
                            "type": "string",
                            "description": "Name of the object to place"
                        },
                        "location": {
                            "type": "string",
                            "description": "Where to place the object"
                        }
                    },
                    "required": ["object_name", "location"]
                }
            },
            {
                "name": "gesture",
                "description": "Perform a gesture",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "gesture_type": {
                            "type": "string",
                            "enum": ["wave", "nod", "point", "thumbs_up"],
                            "description": "Type of gesture to perform"
                        }
                    },
                    "required": ["gesture_type"]
                }
            }
        ]

    def process_command(self, user_input):
        """Process command using function calling"""
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are a helpful humanoid robot."},
                {"role": "user", "content": user_input}
            ],
            functions=self.functions,
            function_call="auto"
        )

        message = response.choices[0].message

        # Check if function was called
        if message.function_call:
            function_name = message.function_call.name
            function_args = json.loads(message.function_call.arguments)

            return {
                "action": function_name,
                "parameters": function_args
            }
        else:
            return {
                "action": "speak",
                "parameters": {"text": message.content}
            }
```

## Action Execution Engine

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator

class ActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')

        # Navigation
        self.navigator = BasicNavigator()

        # Known locations
        self.locations = {
            'kitchen': {'x': 5.0, 'y': 2.0, 'yaw': 0.0},
            'living_room': {'x': 2.0, 'y': 5.0, 'yaw': 1.57},
            'bedroom': {'x': 8.0, 'y': 8.0, 'yaw': 3.14},
            'user': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        }

        # Speech output
        self.speech_pub = self.create_publisher(String, '/speech/output', 10)

        # Object manipulation (placeholder)
        self.held_object = None

    def execute_action(self, action):
        """Execute a single action"""
        action_type = action['action']
        params = action['parameters']

        if action_type == 'navigate_to':
            return self.navigate_to(params['location'])

        elif action_type == 'pick_up_object':
            return self.pick_up(params['object_name'])

        elif action_type == 'place_object':
            return self.place(params['object_name'], params['location'])

        elif action_type == 'gesture':
            return self.perform_gesture(params['gesture_type'])

        elif action_type == 'speak':
            return self.speak(params['text'])

        else:
            self.get_logger().error(f'Unknown action: {action_type}')
            return False

    def navigate_to(self, location):
        """Navigate to a named location"""
        if location not in self.locations:
            self.speak(f"I don't know where {location} is")
            return False

        self.speak(f"Navigating to {location}")

        loc = self.locations[location]
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = loc['x']
        goal.pose.position.y = loc['y']
        goal.pose.orientation.z = np.sin(loc['yaw'] / 2.0)
        goal.pose.orientation.w = np.cos(loc['yaw'] / 2.0)

        self.navigator.goToPose(goal)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.speak(f"Arrived at {location}")
            return True
        else:
            self.speak(f"Failed to reach {location}")
            return False

    def pick_up(self, object_name):
        """Pick up an object"""
        self.speak(f"Picking up {object_name}")

        # TODO: Implement actual manipulation
        # For now, simulate
        import time
        time.sleep(2)

        self.held_object = object_name
        self.speak(f"I have picked up the {object_name}")
        return True

    def place(self, object_name, location):
        """Place an object"""
        if self.held_object != object_name:
            self.speak(f"I'm not holding a {object_name}")
            return False

        self.speak(f"Placing {object_name} at {location}")

        # TODO: Implement actual placement
        import time
        time.sleep(2)

        self.held_object = None
        self.speak(f"I have placed the {object_name}")
        return True

    def perform_gesture(self, gesture_type):
        """Perform a gesture"""
        self.speak(f"Performing {gesture_type} gesture")

        # TODO: Implement actual gesture control
        import time
        time.sleep(1)

        return True

    def speak(self, text):
        """Output speech"""
        self.get_logger().info(f'Speaking: {text}')

        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)

        return True
```

## Complete Conversational Robot System

```python
class ConversationalRobot(Node):
    def __init__(self):
        super().__init__('conversational_robot')

        # Components
        self.speech_recognizer = SpeechRecognizer()
        self.llm = RobotLLMWithFunctions(api_key=os.getenv('OPENAI_API_KEY'))
        self.action_executor = ActionExecutor()

        # Subscribers
        self.create_subscription(
            String,
            '/speech/text',
            self.speech_callback,
            10
        )

        # State
        self.processing = False

    def speech_callback(self, msg):
        """Handle recognized speech"""
        if self.processing:
            return

        self.processing = True
        user_input = msg.data

        self.get_logger().info(f'User said: {user_input}')

        try:
            # Get context from perception
            context = self.get_current_context()

            # Process with LLM
            action_plan = self.llm.process_command(user_input, context)

            # Execute actions
            if 'actions' in action_plan:
                for action in action_plan['actions']:
                    success = self.action_executor.execute_action(action)
                    if not success:
                        self.action_executor.speak("Sorry, I couldn't complete that action")
                        break

            # Speak explanation
            if 'explanation' in action_plan:
                self.action_executor.speak(action_plan['explanation'])

        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')
            self.action_executor.speak("Sorry, I encountered an error")

        finally:
            self.processing = False

    def get_current_context(self):
        """Get current robot and environment state"""
        # TODO: Implement actual perception
        return {
            'location': 'living_room',
            'visible_objects': ['cup', 'book', 'remote'],
            'held_object': self.action_executor.held_object,
            'battery_level': 85
        }

def main():
    rclpy.init()
    robot = ConversationalRobot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()
```

## Vision-Language-Action (VLA) Models

VLA models directly map visual observations and language commands to robot actions.

```python
class VLAController:
    def __init__(self):
        # Load VLA model (e.g., RT-2, PaLM-E)
        self.model = self.load_vla_model()

    def load_vla_model(self):
        """Load pre-trained VLA model"""
        # Placeholder - would load actual model
        pass

    def predict_action(self, image, text_command):
        """
        Predict robot action from image and text
        Returns: joint positions or end-effector pose
        """
        # Preprocess image
        image_tensor = self.preprocess_image(image)

        # Tokenize text
        text_tokens = self.tokenize_text(text_command)

        # Model inference
        action = self.model(image_tensor, text_tokens)

        return action

    def preprocess_image(self, image):
        """Preprocess image for model"""
        # Resize, normalize, etc.
        pass

    def tokenize_text(self, text):
        """Tokenize text command"""
        pass
```

## Multimodal Interaction

```python
class MultimodalRobot(Node):
    def __init__(self):
        super().__init__('multimodal_robot')

        # Vision
        self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        # Speech
        self.create_subscription(
            String,
            '/speech/text',
            self.speech_callback,
            10
        )

        # Gestures (from vision)
        self.gesture_detector = GestureDetector()

        # State
        self.latest_image = None
        self.detected_gesture = None

    def image_callback(self, msg):
        """Process visual input"""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

        # Detect gestures
        self.detected_gesture = self.gesture_detector.detect(self.latest_image)

        if self.detected_gesture:
            self.handle_gesture(self.detected_gesture)

    def speech_callback(self, msg):
        """Process speech input"""
        # Combine with visual context
        if self.latest_image is not None:
            # Use vision + language
            self.process_multimodal_command(msg.data, self.latest_image)
        else:
            # Language only
            self.process_command(msg.data)

    def handle_gesture(self, gesture):
        """Respond to detected gesture"""
        if gesture == 'wave':
            self.action_executor.perform_gesture('wave')
        elif gesture == 'point':
            # Determine what user is pointing at
            pointed_object = self.detect_pointed_object()
            self.action_executor.speak(f"Are you pointing at the {pointed_object}?")

    def process_multimodal_command(self, text, image):
        """Process command with visual context"""
        # Extract visual features
        objects = self.detect_objects(image)

        # Add to context
        context = {
            'visible_objects': objects,
            'user_gesture': self.detected_gesture
        }

        # Process with LLM
        action_plan = self.llm.process_command(text, context)

        # Execute
        self.execute_plan(action_plan)
```

## Text-to-Speech (TTS)

```python
from gtts import gTTS
import pygame
import io

class TextToSpeech:
    def __init__(self):
        pygame.mixer.init()

    def speak(self, text, lang='en'):
        """Convert text to speech and play"""
        # Generate speech
        tts = gTTS(text=text, lang=lang, slow=False)

        # Save to memory
        fp = io.BytesIO()
        tts.write_to_fp(fp)
        fp.seek(0)

        # Play
        pygame.mixer.music.load(fp)
        pygame.mixer.music.play()

        # Wait for completion
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)
```

## Practical Example: Kitchen Assistant Robot

```python
class KitchenAssistant(ConversationalRobot):
    def __init__(self):
        super().__init__()

        # Kitchen-specific knowledge
        self.llm.system_prompt += """
You are a kitchen assistant robot. You know about:
- Common kitchen items and their locations
- Basic cooking tasks
- Food safety
- Kitchen organization

You can help with:
- Fetching ingredients
- Cleaning up
- Setting the table
- Following recipes
"""

        # Kitchen locations
        self.action_executor.locations.update({
            'refrigerator': {'x': 3.0, 'y': 1.0, 'yaw': 0.0},
            'stove': {'x': 4.0, 'y': 1.0, 'yaw': 0.0},
            'sink': {'x': 5.0, 'y': 1.0, 'yaw': 0.0},
            'counter': {'x': 4.5, 'y': 2.0, 'yaw': 1.57},
            'dining_table': {'x': 2.0, 'y': 4.0, 'yaw': 3.14}
        })

    def handle_recipe_command(self, recipe_name):
        """Help with a recipe"""
        # Get recipe steps from LLM
        prompt = f"What are the steps to make {recipe_name}? List ingredients I need to fetch."

        response = self.llm.process_command(prompt)

        # Execute recipe steps
        for action in response['actions']:
            self.action_executor.execute_action(action)
```

## Summary

- **Conversational robotics** enables natural language control of robots
- **Speech recognition** (Whisper) converts voice to text
- **LLMs** (GPT-4, Claude) understand intent and generate plans
- **Function calling** provides structured action outputs
- **Action executors** translate plans to robot commands
- **VLA models** directly map vision + language to actions
- **Multimodal interaction** combines speech, vision, and gestures
- **TTS** provides voice feedback to users
- Complete systems integrate all components for seamless interaction

This completes the Physical AI & Humanoid Robotics textbook content!
