from ctransformers import AutoModelForCausalLM
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray
import torch
from PIL import Image
from transformers import BlipProcessor, BlipForConditionalGeneration
import os
import pyaudio
import numpy as np
from transformers import WhisperProcessor, WhisperForConditionalGeneration
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import sys

    
reached = False
goal_achieved = False

class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = theta

        goal_msg.pose = goal_pose

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        global goal_achieved

        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        goal_achieved = True
        # rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Received feedback: {feedback}')

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_1 = self.create_publisher(Float64MultiArray, '/left_joint_1_position_controller/commands', 10)
        self.publisher_2 = self.create_publisher(Float64MultiArray, '/left_joint_2_position_controller/commands', 10)
        self.publisher_3 = self.create_publisher(Float64MultiArray, '/right_joint_1_position_controller/commands', 10)
        self.publisher_4 = self.create_publisher(Float64MultiArray, '/right_joint_2_position_controller/commands', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish every 0.1 seconds (10 Hz)
        
        # Define different target positions
        self.poses = {
            'holding': [0.288, 0.832, 0.288, 0.832],
            'idle': [1.614, -3.142, 1.614, -3.142],
            'non_holding': [0.254, 0.358, 0.254, 0.358]
        }
        
        # Initialize the current positions and set initial target positions
        self.current_positions = [0, 0, 0, 0]
        self.target_positions = self.poses['non_holding']  # Default initial pose
        self.step = 0.02

    def timer_callback(self):

        global reached

        # Update positions slowly
        for i in range(len(self.current_positions)):
            if self.current_positions[i] < self.target_positions[i]:
                self.current_positions[i] += self.step
                if self.current_positions[i] > self.target_positions[i]:
                    self.current_positions[i] = self.target_positions[i]
            elif self.current_positions[i] > self.target_positions[i]:
                self.current_positions[i] -= self.step
                if self.current_positions[i] < self.target_positions[i]:
                    self.current_positions[i] = self.target_positions[i]

        msg = Float64MultiArray()
        msg.data = [self.current_positions[0]]
        self.publisher_1.publish(msg)
        # self.get_logger().info(f'Publishing: {msg.data}')

        msg = Float64MultiArray()
        msg.data = [self.current_positions[1]]
        self.publisher_2.publish(msg)
        # self.get_logger().info(f'Publishing: {msg.data}')

        msg = Float64MultiArray()
        msg.data = [self.current_positions[2]]
        # self.publisher_3.publish(msg)

        msg = Float64MultiArray()
        msg.data = [self.current_positions[3]]
        # self.publisher_4.publish(msg)
        
        # Check if all joints have reached their target positions within tolerance
        if all(abs(cp - tp) < self.step/4 for cp, tp in zip(self.current_positions, self.target_positions)):
            self.get_logger().info('Reached target positions')
            reached = True
            # self.timer.cancel()  # Stop the timer
            # self.destroy_node()  # Destroy the node to exit

# Suppress ALSA warnings
os.environ["PYTHONWARNINGS"] = "ignore"
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"
os.environ["NUMBA_WARNINGS"] = "ignore"
os.environ["NUMBA_LOG_LEVEL"] = "0"

# Load the Whisper model and processor from local files
processor_whisper = WhisperProcessor.from_pretrained("openai/whisper-small")
model_whisper = WhisperForConditionalGeneration.from_pretrained("openai/whisper-small")
model_whisper.config.forced_decoder_ids = None

# Path to the local directory containing the model files
local_model_path = '/media/yasiru/Transcend/AI_project_files/huggingface/hub/models--TheBloke--Mistral-7B-Instruct-v0.1-GGUF/snapshots/731a9fc8f06f5f5e2db8a0cf9d256197eb6e05d1/'


# Initialize the model
llm = AutoModelForCausalLM.from_pretrained(
    local_model_path, 
    model_file="mistral-7b-instruct-v0.1.Q4_K_S.gguf", 
    model_type="mistral", 
    gpu_layers=24
)

# Path to the local directory containing the model files
local_model_path = '/media/yasiru/Transcend/AI_project_files/huggingface/hub/models--Salesforce--blip-image-captioning-base/snapshots/89b09ea1789f7addf2f6d6f0dfc4ce10ab58ef84'

# Load the processor and model from the local directory
processor = BlipProcessor.from_pretrained(local_model_path)
model = BlipForConditionalGeneration.from_pretrained(local_model_path, torch_dtype=torch.float16).to("cuda")


# Function to generate a response from the model
def generate_response(system_prompt, user_prompt):
    # Combine the system and user prompts into a single input
    input_text = f"System: {system_prompt}\nUser: {user_prompt}\nAI:"
    # Generate response
    response = llm(input_text)
    return response

def process_image(image_path, text=None):
    # Load the image
    try:
        raw_image = Image.open(image_path).convert('RGB')
    except Exception as e:
        print(f"Error loading image: {e}")
        return

    # Prepare inputs for the model
    if text:
        inputs = processor(raw_image, text, return_tensors="pt").to("cuda", torch.float16)
    else:
        inputs = processor(raw_image, return_tensors="pt").to("cuda", torch.float16)

    # Generate caption
    out = model.generate(**inputs, max_new_tokens=50)
    caption = processor.decode(out[0], skip_special_tokens=True)
    print(f"Image Feed: {caption}")
    return caption

# Function to transcribe audio
def transcribe_audio(audio_array, sampling_rate):
    input_features = processor_whisper(audio_array, sampling_rate=sampling_rate, return_tensors="pt").input_features
    # Set the language to English
    forced_decoder_ids = processor_whisper.tokenizer.get_decoder_prompt_ids(language="en", task="transcribe")
    generated_tokens = model_whisper.generate(input_features, forced_decoder_ids=forced_decoder_ids)
    transcription = processor_whisper.batch_decode(generated_tokens, skip_special_tokens=True)
    return transcription

# Define the file path
file_path = 'system_prompt.txt'

# Load system_prompt from the text file
with open(file_path, 'r') as file:
    system_prompt = file.read()

def main(args=None):

    global reached
    global goal_achieved

    rclpy.init(args=args)
    node = JointStatePublisher()
    goal_sender = Nav2GoalSender()

    chunk = 1024  # Record in chunks of 1024 samples
    sample_format = pyaudio.paFloat32  # 32 bits per sample
    channels = 1
    fs = 16000  # Record at 16kHz

    p = pyaudio.PyAudio()  # Create an interface to PortAudio

    stream = p.open(format=sample_format,
                    channels=channels,
                    rate=fs,
                    frames_per_buffer=chunk,
                    input=True)

    print("Listening...")

    buffer = []
    buffer_duration = 0  # Duration of audio in buffer
    listening = True

    # Main loop to interact with the user
    while True:

        while listening:
            data = stream.read(chunk)
            audio_array = np.frombuffer(data, dtype=np.float32)
            buffer.append(audio_array)
            buffer_duration += len(audio_array) / fs

            # Process audio data if buffer duration exceeds threshold (e.g., 5 seconds)
            if buffer_duration >= 5:
                combined_audio = np.concatenate(buffer, axis=0)
                transcription = transcribe_audio(combined_audio, sampling_rate=fs)
                print("Transcription:", transcription)

                if len(transcription[0]) > 15:
                    break

                # Clear buffer
                buffer = []
                buffer_duration = 1

        image_path = "input.jpg"
        text = ""

        caption = process_image(image_path, text)

        reached = False
        user_prompt = transcription[0] #input("User: ")
        transcription = [""]
        buffer = []
        if user_prompt.lower() in ["exit", "quit", "stop"]:
            print("Exiting the chat. Goodbye!")
            break

        if caption != None:
            user_prompt = "image_description: " + caption + " user_input: " + user_prompt
        else:
            user_prompt = "image_description: " +  "None"  + "user_input: " + user_prompt
        response = generate_response(system_prompt, user_prompt)
        print(f"AI: {response}")
        
        if not "none" in response.lower():

            location = response.split("location: ")[1].split(")")[0].split("(")[1].split(",")

            x = float(location[0])
            y = float(location[1])
            theta = float(location[2])

            goal_sender.send_goal(x, y, theta)

            while not goal_achieved:

                rclpy.spin_once(goal_sender)

        try:

            while True:
                # Prompt user for input
                print("Available poses: holding, idle, non_holding")
                # pose_input = input("Enter desired pose ('q' to quit): ")
                if "non_holding" in response:
                    pose_input = "non_holding"
                elif "idle" in response:
                    pose_input = "idle"
                elif "holding" in response:
                    pose_input = "holding"
                else:
                    pose_input = "None"


                if pose_input.lower() == 'q':
                    break  # Exit loop and shutdown

                # print(list(node.poses.keys()))

                if pose_input in list(node.poses.keys()):
                    node.target_positions = node.poses[pose_input]
                    while not reached:
                        rclpy.spin_once(node)
                    if reached:
                        reached = False
                        break
                else:
                    print("Invalid pose. Please enter one of: holding, idle, non_holding")
        
        finally:
            # node.destroy_node()
            # rclpy.shutdown()
            pass

if __name__ == '__main__':
    main()


