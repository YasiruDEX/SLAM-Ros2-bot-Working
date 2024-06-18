from ctransformers import AutoModelForCausalLM

# Path to the local directory containing the model files
local_model_path = '/media/yasiru/Transcend/AI_project_files/huggingface/hub/models--TheBloke--Mistral-7B-Instruct-v0.1-GGUF/snapshots/731a9fc8f06f5f5e2db8a0cf9d256197eb6e05d1/'


# Initialize the model
llm = AutoModelForCausalLM.from_pretrained(
    local_model_path, 
    model_file="mistral-7b-instruct-v0.1.Q4_K_S.gguf", 
    model_type="mistral", 
    gpu_layers=50
)

# Function to generate a response from the model
def generate_response(system_prompt, user_prompt):
    # Combine the system and user prompts into a single input
    input_text = f"System: {system_prompt}\nUser: {user_prompt}\nAI:"
    # Generate response
    response = llm(input_text)
    return response

# Define the system prompt
system_prompt = """You are an instruction generator. Depending on the user input, you have to generate one of three commands: "holding", "non_holding", or "idle" and a location from the prestored database below.

                    kitchen:(2,5), living_room:(3,4), bedroom:(4,3), bathroom:(5,2), garage:(6,1), garden:(7,0)

                        Input: "Go to the kitchen. Hold the item securely."
                        Output: "arm_command: holding, location: (2,5)"

                        Input: "You may now release your grasp. Return to the living room."
                        Output: "arm_command: non_holding, location: (3,4)"

                        Input: "bedroom is the place you need to be.Please stay in standby mode."
                        Output: "arm_command: idle, location: (4,3)"

                        Input: "Maintain your position until further notice."
                        Output: "arm_command: idle, location: None"

                        Input: "You are free to disengage the mechanism."
                        Output: "arm_command: non_holding, location: None"

                        Input: "Take a rest and await further instructions."
                        Output: "arm_command: idle, location: None"

                    This formatting keeps the examples clear and consistent, using "arm_command" followed by the action type ("holding", "non_holding", or "idle") and optionally the location if applicable.

                    ---------end of the system messege---------
                """

# Main loop to interact with the user
while True:
    user_prompt = input("User: ")
    if user_prompt.lower() in ["exit", "quit", "stop"]:
        print("Exiting the chat. Goodbye!")
        break
    response = generate_response(system_prompt, user_prompt)
    print(f"AI: {response}")

