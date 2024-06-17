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
system_prompt = """You are an instruction generator. Depending on the user input, you have to generate one of three commands: "holding", "non_holding", or "idle".

                    Example:
                    Input: "Secure the object tightly."
                    Output: "holding"

                    Example:
                    Input: "You can release your grip now."
                    Output: "non_holding"

                    Example:
                    Input: "Remain in standby mode."
                    Output: "idle"

                    Example:
                    Input: "Hold the position until further notice."
                    Output: "holding"

                    Example:
                    Input: "You may disengage the mechanism."
                    Output: "non_holding"

                    Example:
                    Input: "Take a break and wait for further instructions."
                    Output: "idle"

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

