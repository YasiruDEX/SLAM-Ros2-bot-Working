import torch
from PIL import Image
from transformers import BlipProcessor, BlipForConditionalGeneration

# Path to the local directory containing the model files
local_model_path = '/media/yasiru/Transcend/AI_project_files/huggingface/hub/models--Salesforce--blip-image-captioning-base/snapshots/89b09ea1789f7addf2f6d6f0dfc4ce10ab58ef84'

# Load the processor and model from the local directory
processor = BlipProcessor.from_pretrained(local_model_path)
model = BlipForConditionalGeneration.from_pretrained(local_model_path, torch_dtype=torch.float16).to("cuda")

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
    print(f"Caption: {caption}")

while True:
    # Ask for input file name
    image_path = input("Enter the path to the image file (or 'exit' to quit): ")
    if image_path.lower() == 'exit':
        break

    # Ask for optional text prompt
    text = None #"a photography of" #input("Enter the optional text prompt (or leave empty for unconditional captioning): ")
    # text = text if text.strip() != "" else None

    # Process the image and generate caption
    process_image(image_path, text)
