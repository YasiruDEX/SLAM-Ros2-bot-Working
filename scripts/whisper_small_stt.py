import os
import pyaudio
import numpy as np
from transformers import WhisperProcessor, WhisperForConditionalGeneration
import torch

# Suppress ALSA warnings
os.environ["PYTHONWARNINGS"] = "ignore"
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"
os.environ["NUMBA_WARNINGS"] = "ignore"
os.environ["NUMBA_LOG_LEVEL"] = "0"

# Load the Whisper model and processor from local files
processor = WhisperProcessor.from_pretrained("openai/whisper-small")
model = WhisperForConditionalGeneration.from_pretrained("openai/whisper-small")

# Move the model to GPU if available
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model.to(device)
model.config.forced_decoder_ids = None

# Function to transcribe audio
def transcribe_audio(audio_array, sampling_rate):
    input_features = processor(audio_array, sampling_rate=sampling_rate, return_tensors="pt").input_features.to(device)
    # Set the language to English
    forced_decoder_ids = processor.tokenizer.get_decoder_prompt_ids(language="en", task="transcribe")
    generated_tokens = model.generate(input_features, forced_decoder_ids=forced_decoder_ids)
    transcription = processor.batch_decode(generated_tokens, skip_special_tokens=True)
    return transcription

# Function to continuously listen to the microphone
def listen_microphone():
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

    try:
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

                # Clear buffer
                buffer = []
                buffer_duration = 1

    except KeyboardInterrupt:
        print("Stopped listening.")
    finally:
        # Stop and close the stream
        stream.stop_stream()
        stream.close()
        p.terminate()

if __name__ == "__main__":
    listen_microphone()
