import pyaudio
import numpy as np
import soundfile as sf
from transformers import WhisperProcessor, WhisperForConditionalGeneration

# Load the Whisper model and processor
processor = WhisperProcessor.from_pretrained("openai/whisper-small")
model = WhisperForConditionalGeneration.from_pretrained("openai/whisper-small")
model.config.forced_decoder_ids = None


# Function to transcribe audio
def transcribe_audio(audio_array, sampling_rate):
    input_features = processor(audio_array, sampling_rate=sampling_rate, return_tensors="pt").input_features
    predicted_ids = model.generate(input_features)
    transcription = processor.batch_decode(predicted_ids, skip_special_tokens=True)
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

    frames = []
    listening = True
    while listening:
        try:
            data = stream.read(chunk)
            frames.append(data)
            audio_array = np.frombuffer(data, dtype=np.float32)
            
            # Example for continuous transcription
            if len(audio_array) > 0:
                # Process audio data and transcribe
                transcription = transcribe_audio(audio_array, sampling_rate=fs)
                print("Transcription:", transcription)
            
        except KeyboardInterrupt:
            print("Stopped listening.")
            listening = False
            break

    # Stop and close the stream
    stream.stop_stream()
    stream.close()
    p.terminate()

if __name__ == "__main__":
    listen_microphone()
