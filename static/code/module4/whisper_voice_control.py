#!/usr/bin/env python3
"""
Whisper Voice Control for ROS 2 Robots
Real-time speech recognition for natural language commands
"""

import whisper
import pyaudio
import numpy as np
import threading
import queue


class WhisperVoiceController:
    """
    Real-time voice control using OpenAI Whisper
    Captures audio, transcribes commands, and provides callback interface
    """

    def __init__(self, model_size="small", language="en"):
        """
        Initialize Whisper voice controller

        Args:
            model_size: Whisper model size (tiny, base, small, medium, large)
            language: Language code (en, es, fr, etc.) or None for auto-detect
        """
        print(f"Loading Whisper {model_size} model...")
        self.model = whisper.load_model(model_size)
        self.language = language

        # Audio settings (optimized for Whisper)
        self.RATE = 16000  # 16kHz sample rate
        self.CHANNELS = 1  # Mono
        self.CHUNK = 1024  # Frames per buffer
        self.FORMAT = pyaudio.paInt16

        # PyAudio instance
        self.audio = pyaudio.PyAudio()

        # Command queue for async processing
        self.command_queue = queue.Queue()

        # Control flags
        self.is_listening = False
        self.listen_thread = None

    def start_listening(self, callback, listen_duration=3.0):
        """
        Start continuous listening mode

        Args:
            callback: Function to call with transcribed text: callback(text: str)
            listen_duration: Duration to record for each transcription (seconds)
        """
        self.is_listening = True

        def listen_loop():
            print("Voice control active. Speak commands...")
            while self.is_listening:
                transcript = self.capture_and_transcribe(listen_duration)

                if transcript.strip():  # Non-empty transcript
                    print(f"Heard: \"{transcript}\"")
                    callback(transcript)

                    # Check for stop command
                    if "stop listening" in transcript.lower():
                        self.stop_listening()
                        break

        self.listen_thread = threading.Thread(target=listen_loop, daemon=True)
        self.listen_thread.start()

    def stop_listening(self):
        """Stop continuous listening"""
        self.is_listening = False
        if self.listen_thread:
            self.listen_thread.join(timeout=5.0)
        print("Voice control stopped")

    def capture_and_transcribe(self, duration=3.0):
        """
        Capture audio for duration seconds and transcribe

        Args:
            duration: Recording duration in seconds

        Returns:
            Transcribed text
        """
        stream = self.audio.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK
        )

        # Record audio
        frames = []
        num_chunks = int(self.RATE / self.CHUNK * duration)

        for _ in range(num_chunks):
            try:
                data = stream.read(self.CHUNK, exception_on_overflow=False)
                frames.append(data)
            except Exception as e:
                print(f"Audio read error: {e}")
                break

        stream.stop_stream()
        stream.close()

        # Convert to numpy array
        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
        audio_data = audio_data.astype(np.float32) / 32768.0  # Normalize to [-1, 1]

        # Transcribe with Whisper
        result = self.model.transcribe(
            audio_data,
            language=self.language,
            fp16=False  # Disable FP16 for CPU compatibility
        )

        return result["text"].strip()

    def transcribe_file(self, audio_file_path):
        """
        Transcribe audio from file

        Args:
            audio_file_path: Path to audio file (mp3, wav, etc.)

        Returns:
            Transcribed text
        """
        result = self.model.transcribe(audio_file_path, language=self.language)
        return result["text"]

    def get_available_microphones(self):
        """List available audio input devices"""
        print("Available microphones:")
        for i in range(self.audio.get_device_count()):
            info = self.audio.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                print(f"  [{i}] {info['name']}")

    def cleanup(self):
        """Clean up resources"""
        self.stop_listening()
        self.audio.terminate()


# Example usage for standalone testing
if __name__ == "__main__":
    def handle_command(text):
        """Callback function for voice commands"""
        print(f"Command received: {text}")

        # Example: Parse simple commands
        text_lower = text.lower()

        if "navigate" in text_lower or "go to" in text_lower:
            print("  → Navigation command detected")
        elif "pick" in text_lower or "grab" in text_lower:
            print("  → Manipulation command detected")
        elif "stop" in text_lower:
            print("  → Stop command detected")

    # Create voice controller
    controller = WhisperVoiceController(model_size="small", language="en")

    # List available microphones
    controller.get_available_microphones()

    # Start listening
    try:
        controller.start_listening(callback=handle_command, listen_duration=3.0)

        # Keep main thread alive
        while controller.is_listening:
            import time
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        controller.cleanup()
