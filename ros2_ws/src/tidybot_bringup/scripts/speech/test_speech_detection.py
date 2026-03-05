#!/usr/bin/env python3
"""
Test script for Speech Extraction functionality.

This script demonstrates how to use the SpeechTranscriber and GeminiClass
to transcribe audio and extract object/bin color information.

For ROS2 usage, use speech_extraction_node.py instead.

Usage (standalone):
    # Set credentials first
    export GOOGLE_API_KEY="your-api-key"
    export GOOGLE_APPLICATION_CREDENTIALS="/path/to/service-account.json"

    # Run test
    python3 test_speech_detection.py /path/to/audio.m4a
"""

import os
import sys
import io
from pydub import AudioSegment
from google.cloud import speech

# Add script directory to path for local imports
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from gemini import GeminiClass


class SpeechTranscriber:
    """Google Cloud Speech-to-Text transcriber."""

    def __init__(self, language_code='en-US', sample_rate=16000):
        """
        Initialize a SpeechTranscriber instance.

        :param language_code: The language code for transcription, e.g., 'en-US'.
        :param sample_rate: The sample rate (Hertz) of the audio file, default is 16000.
        """
        self.language_code = language_code
        self.sample_rate = sample_rate
        self.client = speech.SpeechClient()

    def transcribe_audio(self, audio_content):
        """
        Uses Google Cloud Speech-to-Text to transcribe the given audio content (bytes).
        Returns the transcription as a string.

        :param audio_content: The raw bytes of the audio file to be transcribed.
        :return: A string of the combined transcription.
        """
        audio = speech.RecognitionAudio(content=audio_content)

        config = speech.RecognitionConfig(
            sample_rate_hertz=self.sample_rate,
            language_code=self.language_code,
            enable_automatic_punctuation=True,
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        )

        response = self.client.recognize(config=config, audio=audio)

        if response.results:
            transcript = response.results[0].alternatives[0].transcript
            return transcript
        return ""


def convert_m4a_to_wav(audio_path):
    """Convert m4a file to WAV format for Speech-to-Text."""
    audio_segment = AudioSegment.from_file(audio_path, format="m4a")

    wav_buffer = io.BytesIO()
    audio_segment.export(
        wav_buffer,
        format="wav",
        parameters=["-acodec", "pcm_s16le", "-ar", "16000"]
    )
    wav_buffer.seek(0)

    return wav_buffer.read()


def process_audio_file(audio_path, transcriber, gemini):
    """Process a single audio file and extract information."""
    print(f"\nProcessing: {audio_path}")
    print("-" * 50)

    # Convert and transcribe
    wav_content = convert_m4a_to_wav(audio_path)
    transcription = transcriber.transcribe_audio(wav_content)
    print(f"Transcription: {transcription}")

    # Extract information with Gemini
    extraction = gemini.generate_content(transcription, use_prompt=True)
    print(f"Extraction: {extraction}")

    return transcription, extraction


def main():
    # Check for audio file argument
    if len(sys.argv) < 2:
        print("Usage: python3 test_speech_detection.py <audio_file.m4a> [audio_file2.m4a ...]")
        print("\nOr test with sample data path:")
        print("  python3 test_speech_detection.py /path/to/Speech_Data/*.m4a")
        sys.exit(1)

    audio_files = sys.argv[1:]

    # Initialize transcriber
    print("Initializing Speech Transcriber...")
    transcriber = SpeechTranscriber()

    # Initialize Gemini with extraction prompt
    print("Initializing Gemini...")
    gemini = GeminiClass()
    gemini.set_prompt(
        "Given the sentence of instruction, extract the information as a JSON object "
        "with keys 'object_color' and 'bin_color'. Only generate this JSON object, "
        "do not generate any other text. If any of the information is unable to extract "
        "from the given text, set the value to 'unknown'. "
        "Here is the following instruction: "
    )

    # Process each audio file
    results = []
    for audio_path in audio_files:
        if not os.path.exists(audio_path):
            print(f"Warning: File not found: {audio_path}")
            continue

        try:
            transcription, extraction = process_audio_file(audio_path, transcriber, gemini)
            results.append({
                'file': audio_path,
                'transcription': transcription,
                'extraction': extraction
            })
        except Exception as e:
            print(f"Error processing {audio_path}: {e}")

    # Summary
    print("\n" + "=" * 50)
    print("SUMMARY")
    print("=" * 50)
    for result in results:
        print(f"\nFile: {os.path.basename(result['file'])}")
        print(f"  Transcription: {result['transcription']}")
        print(f"  Extraction: {result['extraction']}")


if __name__ == '__main__':
    main()
