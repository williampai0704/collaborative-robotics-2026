#!/usr/bin/env python3
"""
Speech Extraction Node

ROS2 node that records from the robot microphone (or reads an m4a file),
transcribes with Google Cloud Speech-to-Text, extracts object/bin colors
using Gemini, and publishes the result to /speech_extraction.

─── Modes ─────────────────────────────────────────────────────────────────────
Mic mode (default):
    Records from the robot microphone via /microphone/record service.
    Requires the microphone_node to be running (launched by real.launch.py).

File mode:
    Reads a pre-recorded m4a file instead of live mic input.

─── Prerequisites ──────────────────────────────────────────────────────────────
1. Google credentials (choose one):
       export GOOGLE_API_KEY="your-api-key"
       export GOOGLE_APPLICATION_CREDENTIALS="/path/to/service-account.json"
   OR place key at:
       ros2_ws/src/tidybot_bringup/config/credentials/google_cloud_key.json

2. Real robot launch (for mic mode):
       ros2 launch tidybot_bringup real.launch.py

─── Usage ──────────────────────────────────────────────────────────────────────
Mic mode (default, 5-second recording):
    ros2 run tidybot_bringup speech_extraction_node.py

Mic mode with custom duration:
    ros2 run tidybot_bringup speech_extraction_node.py \
        --ros-args -p record_duration:=7.0

File mode:
    ros2 run tidybot_bringup speech_extraction_node.py \
        --ros-args -p use_mic:=false -p audio_file_path:=/path/to/audio.m4a
"""

import os
import io
import re
import json
import wave
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ament_index_python.packages import get_package_share_directory

from google.cloud import speech
from pydub import AudioSegment

from tidybot_msgs.msg import SpeechExtraction
from tidybot_msgs.srv import AudioRecord
from std_msgs.msg import Header

# Import local gemini module
import sys
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


class SpeechExtractionNode(Node):
    """ROS2 node for speech-based command extraction."""

    # Prompt for Gemini to extract structured information
    EXTRACTION_PROMPT = (
        "Given the sentence of instruction, extract the information as a JSON object "
        "with keys 'object_color' and 'bin_color'. Only generate this JSON object, "
        "do not generate any other text. If any of the information is unable to extract "
        "from the given text, set the value to 'unknown'. "
        "Here is the following instruction: "
    )

    def __init__(self):
        super().__init__('speech_extraction_node')

        # Declare parameters
        self.declare_parameter('use_mic', True)
        self.declare_parameter('record_duration', 5.0)
        self.declare_parameter('audio_file_path', '')
        self.declare_parameter('google_api_key', '')
        self.declare_parameter('json_key_path', '')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('extraction_prompt', '')  # overrides EXTRACTION_PROMPT if set

        # Get parameters
        self.use_mic = self.get_parameter('use_mic').get_parameter_value().bool_value
        self.record_duration = self.get_parameter('record_duration').get_parameter_value().double_value
        self.audio_file_path = self.get_parameter('audio_file_path').get_parameter_value().string_value
        google_api_key = self.get_parameter('google_api_key').get_parameter_value().string_value
        json_key_path = self.get_parameter('json_key_path').get_parameter_value().string_value
        custom_prompt = self.get_parameter('extraction_prompt').get_parameter_value().string_value
        active_prompt = custom_prompt if custom_prompt else self.EXTRACTION_PROMPT

        # Try to find credentials in config folder if not provided
        if not json_key_path and not google_api_key:
            json_key_path = self._find_default_credentials()

        # Set up Google Cloud credentials for Speech-to-Text
        if json_key_path and os.path.exists(json_key_path):
            os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = json_key_path
            self.get_logger().info(f'Using credentials from: {json_key_path}')

        # Publisher for extraction results
        self.extraction_pub = self.create_publisher(
            SpeechExtraction,
            '/speech_extraction',
            10
        )

        # Initialize transcriber and Gemini
        try:
            self.transcriber = SpeechTranscriber()
            self.get_logger().info('Speech transcriber initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize transcriber: {e}')
            self.transcriber = None

        try:
            self.gemini = GeminiClass(
                prompt=active_prompt,
                api_key=google_api_key if google_api_key else None,
                json_key_path=json_key_path if json_key_path else None
            )
            self.get_logger().info('Gemini initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Gemini: {e}')
            self.gemini = None

        # Event-based sync helpers for async service calls from background thread
        self._mic_result = None
        self._mic_event = threading.Event()

        if self.use_mic:
            # Mic client — give the service 2 s to come up before recording
            self._mic_cb_group = MutuallyExclusiveCallbackGroup()
            self.mic_client = self.create_client(
                AudioRecord, '/microphone/record',
                callback_group=self._mic_cb_group
            )
            self.get_logger().info(
                f'Mic mode: will record {self.record_duration:.1f}s from /microphone/record')
            self._mic_startup_timer = self.create_timer(2.0, self._trigger_mic_recording)
        elif self.audio_file_path:
            self.get_logger().info(f'File mode: processing {self.audio_file_path}')
            self.process_audio_file(self.audio_file_path)
        else:
            self.get_logger().warn('No mic and no audio_file_path — nothing to process.')

        self.get_logger().info('=' * 50)
        self.get_logger().info('Speech Extraction Node Ready')
        self.get_logger().info('=' * 50)

    def _trigger_mic_recording(self):
        """Called once by a 2-second startup timer; runs mic I/O in a background thread."""
        self._mic_startup_timer.cancel()
        threading.Thread(target=self._mic_record_and_process, daemon=True).start()

    def _call_mic_service(self, start: bool):
        """
        Call /microphone/record from a background thread.
        Uses call_async + threading.Event so the main executor keeps spinning.
        """
        self._mic_event.clear()
        self._mic_result = None

        if not self.mic_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/microphone/record service not available')
            return None

        req = AudioRecord.Request()
        req.start = start
        future = self.mic_client.call_async(req)
        future.add_done_callback(self._mic_done_cb)
        self._mic_event.wait(timeout=30.0)
        return self._mic_result

    def _mic_done_cb(self, future):
        self._mic_result = future.result()
        self._mic_event.set()

    @staticmethod
    def _float32_to_wav(audio_data, sample_rate: int) -> bytes:
        """Convert PCM float32 samples to 16-bit WAV bytes."""
        audio = np.clip(np.array(audio_data, dtype=np.float32), -1.0, 1.0)
        int16 = (audio * 32767).astype(np.int16)
        buf = io.BytesIO()
        with wave.open(buf, 'w') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(sample_rate)
            wf.writeframes(int16.tobytes())
        buf.seek(0)
        return buf.read()

    def _mic_record_and_process(self):
        """Background thread: start mic → sleep → stop mic → transcribe → publish."""
        log = self.get_logger()

        log.info('Starting microphone recording...')
        resp_start = self._call_mic_service(start=True)
        if resp_start is None or not resp_start.success:
            err = resp_start.message if resp_start else 'service call failed'
            log.error(f'Mic start failed: {err}')
            return

        log.info(f'Recording for {self.record_duration:.1f}s — speak now...')
        threading.Event().wait(timeout=self.record_duration)   # interruptible sleep

        log.info('Stopping microphone recording...')
        resp_stop = self._call_mic_service(start=False)
        if resp_stop is None or not resp_stop.success:
            err = resp_stop.message if resp_stop else 'service call failed'
            log.error(f'Mic stop failed: {err}')
            return

        if not resp_stop.audio_data:
            log.error('Mic returned no audio data')
            return

        log.info(f'Got {len(resp_stop.audio_data)} samples @ {resp_stop.sample_rate} Hz '
                 f'({resp_stop.duration:.2f}s) — transcribing...')
        wav_bytes = self._float32_to_wav(resp_stop.audio_data, resp_stop.sample_rate)
        self._transcribe_and_publish(wav_bytes)

    def _find_default_credentials(self):
        """Try to find credentials in default locations."""
        # Check in package config directory
        try:
            pkg_share = get_package_share_directory('tidybot_bringup')
            default_path = os.path.join(pkg_share, 'config', 'credentials', 'google_cloud_key.json')
            if os.path.exists(default_path):
                return default_path
        except Exception:
            pass

        # Check in source config directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        src_config_path = os.path.join(
            script_dir, '..', '..', 'config', 'credentials', 'google_cloud_key.json'
        )
        if os.path.exists(src_config_path):
            return os.path.abspath(src_config_path)

        return ''

    def convert_m4a_to_wav(self, audio_path):
        """
        Convert m4a audio file to WAV format suitable for Google Speech-to-Text.

        :param audio_path: Path to the m4a audio file.
        :return: WAV audio content as bytes.
        """
        audio_segment = AudioSegment.from_file(audio_path, format="m4a")

        # Export as WAV (LINEAR16, 16kHz) to an in-memory buffer
        wav_buffer = io.BytesIO()
        audio_segment.export(
            wav_buffer,
            format="wav",
            parameters=["-acodec", "pcm_s16le", "-ar", "16000"]
        )
        wav_buffer.seek(0)

        return wav_buffer.read()

    def parse_gemini_response(self, response_text):
        """
        Parse Gemini's response to extract object_color and bin_color.

        :param response_text: Raw response from Gemini.
        :return: Tuple of (object_color, bin_color, success).
        """
        try:
            # Try to find JSON in the response
            json_match = re.search(r'\{[^}]+\}', response_text, re.DOTALL)
            if json_match:
                data = json.loads(json_match.group())
                object_color = data.get('object_color', 'unknown')
                bin_color = data.get('bin_color', 'unknown')
                return object_color, bin_color, True

            # Fallback: try to parse the old format [object_color:{}, bin_color:{}]
            object_match = re.search(r'object_color[:\s]*["\']?(\w+)["\']?', response_text, re.IGNORECASE)
            bin_match = re.search(r'bin_color[:\s]*["\']?(\w+)["\']?', response_text, re.IGNORECASE)

            object_color = object_match.group(1) if object_match else 'unknown'
            bin_color = bin_match.group(1) if bin_match else 'unknown'

            return object_color, bin_color, True

        except Exception as e:
            self.get_logger().warn(f'Failed to parse Gemini response: {e}')
            return 'unknown', 'unknown', False

    def process_audio_file(self, audio_path):
        """Process an m4a file: convert to WAV then transcribe and publish."""
        if not os.path.exists(audio_path):
            self.get_logger().error(f'Audio file not found: {audio_path}')
            return
        try:
            self.get_logger().info('Converting audio file...')
            wav_content = self.convert_m4a_to_wav(audio_path)
            self._transcribe_and_publish(wav_content)
        except Exception as e:
            self.get_logger().error(f'Error processing audio file: {e}')

    def _transcribe_and_publish(self, wav_bytes: bytes):
        """Transcribe WAV bytes with Google STT + Gemini, then publish result."""
        msg = SpeechExtraction()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'speech'

        if self.transcriber is None:
            msg.success = False
            msg.error_message = 'Transcriber not initialized'
            self.extraction_pub.publish(msg)
            self.get_logger().error(msg.error_message)
            return

        if self.gemini is None:
            msg.success = False
            msg.error_message = 'Gemini not initialized'
            self.extraction_pub.publish(msg)
            self.get_logger().error(msg.error_message)
            return

        try:
            self.get_logger().info('Transcribing audio...')
            transcription = self.transcriber.transcribe_audio(wav_bytes)
            msg.transcription = transcription
            self.get_logger().info(f'Transcription: {transcription}')

            if not transcription:
                msg.success = False
                msg.error_message = 'Transcription returned empty result'
                self.extraction_pub.publish(msg)
                self.get_logger().warn(msg.error_message)
                return

            self.get_logger().info('Extracting information with Gemini...')
            gemini_response = self.gemini.generate_content(transcription, use_prompt=True)
            self.get_logger().info(f'Gemini response: {gemini_response}')

            object_color, bin_color, success = self.parse_gemini_response(gemini_response)
            msg.object_color = object_color
            msg.bin_color = bin_color
            msg.success = success

            if success:
                self.get_logger().info(
                    f'Extracted: object_color={object_color}, bin_color={bin_color}')
            else:
                msg.error_message = 'Failed to parse Gemini response'

            self.last_extraction_msg = msg
            self.extraction_pub.publish(msg)
            self.publish_count = 0
            self.publish_timer = self.create_timer(0.1, self._publish_result_callback)

        except Exception as e:
            msg.success = False
            msg.error_message = str(e)
            self.extraction_pub.publish(msg)
            self.get_logger().error(f'Error during transcription/extraction: {e}')

    def _publish_result_callback(self):
        """Publish the extraction result multiple times to ensure delivery."""
        if hasattr(self, 'last_extraction_msg') and self.last_extraction_msg is not None:
            self.extraction_pub.publish(self.last_extraction_msg)
            self.publish_count += 1
            if self.publish_count >= 10:  # Publish 10 times over 1 second
                self.publish_timer.cancel()
                self.get_logger().info('Finished publishing extraction result')


def main(args=None):
    rclpy.init(args=args)
    node = SpeechExtractionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
