#!/usr/bin/env python3
"""
GeminiClass - Wrapper for Google's Gemini generative AI model.

This module provides a simple interface for using Gemini for text generation
and image-based content generation.

Requires:
    - google-generativeai package: pip install google-generativeai
    - GOOGLE_API_KEY environment variable or explicit API key configuration
"""

import os
import google.generativeai as genai


class GeminiClass:
    def __init__(self, prompt=None, api_key=None, json_key_path=None):
        """
        Constructor for the GeminiClass.
        Initializes the generative model and sets a default prompt.

        Parameters:
        - prompt (optional): A string to set as the default prompt. Defaults to an empty string if not provided.
        - api_key (optional): Google API key. If not provided, will look for GOOGLE_API_KEY env var.
        - json_key_path (optional): Path to a JSON service account key file.
        """
        # Configure API key
        if api_key:
            genai.configure(api_key=api_key)
        elif json_key_path:
            # For service account authentication
            os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = json_key_path
        elif 'GOOGLE_API_KEY' in os.environ:
            genai.configure(api_key=os.environ['GOOGLE_API_KEY'])
        else:
            raise ValueError(
                "No API key provided. Either pass api_key, json_key_path, "
                "or set GOOGLE_API_KEY environment variable."
            )

        self.gemini_model = genai.GenerativeModel("gemini-2.5-flash")
        if prompt:
            self.prompt = prompt
        else:
            self.prompt = ""

    def set_prompt(self, prompt):
        """
        Sets or updates the prompt for the GeminiClass.

        Parameters:
        - prompt: A string that will serve as the base prompt for content generation.
        """
        self.prompt = prompt

    def generate_content(self, text, use_prompt=True):
        """
        Generates content using the generative model.

        Parameters:
        - text: A string input to provide to the model for content generation.
        - use_prompt: A boolean indicating whether to prepend the class's prompt to the input text.

        Returns:
        - The generated content as a string.
        """
        if use_prompt:
            final_input = f"{self.prompt} {text}"
        else:
            final_input = text

        response = self.gemini_model.generate_content(final_input)

        return response.text

    def generate_from_image(self, image_bytes, text_input):
        """
        Generates content based on an image and optional text input.

        Parameters:
        - image_bytes: The binary data of the image.
        - text_input: A string input to accompany the image for content generation.

        Returns:
        - The generated content as a string.
        """
        if not isinstance(image_bytes, bytes):
            response = self.gemini_model.generate_content([text_input, image_bytes])
        else:
            image_part = {
                "mime_type": "image/jpeg",
                "data": image_bytes
            }
            response = self.gemini_model.generate_content([text_input, image_part])

        return response.text
