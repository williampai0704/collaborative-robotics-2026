# Credentials Directory

This directory is for storing API credentials. **DO NOT commit actual credentials to the repository.**

## Google Cloud Credentials

Place your Google Cloud service account JSON key file here:

```
ros2_ws/src/tidybot_bringup/config/credentials/google_cloud_key.json
```

### How to obtain credentials:

1. Go to the [Google Cloud Console](https://console.cloud.google.com/)
2. Create a project or select an existing one
3. Enable the following APIs:
   - Cloud Speech-to-Text API
   - Generative Language API (for Gemini)
4. Go to "IAM & Admin" > "Service Accounts"
5. Create a service account with appropriate permissions
6. Generate a JSON key and download it
7. Place the JSON file in this directory as `google_cloud_key.json`

### Alternative: Using API Key

For Gemini-only usage, you can use an API key instead:

1. Go to [Google AI Studio](https://aistudio.google.com/)
2. Get an API key
3. Set the environment variable:
   ```bash
   export GOOGLE_API_KEY="your-api-key"
   ```

### Security Notes

- The `.gitignore` file should already exclude `*.json` files in this directory
- Never commit credentials to version control
- Use environment variables for production deployments
