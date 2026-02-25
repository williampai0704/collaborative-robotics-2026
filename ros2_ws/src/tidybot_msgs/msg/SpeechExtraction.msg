# Speech extraction result from audio processing
# Contains extracted object color and goal bin color from voice commands

std_msgs/Header header

# The color of the object to manipulate (e.g., "blue", "red", "yellow")
string object_color

# The color of the target bin (e.g., "red", "blue", "green")
string bin_color

# Whether the extraction was successful
bool success

# Original transcription text
string transcription

# Error message if extraction failed
string error_message
