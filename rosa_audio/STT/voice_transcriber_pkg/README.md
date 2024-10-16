# VOICE TRANSCRIBER FOR ROSA

This package is part of the ROSA communication module demo. It contains the launcher for the demo and a transcriber node, which uses the whisper library from OpenAI.

The transcriber node publishes in the topic /voice the transcribed string.
For the demo it also send a goal to the TTS action to speech the text

To play the audio, the audio_common package from https://github.com/mgonzs13/audio_common is modified to adapt the TTS node to receive the string from /voice

The launcher execute all the nodes and the demo execution process is:
- transducer node only start recording when the first noise is detected
- Once the noise is ended the node publish the text transduced from a temporary .wav in the topic /voice
- The node also send a goal to the TTS node and waits for its execution
- The transcriber will close the audio stream when is transcribing because stream continue storing some audio after transcribing has started 