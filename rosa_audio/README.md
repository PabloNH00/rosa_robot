# VOICE TRANSCRIBER FOR ROSA

This folder contains the ROSA communication module demo. It is formed by a [Text-To-Speech](TTS/) and a [Speech-To-Text](STT/voice_transcriber_pkg/) packages.

## TTS package

It is cloned from the [audio_common repository](https://github.com/mgonzs13/audio_common) **(follow the install instructions for the dependencies)**, which contains multiple nodes for using audio with ROS2. All of them are included in the ROSA project for possible future work although only **tts_node** and **audio_player_node** are used for the demo.

### tts_node & modifications

This node contains a ROS Action called TTS which receives a string and modulate the transcription to sound using the "eSpeak" library. A /voice subscriber is also added for ROSA to manage the STT->TTS communication through a topic (/voice) if necessary.

Parameters for the TTS voice are specified in the action request goal that STT sends

```python
# Default values for TTS message
self.language = 'es'
self.rate = 0.4 
self.volume = 0.5  
self.gender = 'f1'
```

## STT package

This package is entirely made for ROSA and uses the OpenAI library called [whisper](https://nlpcloud.com/es/how-to-install-and-deploy-whisper-the-best-open-source-alternative-to-google-speech-to-text.html) **(click to go to the install guide)** to transcribe the sound captured from the audio input.

### transcriber node

This node loads the large model of whisper (can be "small" or "medium") and creates a client for the TTS action from [audio_common package](TTS/audio_common/audio_common/tts_node.py).

This node is in a permanent loop where audio is saved in a buffer while sound is detected. Once the audio is considered silence (below a configurable threshold) the buffer is processed and saved in a temporary "output.wav" to be transcribed by whisper.

**IMPORTANT: Audio stream is closed and reopenned each time silence is detected to avoid audio capture and residual sounds to be saved while transcribing** 

When the string is generated from the transcription it is send as a goal for the TTS action plays it and published to the /voice topic, then the loop continues recording or waiting to a sound.

## ROSA Audio Demo

To make this demo, a launcher called **rosa_voice.launch.py** is created in the [STT package](STT/voice_transcriber_pkg/launch/rosa_voice.launch.py).

    ros2 launch voice_transcriber_pkg rosa_voice.launch.py

It will launch the "tts_node" and "audio_player_node" from audio_common package and the "transcriber" node from STT package

The launcher execute all the nodes and the demo execution process is:
- transducer node starts recording
- Once the sound is ended (silence detected) the node publish the text transduced from a temporary .wav in the topic /voice
- The node also send a goal to the TTS node and waits for its execution
  - The transcriber will close the audio stream when is transcribing because if not, stream continue storing some audio after transcribing has started 
- transcriber node records again only if sound is detected, otherwise it will wait until that
- The loop ends when user sends a shut-down command from the shell