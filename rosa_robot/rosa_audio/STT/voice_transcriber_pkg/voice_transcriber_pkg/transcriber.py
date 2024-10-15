import pyaudio
import wave
import whisper
import numpy as np
import rclpy
from rclpy.node import Node

from rclpy.action.client import ActionClient
from std_msgs.msg import String
from audio_common_msgs.action import TTS


class VoiceTranscriber(Node):
    def __init__(self):
        # ROS2 attributes
        super().__init__('voice_transcriber')
        self.publisher_ = self.create_publisher(String, '/voice', 10)
         # Action client for TTS action
        self._action_client = ActionClient(self, TTS, "say")

        # STT (whisper) and audio config
        self.model = whisper.load_model("large")
        self.frames = []

        self.audio = pyaudio.PyAudio()

        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        self.CHUNK = 10000

        # Default values for TTS message
        self.language = 'es'
        self.rate = 0.4 
        self.volume = 0.5  
        self.gender = 'f1'

        # Flag
        self.recording = True

        # Open audio stream
        self.stream = self.audio.open(format=self.FORMAT, channels=self.CHANNELS,
                                       rate=self.RATE, input=True,
                                       frames_per_buffer=self.CHUNK)

        self.start_recording()


    def start_recording(self): 
        try:
            self.frames = []

            # Recording loop
            while True:
                # Read audio stream
                if self.recording :
                    data = self.stream.read(self.CHUNK)

                # If silence is detected transcribe the buffer
                if self.is_silence(data) :
                    self.get_logger().info("SILENCE DETECTED")
                    if self.frames: 
                        self.get_logger().info("TRANSCRIBING...")
                        # Close stream to stop audio input and delete remaining data 
                        #IMPORTANT: Stream continues receiving data even when entered in the silence conditional, it must be closed to delete it
                        self.stream.stop_stream()
                        self.stream.close()

                        # Flag->False
                        self.recording = False

                        # Save audio data in a temporal .wav to transcribe
                        wf = wave.open("output.wav", 'wb')
                        wf.setnchannels(1)
                        wf.setsampwidth(self.audio.get_sample_size(pyaudio.paInt16))
                        wf.setframerate(16000)
                        wf.writeframes(b''.join(self.frames))
                        wf.close()

                        # Clean frames for the next iteration
                        self.frames = []

                        # Transcrive audio .wav
                        self.process_audio()

                        # Reopen stream for the next iteration
                        self.stream = self.audio.open(format=self.FORMAT, channels=self.CHANNELS,
                                                rate=self.RATE, input=True,
                                                frames_per_buffer=self.CHUNK)

                        # Flag->True
                        self.recording = True
                # If there is no silence keep recording
                else:
                    if self.recording:
                        self.get_logger().info("RECORDING...")
                        self.frames.append(data)

        except KeyboardInterrupt:
            print("Recording stopped.")

        finally:
            self.get_logger().info("Record finished.")
            self.stop_recording()

    # Detect silence function
    # Change threshold acording to the mic and the enviroment
    def is_silence(self, data, threshold=800):
        audio_data = np.frombuffer(data, dtype=np.int16)
        return np.abs(audio_data).mean() < threshold
    
    # STT function
    def process_audio(self):
        # Transcribe .wav
        result = self.model.transcribe("output.wav", language="es")

        # Set the string
        msg = String()
        msg.data = result["text"]

        # Publish in to /voice topic
        self.publisher_.publish(msg)
        self.get_logger().info(f"Transcription: {msg.data}")

        # Send to TTS action
        self.send_goal(msg.data)

    # Client send goal to action server
    def send_goal(self, frase_text: str):
        # Set the message
        goal_msg = TTS.Goal()
        goal_msg.text = "Has dicho: " + frase_text  
        goal_msg.rate = self.rate
        goal_msg.language = self.language  
        goal_msg.volume = self.volume 
        goal_msg.gender = self.gender

        self.get_logger().info(f"Sending goal: {frase_text}")


        # Wait for action server(TTS) availabe to send goal 
        # Actually not necessary due to recording loop only processes sentences one by one
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal denied")
            return

        # Wait for the action result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
    
    # Shut down sequence
    def stop_recording(self):
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
    

def main(args=None):
    rclpy.init(args=args)
    node = VoiceTranscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()