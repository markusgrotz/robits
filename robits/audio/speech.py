from typing import Optional
import platform
import os

from robits.core.abc.speech import SpeechBase

from robits.audio.cache_utils import disk_cache
from robits.audio.cache_utils import text_to_cache_filename_fn

from robits.core.config_manager import config_manager
import subprocess


def get_default_player_backend():
    system_info = platform.uname()
    system_name = system_info.system
    if system_name == "Darwin":
        return "afplay"
    return "mplayer"


class CmdAudioPlayer:

    commands = {"mplayer": "mplayer %INPUT_FILE%", "afplay": "afplay %INPUT_FILE%"}

    def __init__(self, backend=None, **kwargs):
        self.backend = backend or get_default_player_backend()
        if self.backend not in self.commands.keys():
            raise ValueError("Invalid backend.")

    def play(self, filename: str):
        if not os.path.exists(filename):
            raise ValueError("Invalid audio file")

        cmd = self.commands[self.backend]
        cmd = cmd.replace("%INPUT_FILE%", filename)
        subprocess.call(cmd, shell=True)


class CmdSpeech(SpeechBase):

    commands = {"espeak": "espeak"}

    def __init__(self, backend: str, **kwargs):
        self.backend = backend or "espeak"
        if backend not in self.commands.keys():
            raise ValueError("Invalid backend.")

    def say(self, text: str):
        cmd = self.commands[self.backend]
        subprocess.call([cmd, text])


class CoquiTTS(SpeechBase):

    def __init__(self, output_backend: Optional[str] = None, **kwargs):
        from TTS.api import TTS

        self.model = TTS("tts_models/en/ljspeech/tacotron2-DDC")
        self.player = CmdAudioPlayer(output_backend)

    @disk_cache(text_to_cache_filename_fn)
    def synthesize_speech(self, text: str) -> str:
        filename = text_to_cache_filename_fn(text=text)
        self.model.tts_to_file(text, filename)
        with open("mapping.txt", "a") as f:
            f.write(f"{filename} - {text}\n")
        return filename

    def say(self, text: str):
        audio_filename = self.synthesize_speech(text)
        self.player.play(audio_filename)


class OpenAIAPI(SpeechBase):

    def __init__(self, output_backend: Optional[str] = None, **kwargs):
        from openai import OpenAI

        self.client = OpenAI()
        self.config = {"model": "tts-1", "voice": "alloy"}
        self.player = CmdAudioPlayer(output_backend)

    @disk_cache(text_to_cache_filename_fn)
    def synthesize_speech(self, text: str) -> str:
        filename = text_to_cache_filename_fn(text=text)
        response = self.client.audio.speech.create(input=text, **self.config)
        cache_dir = config_manager.get_main_config().default_cache_dir
        response.write_to_file(filename)
        with open(cache_dir / "mapping.txt", "a") as f:
            f.write(f"{filename} - {text}\n")
        return filename

    def say(self, text: str):
        audio_filename = self.synthesize_speech(text)
        self.player.play(audio_filename)
