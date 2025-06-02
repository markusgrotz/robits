from typing import Optional
from typing import List

from threading import Thread
import tempfile
import logging

from pathlib import Path
from subprocess import Popen

from robits.core.abc.audio import AudioBase
from robits.core.abc.audio import RecorderBase


logger = logging.getLogger(__name__)


class CmdAudioRecorder(RecorderBase):

    commands = {
        "pulse": "parecord --file-format=wav %OUTPUT_FILE%",
        "alsa": "arecord --format=cd %OUTPUT_FILE%",
    }

    def __init__(self, backend: Optional[str] = None) -> None:
        self.backend = backend or "pulse"
        if self.backend not in self.commands.keys():
            raise ValueError("Invalid backend")
        self.outfile: Path = Path()

    def start_recording(self) -> Path:
        _, path = tempfile.mkstemp(".wav")
        self.outfile = Path(path)
        cmd = self.commands[self.backend].replace("%OUTPUT_FILE%", str(self.outfile))
        logger.debug("Starting recording with %s", cmd)
        self.p = Popen(cmd.split(" "))
        return self.outfile

    def stop_recording(self) -> Path:
        logger.debug("Stopping audio recording.")
        self.p.terminate()
        return self.outfile


class SdAudioRecorder(RecorderBase):
    def __init__(self, rate: int = 16000, channels: int = 1, chunk: int = 1024):
        self.rate = rate
        self.channels = channels
        self.chunk = chunk
        self.frames: List[bytes] = []
        self.recording = False

    def start_recording(self) -> Path:
        _, path = tempfile.mkstemp(suffix=".wav")
        self.outfile = Path(path)
        self.frames = []
        self.recording = True
        Thread(target=self._record).start()
        return self.outfile

    def _record(self) -> None:
        import sounddevice as sd

        with sd.InputStream(
            samplerate=self.rate, channels=self.channels, dtype="int16"
        ) as stream:
            while self.recording:
                data, _ = stream.read(self.chunk)
                self.frames.append(data.tobytes())

    def stop_recording(self) -> Path:
        import wave

        self.recording = False
        with wave.open(str(self.outfile), "wb") as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(2)
            wf.setframerate(self.rate)
            wf.writeframes(b"".join(self.frames))
        return self.outfile


class OpenAIWhisper(AudioBase):

    def __init__(self, recorder_type: str = "sd", **kwargs):
        import whisper

        if recorder_type == "sd":
            self.recorder = SdAudioRecorder()
        elif recorder_type == "cmd":
            self.recorder = CmdAudioRecorder()
        else:
            raise ValueError("Invalid recorder")

        self.model = whisper.load_model("medium.en")

    def _transcribe(self, filename: Path) -> str:
        result = self.model.transcribe(str(filename))
        return result["text"]
