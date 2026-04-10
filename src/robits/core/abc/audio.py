from abc import ABC
from abc import abstractmethod

from typing import Optional

from pathlib import Path

import logging

logger = logging.getLogger(__name__)


class RecorderBase:
    """
    A general class to model a audio recorder
    """

    def __init__(self) -> None:
        self.outfile: Path

    @abstractmethod
    def start_recording(self) -> Path:
        pass

    @abstractmethod
    def stop_recording(self) -> Path:
        pass


class AudioBase(ABC):
    """
    A general class that models audio interaction
    """

    def __init__(self) -> None:
        self.recorder: RecorderBase

    def __enter__(self) -> "AudioBase":
        self.recorder.start_recording()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.recorder.stop_recording()

    def process(self) -> Optional[str]:
        """
        Transcribes the recorded audio
        """
        filename = self.recorder.outfile

        if not filename:
            logger.warning(
                "No audio recorded. Please use with statement to record first."
            )
            return None

        logger.debug("Transcribing %s", filename)
        text = self._transcribe(filename)
        return text

    @abstractmethod
    def _transcribe(self, filename: Path) -> str:
        """
        Implementation to transcribe the audio
        """
        pass
