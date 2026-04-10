from abc import ABC
from abc import abstractmethod


class SpeechBase(ABC):
    """
    A general class that models TTS systems
    """

    @abstractmethod
    def say(self, text: str):
        """
        Verbalizes a given text.

        .. todo:: Add support for non-blocking call

        :param text: the text to verbalize
        """
        pass
