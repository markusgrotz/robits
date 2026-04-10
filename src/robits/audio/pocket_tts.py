from typing import Optional

import logging

from pocket_tts import TTSModel

import scipy


from robits.audio.speech import SpeechBase
from robits.audio.speech import CmdAudioPlayer

from robits.audio.cache_utils import disk_cache
from robits.audio.cache_utils import text_to_cache_filename_fn

from robits.core.config_manager import config_manager


logger = logging.getLogger(__name__)


class PocketTTS(SpeechBase):
    def __init__(self, output_backend: Optional[str] = None, **kwargs):

        self.tts_model = TTSModel.load_model()

        logger.debug(
            "Model running on device %s and sample rate %d",
            self.tts_model.device,
            self.tts_model.sample_rate,
        )

        # ['alba', 'marius', 'javert', 'jean', 'fantine', 'cosette', 'eponine', 'azelma'].
        self.voice_state = self.tts_model.get_state_for_audio_prompt("cosette")

        self.player = CmdAudioPlayer(output_backend)

    @disk_cache(text_to_cache_filename_fn)
    def synthesize_speech(self, text: str) -> str:

        filename = text_to_cache_filename_fn(text=text)

        audio = self.tts_model.generate_audio(self.voice_state, text)
        scipy.io.wavfile.write(filename, self.tts_model.sample_rate, audio.numpy())

        cache_dir = config_manager.get_main_config().default_cache_dir
        with open(cache_dir / "mapping.txt", "a") as f:
            f.write(f"{filename} - {text}\n")
        return filename

    def say(self, text: str):
        audio_filename = self.synthesize_speech(text)
        self.player.play(audio_filename)
