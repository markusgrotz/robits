import subprocess

import logging
from logging import Handler


def play_audio(audio_file: str) -> None:
    """
    Currently synchronous
    """
    try:
        subprocess.call(
            ["mplayer", audio_file], stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
    except (ValueError, OSError):
        pass


def play_camera_shutter():
    audio_file = "/usr/share/sounds/freedesktop/stereo/camera-shutter.oga"
    play_audio(audio_file)


def play_attention_sound():
    audio_file = "/usr/share/sounds/freedesktop/stereo/window-attention.oga"
    play_audio(audio_file)


def play_sound_sound():
    audio_file = "/usr/share/sounds/freedesktop/stereo/bell.oga"
    play_audio(audio_file)


def play_info_sound():
    audio_file = "/usr/share/sounds/freedesktop/stereo/dialog-info.oga"
    play_audio(audio_file)


def play_error_sound():
    audio_file = "/usr/share/sounds/freedesktop/stereo/dialog-error.oga"
    play_audio(audio_file)


class AudioHandler(Handler):
    """
    .. todo:: this should be done async
    """

    def emit(self, record):
        if record.levelno >= logging.ERROR:
            play_error_sound()
        elif record.levelno == logging.WARNING:
            play_attention_sound()
