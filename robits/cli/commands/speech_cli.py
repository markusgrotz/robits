#!/usr/bin/env python3


from robits.core.config_manager import config_manager
from robits.core.factory import SpeechFactory

from robits.core.factory import AudioFactory


import rich_click as click
from click_prompt import choice_option


from robits.cli.base_cli import cli
from robits.cli.cli_utils import console


@cli.group()
def speech():
    """
    Audio related commands
    """
    pass


@speech.command()
@choice_option(
    "--speech-system", type=click.Choice(config_manager.available_speech_backends)
)
@click.argument("text")
def say(speech_system, text: str):
    speech_system = SpeechFactory(speech_system).build()
    speech_system.say(text)


@speech.command()
@choice_option(
    "--transcribe-system", type=click.Choice(config_manager.available_audio_backends)
)
def transcribe(transcribe_system):
    console.print("Initializing. This might take a while.")
    transcribe_system = AudioFactory(transcribe_system).build()
    console.rule("Ready")
    with transcribe_system:
        input("Starting transcription. Press enter when you are done.")
    print(transcribe_system.process())


@speech.command()
@choice_option(
    "--speech-system", type=click.Choice(config_manager.available_speech_backends)
)
@choice_option(
    "--transcribe-system", type=click.Choice(config_manager.available_audio_backends)
)
def parrot(speech_system, transcribe_system):
    console.print("Initializing. This might take a while.")
    speech_system = SpeechFactory(speech_system).build()
    transcribe_system = AudioFactory(transcribe_system).build()
    console.rule("Ready.")
    with transcribe_system:
        input("Press enter when done.")

    console.print("Transcribing...")
    text = transcribe_system.process()
    if text:
        console.print(f"Received {text}")
        speech_system.say(text)
    else:
        console.print("Unable to transcribe audio.")

    console.rule("Done")


if __name__ == "__main__":
    cli()
