import io
import base64

import numpy as np

from PIL import Image

from openai import OpenAI


class PromptBuilder:

    def __init__(self):
        self.content = []

    def add_instruction(self, text: str):
        self.content.append({"type": "text", "text": text})

    def add_image(self, np_image: np.ndarray, resize=False):

        image = Image.fromarray(np_image)

        if resize:
            width, height = image.size
            ratio = height / width
            image = image.resize((int(256), int(256 * ratio)))

        # image.save("/tmp/test.png")
        with io.BytesIO() as output:
            image.save(output, format="png")
            base64_image = base64.b64encode(output.getvalue()).decode("utf-8")

        # ..todo:: set the detail flag
        self.content.append(
            {
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/png;base64,{base64_image}",
                    "detail": "auto",
                },
            }
        )  # low, high, auto

        return image

    def build(self):
        return {"role": "user", "content": self.content}


class ChatGPT:
    """
    https://platform.openai.com/docs/guides/vision?lang=curl
    """

    def __init__(self):
        self.client = OpenAI()

    def query(self, prompt) -> str:

        if isinstance(prompt, PromptBuilder):
            prompt = prompt.build()

        response = self.client.chat.completions.create(
            model="gpt-4o-mini", messages=[prompt], max_tokens=300
        )

        return response
