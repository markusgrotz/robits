from textual.widget import Widget
from textual.reactive import reactive
from textual.events import Key
from textual.message import Message
from textual.containers import Horizontal
from textual.containers import Vertical

from textual.widgets import Button, Static
from textual.app import ComposeResult


class Slider(Widget):
    """A simple horizontal slider widget with + and - buttons."""

    value = reactive(0.0)

    def __init__(self, min=0.0, max=1.0, value=0.0, step=0.1, id=None, name=None):
        self._mounted = False
        super().__init__(id=id, name=name)
        self.min = min
        self.max = max
        self.step = step
        self.bar_width = 30
        self.value = value

    def compose(self) -> ComposeResult:
        decrease_btn = Button("-", id="decrease")
        increase_btn = Button("+", id="increase")

        for btn in (decrease_btn, increase_btn):
            btn.styles.width = 3
            btn.styles.height = 3
            btn.styles.min_width = 3
            btn.styles.min_height = 3
            btn.styles.max_width = 3
            btn.styles.max_height = 3

        # Bar and label
        bar = Static(self._render_bar(), id="bar", expand=True)
        value_label = Static(f"{self.value:.3f}", id="value_label", expand=True)
        bar.styles.text_align = "center"
        value_label.styles.text_align = "center"

        center_column = Vertical(value_label, bar, id="slider-column")
        center_column.styles.align_horizontal = "center"

        yield Horizontal(decrease_btn, center_column, increase_btn, id="slider-row")

    def on_mount(self) -> None:
        self._mounted = True
        self.styles.width = self.bar_width + 10

    def _render_bar(self) -> str:
        normalized = (self.value - self.min) / (self.max - self.min)
        filled_chars = int(normalized * self.bar_width)
        empty_chars = self.bar_width - filled_chars
        return f"[{'█' * filled_chars}{' ' * empty_chars}]"

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "decrease":
            self.decrease()
        elif event.button.id == "increase":
            self.increase()

    def decrease(self):
        self.value = max(self.min, self.value - self.step)
        self.post_message(Slider.Changed(self))

    def increase(self):
        self.value = min(self.max, self.value + self.step)
        self.post_message(Slider.Changed(self))

    def on_key(self, event: Key) -> None:
        if event.key == "left":
            self.decrease()
        elif event.key == "right":
            self.increase()

    def watch_value(self, old: float, new: float) -> None:
        if self._mounted:
            self.query_one("#bar", Static).update(self._render_bar())
            self.query_one("#value_label", Static).update(f"{new:.2f}")

    class Changed(Message):
        """Message sent when the slider value changes."""

        def __init__(self, sender: "Slider"):
            super().__init__()
