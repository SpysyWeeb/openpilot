"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections.abc import Callable

import pyray as rl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp, option_item_sp, multiple_button_item_sp
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.network import NavButton
from openpilot.system.ui.widgets.scroller_tici import Scroller

PERSONALITY_NAMES = ["Aggressive", "Standard", "Relaxed"]

ACCEL_DESC = "Maximum acceleration from a stop; the rest of the speed range scales to match. Stock is 1.6 m/s^2."
JERK_DESC = ("How much jerk (rate of change of acceleration) sunnypilot may use, relative to stock. " +
             "1.0x matches stock Standard and Relaxed; 2.0x matches stock Aggressive. " +
             "Lower is earlier and smoother, higher is later and firmer.")
FOLLOW_DESC = "Time gap to the lead vehicle. Stock: Aggressive 1.25 s, Standard 1.45 s, Relaxed 1.75 s."


class CustomPersonalityLayout(Widget):
  def __init__(self, back_btn_callback: Callable):
    super().__init__()
    self._back_button = NavButton(tr("Back"))
    self._back_button.set_click_callback(back_btn_callback)

    self._last_target = -1
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    self._enable_toggle = toggle_item_sp(
      title=tr("Enable Custom Personality"),
      description=tr("Give each Driving Personality your own acceleration, jerk, and follow distance. " +
                     "Switch personalities anytime, including with the steering wheel distance button. " +
                     "Changes apply within seconds, even while driving."),
      param="CustomPersonality",
      callback=self._on_enable_toggle)

    self._target_selector = multiple_button_item_sp(
      tr("Personality"), "",
      [tr("Aggressive"), tr("Standard"), tr("Relaxed")],
      button_width=300, callback=self._on_target_selected,
      param="CustomPersonalityTarget", inline=False)

    # one set of sliders; the personality picker rebinds them to the selected
    # personality's params
    self._accel_item = option_item_sp(
      title=tr("Acceleration"),
      param="CustomPersonalityStandardAccel",
      min_value=8, max_value=20, value_change_step=1,
      description=tr(ACCEL_DESC),
      label_callback=lambda value: f"{value / 10:.1f} m/s^2",
      inline=True)

    self._jerk_item = option_item_sp(
      title=tr("Jerk Value"),
      param="CustomPersonalityStandardJerkMultiplier",
      min_value=5, max_value=40, value_change_step=1,
      description=tr(JERK_DESC),
      label_callback=lambda value: f"{value / 10:.1f}x",
      inline=True)

    self._follow_item = option_item_sp(
      title=tr("Follow Distance"),
      param="CustomPersonalityStandardFollow",
      min_value=20, max_value=50, value_change_step=1,
      description=tr(FOLLOW_DESC),
      label_callback=lambda value: f"{value * 0.05:.2f} s",
      inline=True)

    self._sliders = {
      "Accel": self._accel_item,
      "JerkMultiplier": self._jerk_item,
      "Follow": self._follow_item,
    }

    return [self._enable_toggle, self._target_selector,
            self._accel_item, self._jerk_item, self._follow_item]

  def _retarget_sliders(self, target: int):
    name = PERSONALITY_NAMES[target]
    for knob, item in self._sliders.items():
      action = item.action_item
      action.param_key = f"CustomPersonality{name}{knob}"
      action.current_value = int(action.params.get(action.param_key, return_default=True))
    self._last_target = target

  @staticmethod
  def _on_target_selected(index):
    ui_state.params.put("CustomPersonalityTarget", index)

  def _update_state(self):
    super()._update_state()
    has_long = ui_state.CP is not None and ui_state.has_longitudinal_control
    self._enable_toggle.action_item.set_enabled(has_long)

    target = int(ui_state.params.get("CustomPersonalityTarget", return_default=True) or 0)
    target = min(max(target, 0), len(PERSONALITY_NAMES) - 1)
    if target != self._last_target:
      self._retarget_sliders(target)

    self._on_enable_toggle(self._enable_toggle.action_item.get_state())

  def _on_enable_toggle(self, state):
    # gray out instead of hiding: toggling item visibility mid-list corrupts the
    # scroller layout (items render merged)
    enabled = state and self._enable_toggle.action_item.enabled
    self._target_selector.action_item.set_enabled(enabled)
    for item in self._sliders.values():
      item.action_item.set_enabled(enabled)

  def _render(self, rect):
    self._back_button.set_position(self._rect.x, self._rect.y + 20)
    self._back_button.render()

    content_rect = rl.Rectangle(rect.x, rect.y + self._back_button.rect.height + 40, rect.width, rect.height - self._back_button.rect.height - 40)
    self._scroller.render(content_rect)

  def show_event(self):
    self._scroller.show_event()
    self._enable_toggle.show_description(True)

  def hide_event(self):
    self._scroller.hide_event()
