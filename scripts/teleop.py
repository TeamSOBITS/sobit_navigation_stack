import ipywidgets as widgets
import kachaka_api
from IPython.display import display

client = kachaka_api.KachakaApiClient("192.168.11.19:26400")


def move(linear, angular):
    client.set_robot_velocity(linear=linear, angular=angular)


def add_button(label: str, linear: float, angular: float):
    button = widgets.Button(description=label)
    button.on_click(lambda _: move(linear, angular))
    return button


items = [
    add_button("", 0.5, 0.5),
    add_button("Forward", 0.5, 0),
    add_button("", 0.5, -0.5),
    add_button("Turn left", 0, 0.5),
    add_button("Stop", 0, 0),
    add_button("Turn right", 0, -0.5),
    add_button("", -0.5, 0.5),
    add_button("Backward", -0.5, 0),
    add_button("", -0.5, -0.5),
]
widgets.GridBox(items, layout=widgets.Layout(grid_template_columns="repeat(3, 150px)"))