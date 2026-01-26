import serial
from serial.tools import list_ports

from textual import on
from textual import work
from textual.app import App, ComposeResult
from textual.widgets import RichLog, Footer, TabbedContent, Label, TabPane, Select, Input, SelectionList, Collapsible, Rule, Button
from textual.widgets.selection_list import Selection
from textual.containers import Horizontal, Vertical
from textual.worker import get_current_worker
from textual import log
from textual import color
from textual.suggester import SuggestFromList

from dataclasses import dataclass
import json

@dataclass
class Device():
    device:str
    name:str

@dataclass
class Interface():
    name: str
    topic: str
    _internal_id:int
    color: str

@dataclass
class Message():
    text: str
    itf: Interface

@dataclass
class State:
    active_interfaces: set[str]
    current_serial_port: str

def get_serial_interfaces():
    devices = [Device(device.device, device.name) for device in list_ports.comports()]
    devices.extend(custom_devices)
    devices.sort(key=lambda x: x.device)
    return ((f"{line.device} - {line.name}", line.device) for line in devices)

custom_devices = [
    Device("/dev/pts/4", "pts4")
]

# Do not change _internal_id
interfaces = [
    Interface(name="Device 0", topic="uart0", _internal_id="uart0", color="$primary-darken-2"),
    Interface(name="Device 1", topic="uart1", _internal_id="uart1", color="$primary-lighten-3"),
    Interface(name="Device 2", topic="uart2", _internal_id="uart2", color="$warning-darken-2"),
    Interface(name="Device 3", topic="uart3", _internal_id="uart3", color="$warning-lighten-3"),
    Interface(name="Device 4", topic="uart4", _internal_id="uart4", color="$error"),
    Interface(name="Device 5", topic="uart5", _internal_id="uart5", color="$success")
]

interfaces_dict = {itf._internal_id:itf for itf in interfaces}
devices = get_serial_interfaces()

# test text
# {"topic":"uart0", "message":"hello world!"}

class ReadView(Vertical):

    BINDINGS = [
        ("a", "go_to_all_tab", "all"), 
        # ("1", "go_to_uart0_tab", interfaces_dict["uart0"].name), 
        # ("2", "go_to_uart1_tab", interfaces_dict["uart1"].name),
        # ("3", "go_to_uart2_tab", interfaces_dict["uart2"].name),
        # ("4", "go_to_uart3_tab", interfaces_dict["uart3"].name),
        # ("5", "go_to_uart4_tab", interfaces_dict["uart4"].name),
        # ("6", "go_to_uart5_tab", interfaces_dict["uart5"].name),
    ]
            
    def __init__(self, *children, state = None, name = None, id = None, classes = None, disabled = False, markup = True):
        self.state = state
        super().__init__(*children, name=name, id=id, classes=classes, disabled=disabled, markup=markup)

    def compose(self) -> ComposeResult:
        labels = [Label(f"[{itf.color}] {itf.name}") for itf in interfaces]
        selections = [Selection(f"[{itf.color}] {itf.name}", itf._internal_id, itf._internal_id in self.state.active_interfaces) for itf in interfaces]
        with TabbedContent(id="itf_tabs"):
            with TabPane("All interfaces", id="all_tab"):
                yield Collapsible(SelectionList(*selections), title="Filter")
                yield Horizontal(Label("Legend: "),*labels, classes="horizontal_ctn")
                yield Rule(line_style="heavy", id="rx_all_rule")
                yield RichLog(markup=True, id="all_log", classes="log", auto_scroll=False)
            for key, itf in interfaces_dict.items():
                with TabPane(title=itf.name, id=key):
                    yield RichLog(markup=True, id=f"{key}_log", classes="log", auto_scroll=False)
    
    def action_go_to_all_tab(self) -> None:
        self.query_one("#itf_tabs").active = "all_tab"
                 
    def action_go_to_uart0_tab(self) -> None:
        self.query_one("#itf_tabs").active = "uart0"

    def action_go_to_uart1_tab(self) -> None:
        self.query_one("#itf_tabs").active = "uart1"

    def action_go_to_uart2_tab(self) -> None:
        self.query_one("#itf_tabs").active = "uart2"

    def action_go_to_uart3_tab(self) -> None:
        self.query_one("#itf_tabs").active = "uart3"

    def action_go_to_uart4_tab(self) -> None:
        self.query_one("#itf_tabs").active = "uart4"
    
    def action_go_to_uart5_tab(self) -> None:
        self.query_one("#itf_tabs").active = "uart5"

    @on(SelectionList.SelectedChanged)
    def filter_changed(self, event: Select.Changed) -> None:
        self.state.active_interfaces = set(self.query_one(SelectionList).selected)

class SendView(Vertical):

    BINDINGS = [
        ("ctrl+enter", "send_message", "Send Message"),
        ("s", "auto_scroll", "auto-scroll"),
    ]

    def compose(self) -> ComposeResult:
        yield Label("Topic", classes="padding")
        yield Input(suggester=SuggestFromList([itf.topic for itf in interfaces]))
        yield Label("Message", classes="padding")
        yield Input()
        yield Button("Send", variant="primary")
        yield Rule(line_style="heavy")
        yield RichLog(markup=True, id="sent_log", classes="log", auto_scroll=False)

class RichLogApp(App):

    BINDINGS = [
        ("ctrl+r", "go_to_receive", "Go to Rx"),
        ("ctrl+t", "go_to_transmit", "Go to Tx"),
        ("ctrl+u", "refresh_itf", "Refresh serial ports"),
        ("ctrl+i", "change_itf", "Change serial port"),
        ("c", "clear_logs", "clear"),
        ("s", "auto_scroll", "auto-scroll"),
    ]

    CSS_PATH = "app.tcss"

    def __init__(self, driver_class = None, css_path = None, watch_css = False, ansi_color = False):
        self.state = State(set(itf for itf in interfaces_dict.keys()), None)
        super().__init__(driver_class, css_path, watch_css, ansi_color)

    def compose(self) -> ComposeResult:
        yield Select(get_serial_interfaces(), id="serial_select")
        with TabbedContent(id="rxtx_tabs"):
            with TabPane("Read", id="read_tab"):
                yield ReadView(state=self.state)
            with TabPane("Send", id="send_tab"):
                yield SendView()
        yield Footer()
                

    def on_ready(self) -> None:
        """Called  when the DOM is ready."""
        self.open_serial()

    def action_refresh_itf(self) -> None:
        selector = self.query_one("#serial_select")
        old_value = selector.value
        interfaces = list(get_serial_interfaces())
        self.state.current_serial_port = old_value
        selector.set_options(interfaces)
        for itf in interfaces:
            if old_value == itf[1]:
                selector.value = old_value
                break
        else:
            self.state.current_serial_port = None

    def action_go_to_receive(self) -> None:
        self.query_one("#rxtx_tabs").active = "read_tab"

    def action_go_to_transmit(self) -> None:
        self.query_one("#rxtx_tabs").active = "send_tab"

    def action_change_itf(self) -> None:
        self.query_one("#serial_select").action_show_overlay()

    def action_auto_scroll(self) -> None:
        text_log_all = self.query_one("#all_log")
        text_log_all.auto_scroll = not text_log_all.auto_scroll
        text_log_send = self.query_one("#sent_log")
        text_log_send.auto_scroll = not text_log_send.auto_scroll
        for itf in interfaces_dict.values():
            text_log_itf = self.query_one(f"#{itf._internal_id}_log")
            text_log_itf.auto_scroll = not text_log_itf.auto_scroll

    def action_clear_logs(self) -> None:
        text_log_all = self.query_one("#all_log")
        text_log_all.clear()
        text_log_send = self.query_one("#sent_log")
        text_log_send.clear()
        for itf in interfaces_dict.values():
            text_log_itf = self.query_one(f"#{itf._internal_id}_log")
            text_log_itf.clear()
            
    def get_color(self, color_str):
        if color_str.startswith("$"):
            var = color_str[1:]
            if var in self.theme_variables:
                return self.theme_variables[var]
            return "#ffffff"
        else:
            r, g, b = color.Color.parse(color_str).rgb
            return color.Color.parse(f"rgb({r}, {g}, {b})").hex
            
    def display_text(self, message):
        color = self.get_color(message.itf.color)
        if message.itf._internal_id in self.state.active_interfaces:
            text_log_all = self.query_one("#all_log")
            text_log_all.write(f"[{color}]{message.text}[/]")
        text_log_itf = self.query_one(f"#{message.itf._internal_id}_log")
        text_log_itf.write(f"[{color}]{message.text}[/]")

    def print_open(self, dev):
        text_log_all = self.query_one("#all_log")
        text_log_all.write(f"-- Opened {dev} --")
        text_log_sent = self.query_one("#sent_log")
        text_log_sent.write(f"-- Opened {dev} --")
        for itf in interfaces_dict.values():
            text_log_itf = self.query_one(f"#{itf._internal_id}_log")
            text_log_itf.write(f"-- Opened {dev} --")

    def print_open_failed(self, dev):
        text_log_all = self.query_one("#all_log")
        text_log_all.write(f"-- Failed to open {dev} --")
        text_log_sent = self.query_one("#sent_log")
        text_log_sent.write(f"-- Failed to open {dev} --")
        for itf in interfaces_dict.values():
            text_log_itf = self.query_one(f"#{itf._internal_id}_log")
            text_log_itf.write(f"-- Failed to open {dev} --")


    def print_serial_error(self, dev):
        text_log_all = self.query_one("#all_log")
        text_log_all.write(f"-- {dev} was closed --")
        text_log_sent = self.query_one("#sent_log")
        text_log_sent.write(f"-- {dev} was closed --")
        for itf in interfaces_dict.values():
            text_log_itf = self.query_one(f"#{itf._internal_id}_log")
            text_log_itf.write(f"-- {dev} was closed --")

    def clear_serial_device(self):
        self.query_one("#serial_select").clear()

    @on(Select.Changed)
    def select_changed(self, event: Select.Changed) -> None:
        self.title = str(event.value)
        if event.value == Select.BLANK and self.state.current_serial_port is None:
            self.open_serial()
        elif event.value != Select.BLANK and (event.value != self.state.current_serial_port):
            self.open_serial(event.value)

    @work(exclusive=True, thread=True)
    def open_serial(self, dev=None):
        if dev is None:
            return
        try:
            with serial.Serial(dev) as device:
                self.call_from_thread(self.print_open, dev)
                worker = get_current_worker()
                while not worker.is_cancelled and device.is_open:
                    if device.in_waiting != 0:
                        message = self.get_line_info(device.readline())
                        if message.itf is None:
                            self.call_from_thread(self.send_to_all, message.text)
                        elif message.itf.topic in interfaces_dict.keys():
                            self.call_from_thread(self.display_text, message)
                if worker.is_cancelled:
                    return

        except serial.SerialException:
            self.call_from_thread(self.print_open_failed, dev)
        except OSError:
            pass
        self.call_from_thread(self.print_serial_error, dev)
        self.call_from_thread(self.clear_serial_device)
        self.call_from_thread(self.action_refresh_itf)

    def send_to_all(self, msg):
        text_log_all = self.query_one("#all_log")
        text_log_all.write(f"[red]{msg}")

    def get_line_info(self, line):
        try:
            jsonData = json.loads(line.decode("utf-8")[:-1])
            topic = jsonData["topic"] 
            itf_data = interfaces_dict[topic]
            msg = jsonData["message"]
            return Message(msg, itf_data)
        except json.JSONDecodeError:
            return Message("[ERROR]: ---Invalid Message---", None)
            


if __name__ == "__main__":
    app = RichLogApp()
    app.run()
