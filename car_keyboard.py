import threading
import time

import serial
from pynput import keyboard


SERIAL_PORT = "/dev/cu.usbmodem11201"
BAUD_RATE = 115200
SEND_INTERVAL = 0.08

active_command: str | None = None
running = True

ser = serial.Serial(
    SERIAL_PORT,
    BAUD_RATE,
    timeout=0.1,
)

time.sleep(2)


def send_command(command: str) -> None:
    ser.write(command.encode("ascii"))


def repeat_command() -> None:
    while running:
        if active_command is not None:
            send_command(active_command)

        time.sleep(SEND_INTERVAL)


def on_press(key):
    global active_command

    try:
        char = key.char.lower()

        if char in {"w", "a", "s", "d"}:
            active_command = char.upper()

    except AttributeError:
        if key == keyboard.Key.space:
            active_command = None
            send_command("U")


def on_release(key):
    global active_command
    global running

    try:
        char = key.char.lower()

        if char in {"w", "a", "s", "d"}:
            active_command = None
            send_command("X")

    except AttributeError:
        if key == keyboard.Key.esc:
            active_command = None
            send_command("X")

            running = False
            return False

    return None


sender = threading.Thread(
    target=repeat_command,
    daemon=True,
)

sender.start()

print("W/A/S/D: control")
print("Space: 180-degree turn")
print("Esc: stop and exit")

with keyboard.Listener(
    on_press=on_press,
    on_release=on_release,
) as listener:
    listener.join()

ser.close()
