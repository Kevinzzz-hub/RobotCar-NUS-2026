import socket
import threading
import time
from pynput import keyboard

PI_HOST = "100.101.201.64"
PI_PORT = 5005

SEND_INTERVAL = 0.08
HEARTBEAT_INTERVAL = 1.0
RECONNECT_INTERVAL = 2.0

active_command = None
running = True
client = None

lock = threading.Lock()
socket_lock = threading.Lock()


def connect_to_pi():
    global client

    while running:
        try:
            new_client = socket.create_connection(
                (PI_HOST, PI_PORT),
                timeout=5
            )

            new_client.settimeout(None)

            with socket_lock:
                client = new_client

            print("Connected to Raspberry Pi.")
            return

        except OSError as error:
            print(f"Connection failed: {error}")
            time.sleep(RECONNECT_INTERVAL)


def disconnect():
    global client

    with socket_lock:
        if client is not None:
            try:
                client.close()
            except OSError:
                pass

            client = None


def send_command(command):
    global client

    encoded = command.encode("ascii")

    while running:
        with socket_lock:
            current_client = client

        if current_client is None:
            connect_to_pi()
            continue

        try:
            current_client.sendall(encoded)
            return

        except OSError:
            print("Connection lost. Reconnecting...")
            disconnect()


def command_loop():
    last_heartbeat = 0.0

    while running:
        now = time.time()

        with lock:
            command = active_command

        if command is not None:
            send_command(command)

        elif now - last_heartbeat >= HEARTBEAT_INTERVAL:
            send_command("H")
            last_heartbeat = now

        time.sleep(SEND_INTERVAL)


def on_press(key):
    global active_command

    try:
        char = key.char.lower()

        if char in {"w", "a", "s", "d"}:
            with lock:
                active_command = char.upper()

        elif char == "o":
            send_command("O")

        elif char == "m":
            send_command("M")

    except AttributeError:
        if key == keyboard.Key.space:
            with lock:
                active_command = None

            send_command("U")


def on_release(key):
    global active_command
    global running

    try:
        char = key.char.lower()

        if char in {"w", "a", "s", "d"}:
            with lock:
                active_command = None

            send_command("X")

    except AttributeError:
        if key == keyboard.Key.esc:
            with lock:
                active_command = None

            send_command("X")
            running = False
            return False

    return None


connect_to_pi()

sender_thread = threading.Thread(
    target=command_loop,
    daemon=True
)
sender_thread.start()

print("W/A/S/D: control")
print("O : autonomous mode")
print("M : manual mode")
print("Space: 180-degree turn")
print("Esc: stop and exit")

try:
    with keyboard.Listener(
        on_press=on_press,
        on_release=on_release
    ) as listener:
        listener.join()

finally:
    running = False

    try:
        send_command("X")
    except Exception:
        pass

    disconnect()
    print("Disconnected.")
