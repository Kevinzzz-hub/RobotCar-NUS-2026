import socket
import threading
import time

import serial
from serial import SerialException


SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

TCP_HOST = "0.0.0.0"
TCP_PORT = 5005

VALID_COMMANDS = set(b"WASDXUOM")

running = True


def open_arduino() -> serial.Serial:
    while True:
        try:
            arduino = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                timeout=0.2,
                write_timeout=None,
            )

            print("Waiting for Arduino startup...")
            time.sleep(6)

            arduino.reset_input_buffer()
            arduino.reset_output_buffer()

            print("Starting Arduino handshake...")

            handshake_ok = False

            for attempt in range(10):
                arduino.write(b"H")

                response = arduino.read(1)

                if response == b"A":
                    arduino.write(b"R")
                    handshake_ok = True
                    break

                time.sleep(0.5)

            if not handshake_ok:
                print("Handshake failed. Reconnecting...")
                arduino.close()
                time.sleep(2)
                continue

            print("Arduino handshake complete.")
            print(f"Arduino ready on {SERIAL_PORT}")

            return arduino

        except SerialException as error:
            print(f"Arduino connection failed: {error}")
            time.sleep(2)

def read_arduino_loop(arduino: serial.Serial) -> None:
    latest_lines = []
    last_print_time = time.time()

    while running:
        try:
            line = arduino.readline()

            if line:
                text = line.decode(
                    "utf-8",
                    errors="ignore",
                ).strip()

                if text:
                    latest_lines.append(text)

                    # ????? 20 ?
                    if len(latest_lines) > 20:
                        latest_lines.pop(0)

            # ? 30 ?????
            if time.time() - last_print_time >= 30:
                print()
                print("===== Arduino status =====")

                if latest_lines:
                    for item in latest_lines:
                        print(item)
                else:
                    print("No new Arduino data.")

                print("==========================")
                print()

                latest_lines.clear()
                last_print_time = time.time()

        except SerialException as error:
            print(f"Arduino read error: {error}")
            return

def safely_send(
    arduino: serial.Serial,
    command: bytes,
) -> bool:
    try:
        arduino.write(command)
        return True

    except SerialException as error:
        print(f"Arduino write error: {error}")
        return False


def safely_stop(arduino: serial.Serial) -> None:
    safely_send(arduino, b"X")


def main() -> None:
    global running

    arduino = open_arduino()

    reader_thread = threading.Thread(
        target=read_arduino_loop,
        args=(arduino,),
        daemon=True,
    )
    reader_thread.start()

    server = socket.socket(
        socket.AF_INET,
        socket.SOCK_STREAM,
    )

    server.setsockopt(
        socket.SOL_SOCKET,
        socket.SO_REUSEADDR,
        1,
    )

    server.bind((TCP_HOST, TCP_PORT))
    server.listen(1)

    print(f"Listening on port {TCP_PORT}")

    try:
        while True:
            connection, address = server.accept()

            connection.setsockopt(
                socket.SOL_SOCKET,
                socket.SO_KEEPALIVE,
                1,
            )

            print(f"Mac connected: {address}")

            try:
                while True:
                    data = connection.recv(1024)

                    if not data:
                        print("Mac disconnected.")
                        break

                    for command in data.upper():
                        # H ??????
                        if command == ord("H"):
                            continue

                        if command not in VALID_COMMANDS:
                            continue

                        safely_send(
                            arduino,
                            bytes([command]),
                        )

            except (
                ConnectionResetError,
                BrokenPipeError,
                OSError,
            ) as error:
                print(f"Network error: {error}")

            finally:
                safely_stop(arduino)
                connection.close()

                print(
                    "Car stopped. "
                    "Waiting for connection..."
                )

    except KeyboardInterrupt:
        print("\nServer stopped.")

    finally:
        running = False

        safely_stop(arduino)

        arduino.close()
        server.close()


if __name__ == "__main__":
    main()
