"""Stream mono 16 kHz PCM WAV audio to an A2DP-connected module over UART.

Usage:
    python tools/uart_audio_sender.py audio.wav

The module must be running the normal RM_pico firmware. The script sends the
A2DP setup commands at 115200 baud, switches to 460800 for PCM, and restores
115200 when playback ends.
"""

import argparse
import queue
import threading
import time
import wave

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial is required; install it with: py -m pip install pyserial") from exc


CONTROL_BAUD = 115200
AUDIO_BAUD = 460800
SAMPLE_RATE = 16000
SAMPLE_WIDTH_BYTES = 2
PREBUFFER_FRAMES = 2048
FRAMES_PER_CHUNK = 2048
VOLUME_MAX = 127
VOLUME_STEP = 8


def start_volume_input(commands):
    """Read stdin lines (0..127, '+' or '-') and queue AT+A2DPVOL commands."""
    def worker():
        volume = VOLUME_MAX // 2
        print(f"Volume control: type 0-{VOLUME_MAX}, '+' or '-' then Enter")
        while True:
            try:
                line = input().strip()
            except EOFError:
                return
            if line == "+":
                volume = min(VOLUME_MAX, volume + VOLUME_STEP)
            elif line == "-":
                volume = max(0, volume - VOLUME_STEP)
            elif line.isdigit() and int(line) <= VOLUME_MAX:
                volume = int(line)
            else:
                continue
            commands.put(b"AT+A2DPVOL%d\r\n" % volume)

    threading.Thread(target=worker, daemon=True).start()


def read_until(port, token, timeout, show_output=True):
    deadline = time.monotonic() + timeout
    received = bytearray()
    while time.monotonic() < deadline:
        chunk = port.read(256)
        if not chunk:
            continue
        if show_output:
            print(chunk.decode("ascii", "replace"), end="", flush=True)
        received.extend(chunk)
        if token in received:
            return bytes(received)
        if len(received) > 8192:
            del received[:-4096]
    raise TimeoutError(
        f"Timed out waiting for {token.decode('ascii')}; received {bytes(received)!r}")


def send_command(port, command, response, timeout, show_output=True):
    port.write(command + b"\r\n")
    port.flush()
    return read_until(port, response, timeout, show_output=show_output)


def find_control_baud(port):
    for baud in (CONTROL_BAUD, AUDIO_BAUD):
        port.baudrate = baud
        port.reset_input_buffer()
        try:
            response = send_command(port, b"AT+A2DPON", b"OK+A2DPON", 5, show_output=False)
            if b"Mode:A2DP Source" not in response:
                response += read_until(port, b"Mode:A2DP Source", 20, show_output=False)
        except TimeoutError:
            continue

        print(response.decode("ascii", "replace"), end="", flush=True)

        if baud == AUDIO_BAUD:
            port.write(b"AT+BAUD4\r\n")
            port.flush()
            time.sleep(0.1)
            port.baudrate = CONTROL_BAUD
            port.reset_input_buffer()
        return response

    raise TimeoutError("No A2DP response at 115200 or 460800 baud; reset the module")


def stream_wav(port, audio, commands):
    total_frames = audio.getnframes()
    prebuffer_frames = min(PREBUFFER_FRAMES, total_frames)
    sent_frames = prebuffer_frames
    start_time = time.monotonic() - prebuffer_frames / SAMPLE_RATE

    while True:
        data = audio.readframes(FRAMES_PER_CHUNK)
        if not data:
            break

        written = port.write(data)
        if written != len(data):
            raise serial.SerialTimeoutException("UART write was incomplete")

        # Sent between PCM chunks so the command is not interleaved with audio bytes.
        while True:
            try:
                port.write(commands.get_nowait())
            except queue.Empty:
                break

        sent_frames += len(data) // SAMPLE_WIDTH_BYTES
        deadline = start_time + sent_frames / SAMPLE_RATE
        while True:
            drain_uart(port)
            delay = deadline - time.monotonic()
            if delay <= 0:
                break
            time.sleep(min(delay, 0.02))

    port.flush()
    drain_uart(port)
    print(f"\nSent {sent_frames}/{total_frames} PCM frames")


def drain_uart(port):
    while port.in_waiting:
        data = port.read(min(port.in_waiting, 512))
        if data:
            print(data.decode("ascii", "replace"), end="", flush=True)


def send_prebuffer(port, audio):
    data = audio.readframes(PREBUFFER_FRAMES)
    sent_frames = 0
    start_time = time.monotonic()

    for offset in range(0, len(data), 512):
        chunk = data[offset:offset + 512]
        written = port.write(chunk)
        if written != len(chunk):
            raise serial.SerialTimeoutException("UART prebuffer write was incomplete")

        sent_frames += len(chunk) // SAMPLE_WIDTH_BYTES
        deadline = start_time + sent_frames / SAMPLE_RATE
        delay = deadline - time.monotonic()
        if delay > 0:
            time.sleep(delay)

    port.flush()
    return sent_frames


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("wav", help="mono 16-bit PCM WAV at 16000 Hz")
    parser.add_argument("--port", default="COM4", help="serial port (default COM4)")
    parser.add_argument("--address", default="FC58FAACAA1A",
                        help="A2DP sink MAC without separators (default FC58FAACAA1A)")
    args = parser.parse_args()

    if len(args.address) != 12 or any(char not in "0123456789abcdefABCDEF" for char in args.address):
        parser.error("--address must be exactly 12 hexadecimal digits without separators")

    port = None
    audio_baud_selected = False
    try:
        with wave.open(args.wav, "rb") as audio:
            if (audio.getcomptype() != "NONE" or audio.getnchannels() != 1 or
                    audio.getsampwidth() != SAMPLE_WIDTH_BYTES or
                    audio.getframerate() != SAMPLE_RATE):
                parser.error("WAV must be uncompressed mono 16-bit PCM at 16000 Hz")

            port = serial.Serial()
            port.port = args.port
            port.baudrate = CONTROL_BAUD
            port.timeout = 0.1
            port.write_timeout = 2
            port.rtscts = False
            port.dsrdtr = False
            port.dtr = False
            port.rts = False
            port.open()

            print(f"Locating module baud on {args.port}")
            find_control_baud(port)
            print("Resetting A2DP stack")
            off_response = send_command(port, b"AT+A2DPOFF", b"OK+A2DPOFF", 5)
            if b"Mode:Bluetooth Off" not in off_response:
                read_until(port, b"Mode:Bluetooth Off", 20)

            on_response = send_command(port, b"AT+A2DPON", b"OK+A2DPON", 20)
            if b"Mode:A2DP Source" not in on_response:
                read_until(port, b"Mode:A2DP Source", 20)
            print(f"Preparing A2DP connection at {CONTROL_BAUD} baud")
            port.write(b"AT+BAUD6\r\n")
            port.flush()
            time.sleep(0.1)
            port.baudrate = AUDIO_BAUD
            audio_baud_selected = True
            port.reset_input_buffer()

            send_prebuffer(port, audio)

            connect_response = send_command(
                port, b"AT+A2DPCON" + args.address.encode("ascii"), b"OK+A2DPCON", 5)
            if b"A2DP media started" not in connect_response:
                print("Waiting for A2DP media to start...", flush=True)
                read_until(port, b"A2DP media started", 30)

            print(f"Streaming {args.wav} at {AUDIO_BAUD} baud")
            commands = queue.Queue()
            start_volume_input(commands)
            stream_wav(port, audio, commands)
    except (OSError, ValueError, wave.Error, serial.SerialException, TimeoutError) as exc:
        parser.error(str(exc))
    finally:
        if port is not None and port.is_open:
            if audio_baud_selected:
                try:
                    port.write(b"AT+BAUD4\r\n")
                    port.flush()
                    time.sleep(0.1)
                    port.baudrate = CONTROL_BAUD
                except serial.SerialException:
                    pass
            port.close()


if __name__ == "__main__":
    main()
