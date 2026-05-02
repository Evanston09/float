import argparse
import csv
import socket
import time
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

DEFAULT_PORT = 80
CONNECT_TIMEOUT_SECONDS = 10
RESPONSE_TIMEOUT_SECONDS = 10
RETRY_DELAY_SECONDS = 5

MISSION_CSV = Path("mission_data.csv")
DEPTH_CSV = Path("depth_record_data.csv")

DEFAULT_CONFIG = {
    "deep": 2.5,
    "shallow": 0.40,
    "surface": 0.05,
    "hold": 30.0,
    "profiles": 2,
    "sink_pulse": 5.0,
    "deep_neutralize": 5.0,
    "rise_pulse": 5.0,
    "shallow_neutralize": 5.0,
    "return_surface": 5.0,
    "deep_tol": 0.10,
    "shallow_tol": 0.05,
    "surface_tol": 0.05,
    "min_safe": 0.30,
    "log": 1.0,
    "threshold_timeout": 180.0,
}

DEFAULT_SERVO_CONFIG = {
    "down": 30,
    "neutral": 90,
    "up": 150,
}


class LineSocket:
    def __init__(self, sock):
        self.sock = sock
        self.buffer = b""

    def readline(self, timeout):
        self.sock.settimeout(timeout)
        while b"\n" not in self.buffer:
            try:
                chunk = self.sock.recv(256)
            except socket.timeout as exc:
                raise TimeoutError("timed out waiting for a line") from exc
            if not chunk:
                return ""
            self.buffer += chunk

        line, self.buffer = self.buffer.split(b"\n", 1)
        return line.decode("utf-8", errors="replace").strip()

def connect(host, port):
    return socket.create_connection((host, port), timeout=CONNECT_TIMEOUT_SECONDS)


def read_response_lines(client):
    lines = []
    while True:
        line = client.readline(RESPONSE_TIMEOUT_SECONDS)
        if not line:
            break
        if line == "FLOAT READY":
            continue
        lines.append(line)
        if line.startswith(("OK", "ERROR")):
            break
        if line == "END_DATA":
            break
    return lines


def send_command(host, port, command):
    with connect(host, port) as sock:
        client = LineSocket(sock)
        sock.sendall(f"{command}\n".encode("utf-8"))
        return read_response_lines(client)


def print_response(lines):
    for line in lines:
        print(line)


def parse_status(lines):
    for line in lines:
        if not line.startswith("OK STATUS"):
            continue

        fields = {}
        for part in line.split():
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            fields[key] = value
        return fields

    raise RuntimeError("No STATUS response.")


def require_ok(lines):
    print_response(lines)
    if not lines or not lines[-1].startswith("OK"):
        raise RuntimeError("command failed")


def prompt_float(prompt, default):
    value = input(f"{prompt} [{default}]: ").strip()
    if not value:
        return default
    return float(value)


def prompt_int(prompt, default):
    value = input(f"{prompt} [{default}]: ").strip()
    if not value:
        return default
    return int(value)


def configure_mission(host, port, config):
    print("Enter mission settings. Press Enter to keep a default.")
    config["deep"] = prompt_float("Deep target meters", config["deep"])
    config["shallow"] = prompt_float("Shallow target meters", config["shallow"])
    config["surface"] = prompt_float("Final surface target meters", config["surface"])
    config["hold"] = prompt_float("Hold time seconds", config["hold"])
    config["profiles"] = prompt_int("Profile count", config["profiles"])
    config["sink_pulse"] = prompt_float("Sink pulse seconds (UP=150, adds water)", config["sink_pulse"])
    config["deep_neutralize"] = prompt_float(
        "Deep neutralize pulse seconds (DOWN=30, removes water)",
        config["deep_neutralize"],
    )
    config["rise_pulse"] = prompt_float("Rise pulse seconds (DOWN=30, removes water)", config["rise_pulse"])
    config["shallow_neutralize"] = prompt_float(
        "Shallow neutralize pulse seconds (UP=150, adds water)",
        config["shallow_neutralize"],
    )
    config["return_surface"] = prompt_float(
        "Final return surface pulse seconds (DOWN=30, removes water)",
        config["return_surface"],
    )
    config["deep_tol"] = prompt_float("Deep tolerance meters", config["deep_tol"])
    config["shallow_tol"] = prompt_float("Shallow tolerance meters", config["shallow_tol"])
    config["surface_tol"] = prompt_float("Surface tolerance meters", config["surface_tol"])
    config["min_safe"] = prompt_float("Near-surface safety depth meters", config["min_safe"])
    config["log"] = prompt_float("Log interval seconds", config["log"])
    config["threshold_timeout"] = prompt_float("Max seconds waiting for a depth threshold", config["threshold_timeout"])

    parts = [f"{key}={value}" for key, value in config.items()]
    require_ok(send_command(host, port, "CONFIG " + " ".join(parts)))


def configure_servos(host, port, servo_config):
    print("Enter servo angles. Press Enter to keep a default.")
    servo_config["down"] = prompt_int("Down angle", servo_config["down"])
    servo_config["neutral"] = prompt_int("Neutral angle", servo_config["neutral"])
    servo_config["up"] = prompt_int("Up angle", servo_config["up"])

    parts = [f"{key}={value}" for key, value in servo_config.items()]
    require_ok(send_command(host, port, "SERVO_CONFIG " + " ".join(parts)))


def manual_control(host, port):
    print(
        "Manual commands: up <seconds> = sink/add water, down <seconds> = rise/remove water, "
        "one <1|2> <down|up|neutral> <seconds>, neutral, abort, back"
    )
    while True:
        raw = input("manual> ").strip().lower()
        if raw in {"back", "exit", "quit"}:
            return
        if raw == "neutral":
            require_ok(send_command(host, port, "NEUTRAL"))
            continue
        if raw == "abort":
            require_ok(send_command(host, port, "ABORT"))
            continue

        parts = raw.split()
        if len(parts) == 4 and parts[0] == "one":
            servo_number, direction, seconds = parts[1], parts[2], parts[3]
            if servo_number not in {"1", "2"} or direction not in {"down", "up", "neutral"}:
                print("Use: one 1 down 0.25 or one 2 up 0.5")
                continue
            try:
                float(seconds)
            except ValueError:
                print("Seconds must be a number.")
                continue

            require_ok(send_command(host, port, f"MANUAL_ONE {servo_number} {direction.upper()} {seconds}"))
            continue

        if len(parts) != 2 or parts[0] not in {"down", "up"}:
            print("Use: down 3, up 2, one 1 down 0.25, neutral, abort, back")
            continue

        direction, seconds = parts
        try:
            float(seconds)
        except ValueError:
            print("Seconds must be a number.")
            continue

        require_ok(send_command(host, port, f"MANUAL {direction.upper()} {seconds}"))


def wait_for_reconnect_and_download(host, port, command):
    attempt = 1
    while True:
        try:
            print(f"Download attempt {attempt}...")
            lines = send_command(host, port, command)
            if lines and lines[0] in {"MISSION_DATA", "DEPTH_DATA"}:
                return lines
            print_response(lines)
        except (OSError, TimeoutError, RuntimeError) as exc:
            print(f"Not ready/reachable yet: {exc}")

        time.sleep(RETRY_DELAY_SECONDS)
        attempt += 1


def wait_for_mission_complete(host, port):
    attempt = 1
    last_phase = None
    last_report_time = 0.0

    while True:
        try:
            fields = parse_status(send_command(host, port, "STATUS"))
            mode = fields.get("mode", "UNKNOWN")
            phase = fields.get("phase", "UNKNOWN")
            samples = fields.get("samples", "?")
            mission_complete = fields.get("mission_complete", "0")
            now = time.monotonic()

            if phase != last_phase or now - last_report_time >= 30:
                print(
                    f"Status: mode={mode} phase={phase} "
                    f"mission_complete={mission_complete} samples={samples}"
                )
                last_phase = phase
                last_report_time = now

            if mission_complete == "1":
                return
        except (OSError, TimeoutError, RuntimeError) as exc:
            print(f"Mission status attempt {attempt}: not reachable/ready yet: {exc}")

        time.sleep(RETRY_DELAY_SECONDS)
        attempt += 1


def parse_log(lines):
    if len(lines) < 3:
        raise RuntimeError("No data returned.")

    label = lines[0]
    header = lines[1].split(",")
    data_lines = [line for line in lines[2:] if line != "END_DATA"]
    rows = []

    for row in csv.reader(data_lines):
        if len(row) != len(header):
            continue
        rows.append(dict(zip(header, row)))

    if not rows:
        raise RuntimeError(f"{label} contained no valid rows.")

    df = pd.DataFrame(rows)
    for column in ("time", "depth"):
        df[column] = pd.to_numeric(df[column], errors="coerce")
    for column in ("control", "servo"):
        df[column] = pd.to_numeric(df[column], errors="coerce").astype("Int64")
    return label, df


def save_and_plot(lines, output_path):
    label, df = parse_log(lines)
    df.to_csv(output_path, index=False)
    print(f"Saved {len(df)} {label} samples to {output_path}")

    plt.plot(df["time"], df["depth"])
    plt.xlabel("Time (s)")
    plt.ylabel("Depth (m)")
    plt.title(label.replace("_", " ").title())
    plt.gca().invert_yaxis()
    plt.grid()
    plt.show()


def start_depth_record(host, port):
    require_ok(send_command(host, port, "START_DEPTH_RECORD"))
    print("Depth recording started. Recover/reconnect later, then choose stop/download.")


def stop_and_download_depth(host, port):
    try:
        require_ok(send_command(host, port, "STOP_DEPTH_RECORD"))
    except Exception as exc:
        print(f"Stop command did not complete: {exc}")
    lines = wait_for_reconnect_and_download(host, port, "GET_DEPTH_DATA")
    save_and_plot(lines, DEPTH_CSV)


def start_mission(host, port):
    input("Press Enter to send START_MISSION...")
    require_ok(send_command(host, port, "START_MISSION"))
    print("Mission started. Polling STATUS until mission_complete=1 before downloading.")
    wait_for_mission_complete(host, port)
    print("Mission complete. Downloading mission data...")
    lines = wait_for_reconnect_and_download(host, port, "GET_MISSION_DATA")
    save_and_plot(lines, MISSION_CSV)


def download_mission(host, port):
    lines = wait_for_reconnect_and_download(host, port, "GET_MISSION_DATA")
    save_and_plot(lines, MISSION_CSV)


def menu(host, port):
    config = DEFAULT_CONFIG.copy()
    servo_config = DEFAULT_SERVO_CONFIG.copy()

    actions = {
        "1": ("Ping/status", lambda: print_response(send_command(host, port, "STATUS"))),
        "2": ("Configure mission", lambda: configure_mission(host, port, config)),
        "3": ("Configure servo angles", lambda: configure_servos(host, port, servo_config)),
        "4": ("Zero depth", lambda: require_ok(send_command(host, port, "ZERO_DEPTH"))),
        "5": ("Start depth recording", lambda: start_depth_record(host, port)),
        "6": ("Stop/download depth recording", lambda: stop_and_download_depth(host, port)),
        "7": ("Manual timed control", lambda: manual_control(host, port)),
        "8": ("Start mission and auto-download", lambda: start_mission(host, port)),
        "9": ("Download mission data", lambda: download_mission(host, port)),
        "10": ("Abort/neutral", lambda: require_ok(send_command(host, port, "ABORT"))),
    }

    while True:
        print("\nMaster Float Station")
        for key, (label, _) in actions.items():
            print(f"{key}. {label}")
        print("0. Exit")

        choice = input("> ").strip()
        if choice == "0":
            return
        if choice not in actions:
            print("Unknown selection.")
            continue

        try:
            actions[choice][1]()
        except Exception as exc:
            print(f"Error: {exc}")


def parse_args():
    parser = argparse.ArgumentParser(description="Master topside station for the vertical profiling float.")
    parser.add_argument("host", help="Float IP address shown by the Arduino Serial Monitor")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="Float TCP port")
    return parser.parse_args()


def main():
    args = parse_args()
    menu(args.host, args.port)


if __name__ == "__main__":
    main()
