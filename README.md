# Radxa X4 - PWM Fan Controller

### Key features
- Dual-architecture setup (x86 Host + RP2040 MCU) to bypass hardware limitations.
- Serial communication to transfer host thermal data to the microcontroller.
- Dynamic, interpolated PWM fan speed adjustment based on live CPU thermals.
- Array-based custom fan curve with linear interpolation for smooth acoustic transitions.
- Red-Teaming security: Integrated watchdog timer forces 50% fan speed upon host communication failure to prevent thermal runaway.
- Non-blocking serial polling to maintain high-frequency PWM stability.
- Watchdog Logic: Constantly monitors the serial input for heartbeat-like updates from the x86 side.

### Required components
- Radxa X4 SBC
- 5V PWM Fan (connected to GPIO 20 for the PWM, GPIO 21 for the Tacho signal. GPIO20 (pin#8) = PWM2A, GPIO21 (pin#10) = PWM2B)
- [MicroPython firmware flashed onto the embedded RP2040](https://www.youtube.com/watch?v=rUkpIG_3D9k) - [Radxa X4 MircoPython](https://docs.radxa.com/en/x/x4/software/micro_python)
- Python 3 installed on the Host OS

## [Wire](https://i.sstatic.net/5ipAk.png) - see [Radxa X4 GPIO Definition](https://docs.radxa.com/en/x/x4/software/gpio?version=v1.110)
- blue :large_blue_circle: PWM → GPIO20
- green :green_circle: Tacho → GPIO21
- black :black_circle: → GND
- yellow :yellow_circle: → VCC

### Prepare Setup
Press the [BOOTSEL](https://docs.radxa.com/en/x/x4) button and when you release it, you will find a USB mass storage device (i.e. RP2040). Check with `lsblk`
```bash
mkdir /mnt/pico
sudo mount /dev/sda1 /mnt/pico
wget -P /mnt/pico/ https://micropython.org/resources/firmware/RPI_PICO-20241129-v1.24.1.uf2
```
When the usb device disappears, the program starts executing.

## RP2040 Script - the brain (MicroPython)
**fan_control.py**
```python
import sys
import machine
import time
import uselect

# Standard PWM frequency for 4-pin PC fans.
PWM_FREQ = 25000

# Radxa X4 RP2040 GPIO numbers.
PWM_PIN = 20
TACH_PIN = 21

# Tachometer characteristics.
TACH_PULSES_PER_REV = 2
RPM_SAMPLE_INTERVAL_MS = 500

# Failsafe.
# 50% duty cycle as 16-bit integer (65535 * 0.5).
FAILSAFE_TIMEOUT_MS = 5000
FAILSAFE_DUTY = 32768

# Fan characteristics.
# Set this to the real maximum RPM from your fan's datasheet.
FAN_MAX_RPM = 5000

# Many 4-pin PWM fans have undefined behaviour below ~20% PWM.
MIN_STABLE_PWM_PCT = 20.0
SPINUP_PWM_PCT = 100.0
SPINUP_TIME_MS = 800

# Closed-loop correction gains.
RPM_KP = 0.02
RPM_KI = 0.004
RPM_INTEGRAL_LIMIT = 4000.0

# Temperature to PWM target curve.
PWM_FAN_CURVE = [
    (30, 0),     # 0% fan speed < 30°C
    (35, 20),    # 20% fan speed between 35°C and 40°C
    (40, 50),    # 50% fan speed between 40°C and 50°C
    (50, 80),    # 80% fan speed between 50°C and 60°C
    (70, 100),   # 100% fan speed > 70°C
]

# Initialize the PWM object on the specified pin.
fan_pwm = machine.PWM(machine.Pin(PWM_PIN))

# Apply the frequency setting to the PWM object.
fan_pwm.freq(PWM_FREQ)

tach_pin = machine.Pin(TACH_PIN, machine.Pin.IN, machine.Pin.PULL_UP)

# Setup polling object to check for incoming serial data without blocking.
poller = uselect.poll()

# Register the standard input (serial connection) for polling.
poller.register(sys.stdin, uselect.POLLIN)

tach_pulse_count = 0
last_pulse_us = 0
current_rpm = 0


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def pct_to_duty_u16(pct):
    pct = clamp(pct, 0.0, 100.0)
    return int((pct / 100.0) * 65535)


def set_pwm_pct(pct):
    fan_pwm.duty_u16(pct_to_duty_u16(pct))


def get_curve_pct(temp):
    if temp <= PWM_FAN_CURVE[0][0]:
        return float(PWM_FAN_CURVE[0][1])

    for i in range(len(PWM_FAN_CURVE) - 1):
        temp_low, pct_low = PWM_FAN_CURVE[i]
        temp_high, pct_high = PWM_FAN_CURVE[i + 1]

        if temp_low < temp <= temp_high:
            ratio = (temp - temp_low) / (temp_high - temp_low)
            return pct_low + ratio * (pct_high - pct_low)

    return 100.0


def target_rpm_from_pct(pct):
    if pct <= 0.0:
        return 0
    return int((pct / 100.0) * FAN_MAX_RPM)


def tach_irq(pin):
    global tach_pulse_count
    global last_pulse_us

    tach_pulse_count += 1
    last_pulse_us = time.ticks_us()


def sample_rpm(now_ms, last_sample_ms):
    global tach_pulse_count
    global current_rpm

    elapsed_ms = time.ticks_diff(now_ms, last_sample_ms)
    if elapsed_ms < RPM_SAMPLE_INTERVAL_MS:
        return current_rpm, last_sample_ms

    irq_state = machine.disable_irq()
    pulses = tach_pulse_count
    tach_pulse_count = 0
    pulse_snapshot_us = last_pulse_us
    machine.enable_irq(irq_state)

    if pulses > 0 and elapsed_ms > 0:
        freq_hz = (pulses * 1000.0) / elapsed_ms
        current_rpm = int((freq_hz * 60.0) / TACH_PULSES_PER_REV)
    else:
        if pulse_snapshot_us == 0:
            current_rpm = 0
        else:
            silent_us = time.ticks_diff(time.ticks_us(), pulse_snapshot_us)
            if silent_us > 1500000:
                current_rpm = 0

    return current_rpm, now_ms


def write_line(text):
    sys.stdout.write(text + "\n")
    if hasattr(sys.stdout, "flush"):
        sys.stdout.flush()


def main():
    tach_pin.irq(trigger=machine.Pin.IRQ_FALLING, handler=tach_irq)

    last_update_ticks = time.ticks_ms()
    last_rpm_sample_ms = last_update_ticks
    last_control_ms = last_update_ticks

    current_temp = None
    integral = 0.0
    spinup_until_ms = last_update_ticks
    fan_running = False

    set_pwm_pct(0.0)

    while True:
        now_ms = time.ticks_ms()

        res = poller.poll(10)
        if res:
            line = sys.stdin.readline().strip()

            if line == "rpm?":
                write_line(str(current_rpm))
            elif line == "status?":
                temp_text = "null" if current_temp is None else str(current_temp)
                write_line('{"rpm":%d,"temp":%s}' % (current_rpm, temp_text))
            elif line.startswith("T:"):
                try:
                    current_temp = float(line[2:])
                    last_update_ticks = now_ms
                except ValueError:
                    pass
            else:
                try:
                    current_temp = float(line)
                    last_update_ticks = now_ms
                except ValueError:
                    pass

        current_rpm, last_rpm_sample_ms = sample_rpm(now_ms, last_rpm_sample_ms)

        elapsed_since_update = time.ticks_diff(now_ms, last_update_ticks)
        if elapsed_since_update > FAILSAFE_TIMEOUT_MS:
            fan_pwm.duty_u16(FAILSAFE_DUTY)
            integral = 0.0
            fan_running = True
            time.sleep(0.05)
            continue

        if current_temp is None:
            time.sleep(0.05)
            continue

        base_pct = get_curve_pct(current_temp)
        target_rpm = target_rpm_from_pct(base_pct)

        control_elapsed_ms = time.ticks_diff(now_ms, last_control_ms)
        if control_elapsed_ms <= 0:
            control_elapsed_ms = 1
        dt_s = control_elapsed_ms / 1000.0
        last_control_ms = now_ms

        if target_rpm <= 0:
            desired_pct = 0.0
            integral = 0.0
            fan_running = False
        else:
            if base_pct >= MIN_STABLE_PWM_PCT and not fan_running:
                spinup_until_ms = time.ticks_add(now_ms, SPINUP_TIME_MS)
                fan_running = True

            error = target_rpm - current_rpm
            integral += error * dt_s
            integral = clamp(integral, -RPM_INTEGRAL_LIMIT, RPM_INTEGRAL_LIMIT)

            correction_pct = (RPM_KP * error) + (RPM_KI * integral)
            desired_pct = base_pct + correction_pct

            if base_pct >= MIN_STABLE_PWM_PCT and desired_pct < MIN_STABLE_PWM_PCT:
                desired_pct = MIN_STABLE_PWM_PCT

            desired_pct = clamp(desired_pct, 0.0, 100.0)

            if time.ticks_diff(spinup_until_ms, now_ms) > 0:
                desired_pct = SPINUP_PWM_PCT

        set_pwm_pct(desired_pct)
        time.sleep(0.05)


if __name__ == '__main__':
    main()
```
Transfer and activate
```bash
pip3 install mpremote

ls -la /dev/ttyACM*

mpremote connect /dev/ttyACM0 cp fan_control.py :main.py
mpremote connect /dev/ttyACM0 soft-reset
```
Check script running:
```bash
mpremote connect /dev/ttyACM0 repl
```

## Host Script (Linux x86 Python 3)
**host_fan_script.py** save as `/srv/host_fan_script.py`
```python
import json
import os
import serial
import time

# The serial port where the RP2040 is mounted. Adjust if necessary (e.g., /dev/ttyACM0).
SERIAL_PORT = '/dev/ttyACM0'

# The baud rate for the serial connection.
BAUD_RATE = 115200

# The file path to read the host CPU thermal zone temperature in Linux.
THERMAL_FILE = '/sys/class/thermal/thermal_zone1/temp'

# The file path used to expose the latest fan status to other local processes.
STATUS_FILE = '/run/fan-status.json'

# Interval between host temperature updates.
LOOP_INTERVAL_SEC = 2

# Timeout for waiting on a status reply from the RP2040.
STATUS_TIMEOUT_SEC = 1.0


def get_cpu_temp():
    # Open the system thermal file in read mode.
    with open(THERMAL_FILE, 'r') as f:
        # Read the contents and strip any trailing newlines.
        temp_str = f.read().strip()
        # Convert the millidegree Celsius value to standard Celsius.
        return float(temp_str) / 1000.0


def request_status(ser):
    # Clear any stale input before issuing a fresh status request.
    ser.reset_input_buffer()
    # Send the status request command to the RP2040.
    ser.write(b'status?\n')
    ser.flush()

    # Wait up to the configured timeout for a JSON reply line.
    deadline = time.monotonic() + STATUS_TIMEOUT_SEC
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue

        # Decode the received bytes into a string.
        line = raw.decode('utf-8', errors='ignore').strip()
        if not line:
            continue

        # Accept only the expected JSON object format.
        if line.startswith('{') and line.endswith('}'):
            return json.loads(line)

    return None


def write_status_file(status):
    # Write the latest status atomically for external readers.
    tmp_file = STATUS_FILE + '.tmp'
    with open(tmp_file, 'w', encoding='utf-8') as f:
        json.dump(status, f, separators=(',', ':'))
    os.replace(tmp_file, STATUS_FILE)


def main():
    # Ensure the target directory for the status file exists.
    os.makedirs(os.path.dirname(STATUS_FILE), exist_ok=True)

    # Initialize the serial connection to the RP2040.
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.2)
    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Continuous loop to monitor and transmit temperature.
    while True:
        try:
            # Fetch the current CPU temperature.
            temp = get_cpu_temp()
            # Format the temperature with a prefix and newline character.
            data = f"T:{temp:.1f}\n"
            # Encode the string to bytes and send it over the serial connection.
            ser.write(data.encode('utf-8'))
            ser.flush()

            # Request the current RPM and board-side temperature from the RP2040.
            status = request_status(ser)

            if status is None:
                status = {
                    "rpm": None,
                    "temp": temp,
                    "host_temp": temp,
                    "error": "no_status_reply"
                }
            else:
                status["host_temp"] = temp

            # Persist the latest status for local consumers.
            write_status_file(status)

            # Print the current host temperature and fan RPM.
            print(
                f"CPU {temp:.1f} °C | "
                f"Fan {status.get('rpm')} RPM | "
                f"RP2040 Temp {status.get('temp')}"
            )

            # Pause for 2 seconds before the next reading.
            time.sleep(LOOP_INTERVAL_SEC)
        except Exception as e:
            # Print any errors encountered during the loop.
            print("Error:", e)
            # Write the error state for local consumers.
            write_status_file({
                "rpm": None,
                "temp": None,
                "host_temp": None,
                "error": str(e)
            })
            # Wait briefly before retrying in case of an error.
            time.sleep(2)


# Execute the main function when the script is run directly.
if __name__ == '__main__':
    main()
```

### systemd
```bash
sudo cat >/etc/systemd/system/fan-control.service <<'EOF'
[Unit]
Description=Radxa X4 Fan Control Host Bridge
After=multi-user.target

[Service]
ExecStart=/usr/bin/python3 /srv/host_fan_script.py
Restart=always
RestartSec=5
User=root
WorkingDirectory=/root

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable fan-control.service
sudo systemctl start fan-control.service

sudo systemctl status fan-control.service
journalctl -u fan-control.service -f
```

### Get current state
```bash
cat /run/fan-status.json
```
