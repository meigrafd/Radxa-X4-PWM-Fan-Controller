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
- 5V PWM Fan (connected to GPIO 20 for the PWM signal)
- [MicroPython firmware flashed onto the embedded RP2040](https://www.youtube.com/watch?v=rUkpIG_3D9k)
- Python 3 installed on the Host OS

### Prepare Setup
Press the [BOOTSEL](https://docs.radxa.com/en/x/x4) button and when you release it, you will find a USB mass storage device (i.e. RP2040). Check with `lsblk`
```bash
mkdir /mnt/pico && sudo mount /dev/sda1 /mnt/pico
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

# Frequency for the PWM signal in Hertz. Standard for 4-pin PC fans is 25000Hz.
PWM_FREQ = 25000

# GPIO Pin number connected to the fan's PWM control wire.
PWM_PIN = 20

# Timeout in milliseconds to trigger the safety failsafe if no data is received.
FAILSAFE_TIMEOUT_MS = 5000

# 50% duty cycle as 16-bit integer (65535 * 0.5).
FAILSAFE_DUTY = 32768

# Define the fan curve as a list of (temperature, fan_percentage) tuples.
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

# Setup polling object to check for incoming serial data without blocking.
poller = uselect.poll()

# Register the standard input (serial connection) for polling.
poller.register(sys.stdin, uselect.POLLIN)

def get_duty_cycle(temp):
    # Fallback if temperature is completely below the lowest defined curve point.
    if temp <= PWM_FAN_CURVE[0][0]:
        # Return the corresponding lowest duty cycle percentage mapped to 16-bit.
        return int((PWM_FAN_CURVE[0][1] / 100.0) * 65535)

    # Iterate through the curve to find the matching temperature segment.
    for i in range(len(PWM_FAN_CURVE) - 1):
        # Extract lower bound of the current segment.
        temp_low, pct_low = PWM_FAN_CURVE[i]
        # Extract upper bound of the current segment.
        temp_high, pct_high = PWM_FAN_CURVE[i+1]

        # Check if the current temperature falls within this segment.
        if temp_low < temp <= temp_high:
            # Calculate the interpolation ratio between the two temperature points.
            ratio = (temp - temp_low) / (temp_high - temp_low)
            # Calculate the interpolated percentage based on the ratio.
            pct = pct_low + ratio * (pct_high - pct_low)
            # Convert percentage (0-100) to 16-bit duty cycle (0-65535) and return.
            return int((pct / 100.0) * 65535)
            
    # Absolute fallback for extremely high temperatures exceeding the curve limits.
    return 65535

def main():
    # Record the initial time to track incoming data intervals.
    last_update_ticks = time.ticks_ms()
    
    # Loop indefinitely to read serial data and update the fan speed.
    while True:
        # Check if there is data available to read on stdin with a 10ms timeout.
        res = poller.poll(10)
        if res:
            # Read the incoming line and strip whitespace/newlines.
            line = sys.stdin.readline().strip()
            try:
                # Convert the received string to a floating-point temperature value.
                cpu_temp = float(line)
                # Calculate the new duty cycle based on the fan curve interpolation.
                target_duty = get_duty_cycle(cpu_temp)
                # Apply the calculated duty cycle to the PWM pin.
                fan_pwm.duty_u16(target_duty)
                # Update the timestamp of the last successful data reception.
                last_update_ticks = time.ticks_ms()
            except ValueError:
                # Handle cases where the received data is not a valid number.
                pass
        
        # Calculate the time elapsed since the last data packet was received.
        elapsed_time = time.ticks_diff(time.ticks_ms(), last_update_ticks)
        
        # Check if the elapsed time exceeds the defined safety timeout limit.
        if elapsed_time > FAILSAFE_TIMEOUT_MS:
            # Force the fan to maximum speed (100%) to prevent thermal runaway.
            fan_pwm.duty_u16(FAILSAFE_DUTY)
            
        # Pause for a short duration to prevent CPU hogging on the microcontroller.
        time.sleep(0.1)

# Execute the main function when the script runs.
if __name__ == '__main__':
    main()
```
Transfer and activate
```bash
pip3 install mpremote

mpremote connect /dev/ttyACM0 cp fan_control.py :main.py
mpremote connect /dev/ttyACM0 soft-reset
```
Check script running:
```bash
mpremote connect /dev/ttyACM0 repl
```

## Host Script (Linux x86 Python 3)
**host_fan_script.py**
```python
import serial
import time

# The serial port where the RP2040 is mounted. Adjust if necessary (e.g., /dev/ttyACM0).
SERIAL_PORT = '/dev/ttyACM0'

# The baud rate for the serial connection.
BAUD_RATE = 115200

# The file path to read the host CPU thermal zone temperature in Linux.
THERMAL_FILE = '/sys/class/thermal/thermal_zone0/temp'

def get_cpu_temp():
    # Open the system thermal file in read mode.
    with open(THERMAL_FILE, 'r') as f:
        # Read the contents and strip any trailing newlines.
        temp_str = f.read().strip()
        # Convert the millidegree Celsius value to standard Celsius.
        return float(temp_str) / 1000.0

def main():
    # Initialize the serial connection to the RP2040.
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    # Continuous loop to monitor and transmit temperature.
    while True:
        try:
            # Fetch the current CPU temperature.
            temp = get_cpu_temp()
            # Format the temperature as a string with a newline character.
            data = f"{temp}\n"
            # Encode the string to bytes and send it over the serial connection.
            ser.write(data.encode('utf-8'))
            # Pause for 2 seconds before the next reading.
            time.sleep(2)
        except Exception as e:
            # Print any errors encountered during the loop.
            print("Error:", e)
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
ExecStart=/usr/bin/python3 /root/host_fan_script.py
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
