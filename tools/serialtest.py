import serial

# Setup serial connection (the port may vary depending on your system)
ser = serial.Serial(
    port='/dev/serial1',  # or /dev/serial0 depending on your Pi configuration
    baudrate=115200,      # Make sure this matches the ESP32's baudrate
    timeout=1
)

# Send data
ser.write(b'Hello ESP32\n')

# Read data
response = ser.readline()
print(response.decode())