import serial
import time

PORT = "/dev/ttyAMA0"
BAUD = 115200
NUM_LEDS = 100  # one LED in the strip was wired out after failing, 2026-08-10

speed = 0
print(f"Connecting to ESP32 on {PORT}...")

def send_leds(leds):
    # Join the list into a single 300-character string: one hex nibble per
    # channel (half the precision of a full RRGGBB), which is what halves the
    # DATA frame's wire time and buys the fps back.
    hex_string = "".join(leds)

    # Format and send the command exactly as the ESP32 expects: "DATA <300_hex_chars>\n"
    command = f"DATA {hex_string}\n"
    esp32.write(command.encode('ascii'))


try:
    # Open the UART connection from the Pi to the ESP32
    esp32 = serial.Serial(PORT, BAUD, timeout=1)
    
    pos, direction = 0, 1
    
    print("Running moving green dot test... (Press Ctrl+C to stop)")
    
    while True:
        # Create a list of 100 black LEDs (hex nibbles "000")
        leds = ["000"] * NUM_LEDS

        # Set the current position to Bright Green ("0F0")
        leds[pos] = "0F0"

        send_leds(leds)

        # Update the position of the dot
        pos += direction

        # Reverse direction if it hits either end of the 100 LED array
        if pos == 0 or pos == (NUM_LEDS - 1):
            direction *= -1
        # Adjust this sleep value to change the dot's speed
        time.sleep(0.03*speed)

        if pos == 27:
            leds = ["000"] * NUM_LEDS
            send_leds(leds)
            time.sleep(0.03*22*speed)


        if pos == 51:
            leds = ["000"] * NUM_LEDS
            send_leds(leds)
            time.sleep(0.03*40*speed)


        if pos == 75:
            leds = ["000"] * NUM_LEDS
            send_leds(leds)
            time.sleep(0.03*22*speed)
            


except serial.SerialException as e:
    print(f"Failed to open port {PORT}. Error: {e}")
except KeyboardInterrupt:
    print("\nTest stopped.")
finally:
    if 'esp32' in locals() and esp32.is_open:
        esp32.close()