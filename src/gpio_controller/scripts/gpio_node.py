#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.srv import SetLedPattern
import RPi.GPIO as GPIO
import time
import threading

# ros2 service call /change_led_pattern interfaces/srv/SetLedPattern "{mode: 'manual'}"

# --- Configuration --- 
LED_PIN = 26  # BCM Pin number for the LED
# Flashing patterns: [ON_ms, OFF_ms, ON_ms, OFF_ms, ...]
PATTERNS = {
    "manual": [100, 100],              # Simple blink (0.1s on, 0.1s off)
    "autonomous": [500, 200, 100, 200]     # More complex pattern
}
# --- End Configuration ---

class LedControllerService(Node):
    def __init__(self):
        super().__init__('gpio_controller_service')
        self.srv = self.create_service(SetLedPattern, 'change_led_pattern', self.handle_set_pattern_request)

        # GPIO Setup
        self.led_pin = LED_PIN
        GPIO.setwarnings(False) # Disable warnings
        GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
        GPIO.setup(self.led_pin, GPIO.OUT)
        GPIO.output(self.led_pin, GPIO.LOW) # Start with LED off

        # Threading control
        self.current_mode = "off"
        self.flashing_thread = None
        self.stop_event = threading.Event()

        self.get_logger().info(f"LED Controller Service started. Listening on 'change_led_pattern'. Using GPIO pin {self.led_pin}.")
        self.get_logger().info(f"Available modes: {list(PATTERNS.keys()) + ['off']}")


    def handle_set_pattern_request(self, request, response):
        """Callback function for the service request."""
        requested_mode = request.mode.lower() # Make case-insensitive
        self.get_logger().info(f"Received request to change mode to: {requested_mode}")

        # --- Stop existing flashing thread if running ---
        if self.flashing_thread is not None and self.flashing_thread.is_alive():
            self.get_logger().debug("Stopping previous flashing thread...")
            self.stop_event.set() # Signal the thread to stop
            self.flashing_thread.join(timeout=1.0) # Wait briefly for it to finish
            if self.flashing_thread.is_alive():
                self.get_logger().warning("Flashing thread did not stop gracefully.")
            self.flashing_thread = None # Clear the thread reference
        # Ensure LED is off before potentially starting a new pattern or staying off
        GPIO.output(self.led_pin, GPIO.LOW)
        self.stop_event.clear() # Reset event for potential reuse
        # --- Thread stopped ---

        if requested_mode == "off":
            self.current_mode = "off"
            # GPIO pin already set low above
            response.success = True
            response.message = "LED turned off."
            self.get_logger().info("Mode changed to 'off'.")

        elif requested_mode in PATTERNS:
            self.current_mode = requested_mode
            pattern = PATTERNS[requested_mode]
            # Start a new flashing thread
            self.flashing_thread = threading.Thread(target=self.flash_led, args=(pattern, self.stop_event), daemon=True)
            self.flashing_thread.start()
            response.success = True
            response.message = f"LED pattern changed to '{requested_mode}'."
            self.get_logger().info(f"Mode changed to '{requested_mode}'. Flashing thread started.")

        else:
            response.success = False
            response.message = f"Invalid mode '{request.mode}'. Available modes: {list(PATTERNS.keys()) + ['off']}"
            self.get_logger().warning(f"Invalid mode request: {request.mode}")

        return response

    def flash_led(self, pattern, stop_event):
        """Runs in a separate thread to flash the LED according to the pattern."""
        self.get_logger().debug(f"Flashing thread started for pattern: {pattern}")
        pattern_index = 0
        is_on = False # Track LED state
        while not stop_event.is_set():
            try:
                duration_ms = pattern[pattern_index % len(pattern)]
                duration_sec = duration_ms / 1000.0

                # Even indices are ON times, Odd indices are OFF times
                if (pattern_index % 2) == 0: # Turn ON
                    is_on = True
                    GPIO.output(self.led_pin, GPIO.HIGH)
                    self.get_logger().debug(f"LED ON for {duration_sec:.3f}s")
                else: # Turn OFF
                    is_on = False
                    GPIO.output(self.led_pin, GPIO.LOW)
                    self.get_logger().debug(f"LED OFF for {duration_sec:.3f}s")

                # Wait for the specified duration, checking stop_event frequently
                wait_interval = 0.01 # Check every 10ms
                end_time = time.monotonic() + duration_sec
                while time.monotonic() < end_time:
                    if stop_event.wait(timeout=wait_interval): # Returns True if event is set
                         self.get_logger().debug("Stop event received during wait.")
                         raise InterruptedError # Break out of inner loop
                    # Check again in case wait timed out just as event was set
                    if stop_event.is_set():
                         self.get_logger().debug("Stop event received after wait timeout check.")
                         raise InterruptedError

                pattern_index += 1

            except InterruptedError:
                self.get_logger().debug("Flashing loop interrupted by stop event.")
                break # Exit the while loop cleanly
            except Exception as e:
                self.get_logger().error(f"Error in flashing thread: {e}")
                break # Exit on other errors too

        # Ensure LED is turned off when the thread exits
        if is_on:
            GPIO.output(self.led_pin, GPIO.LOW)
        self.get_logger().debug("Flashing thread finished.")


    def cleanup(self):
        """Properly clean up GPIO resources."""
        self.get_logger().info("Cleaning up GPIO...")
        if self.flashing_thread is not None and self.flashing_thread.is_alive():
            self.stop_event.set()
            self.flashing_thread.join(timeout=1.0)
        GPIO.cleanup(self.led_pin) # Clean up only the pin we used
        self.get_logger().info("GPIO cleanup complete.")


def main(args=None):
    rclpy.init(args=args)
    led_service_node = LedControllerService()
    try:
        rclpy.spin(led_service_node)
    except KeyboardInterrupt:
        led_service_node.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        # Clean up resources before shutting down
        led_service_node.cleanup()
        led_service_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()