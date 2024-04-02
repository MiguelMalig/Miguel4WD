import time
import RPi.GPIO as GPIO
from rpi_ws281x import *

# Import Motor and ADC classes from respective modules
from Motor import Motor
from ADC import Adc

# Set up GPIO for buzzer
GPIO.setwarnings(False)
Buzzer_Pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(Buzzer_Pin, GPIO.OUT)

# LED strip configuration
LED_COUNT = 8
LED_PIN = 18
LED_FREQ_HZ = 800000
LED_DMA = 10
LED_BRIGHTNESS = 255
LED_INVERT = False
LED_CHANNEL = 0

# Initialize NeoPixel object
strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
strip.begin()

class Light:
    def __init__(self):
        self.adc = Adc()
        self.motor = Motor()
        self.motor.setMotorModel(0, 0, 0, 0)

    def run(self):
        try:
            while True:
                L = self.adc.recvADC(0)
                R = self.adc.recvADC(1)
                
                # Check if there's light
                if L < 2.99 and R < 2.99:
                    # Start rainbow animation and buzzer
                    self.rainbow_animation()
                    self.activate_buzzer()
                else:
                    # Turn off buzzer
                    self.deactivate_buzzer()

                    # Blink red if no light
                    self.blink_red()

        except KeyboardInterrupt:
            # On keyboard interrupt, stop the motor
            self.motor.setMotorModel(0, 0, 0, 0)

    def activate_buzzer(self):
        GPIO.output(Buzzer_Pin, True)

    def deactivate_buzzer(self):
        GPIO.output(Buzzer_Pin, False)

    def rainbow_animation(self):
        for j in range(256):
            for i in range(strip.numPixels()):
                strip.setPixelColor(i, self.wheel((i + j) & 255))
            strip.show()
            time.sleep(0.02)

    def wheel(self, pos):
        if pos < 0 or pos > 255:
            r = g = b = 0
        elif pos < 85:
            r = pos * 3
            g = 255 - pos * 3
            b = 0
        elif pos < 170:
            pos -= 85
            r = 255 - pos * 3
            g = 0
            b = pos * 3
        else:
            pos -= 170
            r = 0
            g = pos * 3
            b = 255 - pos * 3
        return Color(r, g, b)

    def blink_red(self):
        for _ in range(5):
            self.colorWipe(strip, Color(255, 0, 0))  # Red wipe
            self.colorWipe(strip, Color(0, 0, 0), 10)  # Clear LEDs
            time.sleep(0.5)

    def colorWipe(self, strip, color, wait_ms=50):
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, color)
            strip.show()
            time.sleep(wait_ms / 1000.0)

if __name__ == '__main__':
    print('Program is starting ... ')
    led_Car = Light()
    led_Car.run()
