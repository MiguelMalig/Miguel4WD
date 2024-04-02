Old

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

# LED strip configuration:
LED_COUNT = 8 # Number of LED pixels.
LED_PIN = 18 # GPIO pin connected to the pixels (18 uses PWM!).
LED_FREQ_HZ = 800000 # LED signal frequency in hertz (usually 800khz)
LED_DMA = 10 # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255 # Set to 0 for darkest and 255 for brightest
LED_INVERT = False # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL = 0 # set to '1' for GPIOs 13, 19, 41, 45 or 53


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
            self.adc=Adc()
            self.PWM=Motor()
            self.PWM.setMotorModel(0,0,0,0)

            while True:
                L = self.adc.recvADC(0)
                R = self.adc.recvADC(1)
                
                # Check if there's light
                if L < 2.99 and R < 2.99:
                    self.PWM.setMotorModel(0,0,0,0)
                     # Turn off buzzer
                    self.deactivate_buzzer()
                    # Blink red if no light
                    self.blink_red()
                elif L > 3 or R > 3:
                    # Start rainbow animation and buzzer
                    self.theaterChaseRainbow(strip)
                    self.activate_buzzer()
                    #if light left side
                    if L>R:
                        self.PWM.setMotorModel(-1200,-1200,1400,1400)
                    #if light right side
                    elif R > L :
                         self.PWM.setMotorModel(1400,1400,-1200,-1200)

        except KeyboardInterrupt:
            # On keyboard interrupt, stop the motor and clean up GPIO
            self.motor.setMotorModel(0, 0, 0, 0)
            GPIO.cleanup()

    def activate_buzzer(self):
        GPIO.output(Buzzer_Pin, True)

    def deactivate_buzzer(self):
        GPIO.output(Buzzer_Pin, False)

    def theaterChase(self,strip, color, wait_ms=50, iterations=10):
        ##"""Movie theater light style chaser animation."""
        color=self.LED_TYPR(self.ORDER,color)
        for j in range(iterations):
            for q in range(3):
                for i in range(0,self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i+q, color)
                self.strip.show()
                time.sleep(wait_ms/1000.0)
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i+q, 0)
                    
    def theaterChaseRainbow(self,strip, wait_ms=50):
        # Rainbow movie theater light style chaser animation
        for j in range(256):
            for q in range(3):
                for i in range(0, strip.numPixels(), 3):
                    strip.setPixelColor(i + q, self.wheel((i + j) % 255))
                strip.show()
                time.sleep(wait_ms / 1000.0)
                for i in range(0, strip.numPixels(), 3):
                    strip.setPixelColor(i + q, 0)


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


