#!/usr/bin/env python
#Add @reboot /home/pi/Projects/Python/showData.py & to crontab to run on boot
import smbus
import time
import dht11
import RPi.GPIO as GPIO
import netifaces as NI
import os

# Define some device parameters
I2C_ADDR  = 0x27 # I2C device address, if any error, change this address to 0x27
LCD_WIDTH = 16   # Maximum characters per line

# Define some device constants
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line

LCD_BACKLIGHT  = 0x08  # On
#LCD_BACKLIGHT = 0x00  # Off

ENABLE = 0b00000100 # Enable bit

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

#Open I2C interface
#bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
bus = smbus.SMBus(1) # Rev 2 Pi uses 1

def get_ip(interface):
	ip = NI.ifaddresses(interface)[2][0]['addr']
	return ip 

def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off 
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = the data
  # mode = 1 for data
  #        0 for command

  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
  bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT

  # High bits
  bus.write_byte(I2C_ADDR, bits_high)
  lcd_toggle_enable(bits_high)

  # Low bits
  bus.write_byte(I2C_ADDR, bits_low)
  lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
  # Toggle enable
  time.sleep(E_DELAY)
  bus.write_byte(I2C_ADDR, (bits | ENABLE))
  time.sleep(E_PULSE)
  bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
  time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display
  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

def main():
	while True:
		# Main program block
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM)       # Use BCM GPIO numbers
		GPIO.setup(17, GPIO.OUT, initial=GPIO.LOW)

		# Initialise display
		lcd_init()

		# Initial greeting
		lcd_string("Hello David!",LCD_LINE_1)
		lcd_string("My name is pi.",LCD_LINE_2)

		#Turn on led 17
		GPIO.output(17, GPIO.HIGH)

		lcd_string("Hello David!",LCD_LINE_1)
		lcd_string("My name is pi.",LCD_LINE_2)
		time.sleep(2)

		lcd_string("eth0 IP: ", LCD_LINE_1)
		lcd_string(get_ip('eth0'), LCD_LINE_2)
		time.sleep(2)

		lcd_string("wlan0 IP: ", LCD_LINE_1)
		lcd_string(get_ip('wlan0'), LCD_LINE_2)
		time.sleep(2)
 
if __name__ == '__main__':

  try:
    main()
  except:
    LCD_BACKLIGHT = 0x00  # Off
    lcd_string("Goodbye... ", LCD_LINE_1)
    time.sleep(1)
    print "\nStopping..."
    pass
  finally:
    lcd_byte(0x01, LCD_CMD)
    GPIO.output(17, GPIO.LOW)
