#!/usr/bin/env python3
import datetime
import os
import random
import requests
import time
import logging
import argparse
import subprocess
import serial
from threading import Thread
import json
import uuid
import math

import board
from prometheus_client import start_http_server, Gauge, Histogram
import SafecastPy
# import notecard.notecard as notecard
from periphery import Serial

from bme280 import BME280
from enviroplus import gas
from enviroplus.noise import Noise
from pms5003 import PMS5003, ReadTimeoutError as pmsReadTimeoutError, SerialTimeoutError as pmsSerialTimeoutError

import colorsys
import ST7735
from PIL import Image, ImageDraw, ImageFont
from fonts.ttf import RobotoMedium as UserFont
from pms5003 import PMS5003
from adafruit_lc709203f import LC709203F, PackSize
import paho.mqtt.client as mqtt

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus

try:
    # Transitional fix for breaking change in LTR559
    from ltr559 import LTR559
    ltr559 = LTR559()
except ImportError:
    import ltr559

logging.basicConfig(
    format='%(asctime)s.%(msecs)03d %(levelname)-8s %(message)s',
    level=logging.INFO,
    handlers=[logging.FileHandler("enviroplus_exporter.log"),
              logging.StreamHandler()],
    datefmt='%Y-%m-%d %H:%M:%S')

logging.info("""enviroplus_exporter.py - Expose readings from the Enviro+ sensor by Pimoroni in Prometheus format

Press Ctrl+C to exit!

""")

DEBUG = os.getenv('DEBUG', 'false') == 'true'

RBPI_SERIAL = ""

bus = SMBus(1)
bme280 = BME280(i2c_dev=bus)
noise = Noise()

# Initialize display
st7735 = ST7735.ST7735(
    port=0,
    cs=1,
    dc=9,
    backlight=12,
    rotation=270,
    spi_speed_hz=10000000
)

# Initialize display
st7735.begin()

WIDTH = st7735.width
HEIGHT = st7735.height

# Set up canvas and font
img = Image.new('RGB', (WIDTH, HEIGHT), color=(0, 0, 0))
draw = ImageDraw.Draw(img)

# Define your own warning limits
# The limits definition follows the order of the variables array
# Example limits explanation for temperature:
# [4,18,28,35] means
# [-273.15 .. 4] -> Dangerously Low
# (4 .. 18]      -> Low
# (18 .. 28]     -> Normal
# (28 .. 35]     -> High
# (35 .. MAX]    -> Dangerously High
# DISCLAIMER: The limits provided here are just examples and come
# with NO WARRANTY. The authors of this example code claim
# NO RESPONSIBILITY if reliance on the following values or this
# code in general leads to ANY DAMAGES or DEATH.



try:
    pms5003 = PMS5003()
except serial.serialutil.SerialException:
    logging.warning("Failed to initialise PMS5003.")

battery_sensor = False
try:
    sensor = LC709203F(board.I2C())
    battery_sensor = True
except ValueError:
    pass

PREFIX = os.getenv('PREFIX', 'enviroplus_')

TEMPERATURE = Gauge(PREFIX+'temperature','Temperature measured (*C)', ['serial'])
PRESSURE = Gauge(PREFIX+'pressure','Pressure measured (hPa)', ['serial'])
HUMIDITY = Gauge(PREFIX+'humidity','Relative humidity measured (%)', ['serial'])
OXIDISING = Gauge(PREFIX+'oxidising','Mostly nitrogen dioxide but could include NO and Hydrogen (Ohms)', ['serial'])
REDUCING = Gauge(PREFIX+'reducing', 'Mostly carbon monoxide but could include H2S, Ammonia, Ethanol, Hydrogen, Methane, Propane, Iso-butane (Ohms)', ['serial'])
NH3 = Gauge(PREFIX+'NH3', 'mostly Ammonia but could also include Hydrogen, Ethanol, Propane, Iso-butane (Ohms)', ['serial'])
OXIDISING_PPM = Gauge(PREFIX+'oxidising_ppm','Mostly nitrogen dioxide but could include NO and Hydrogen (ppm)', ['serial'])
REDUCING_PPM = Gauge(PREFIX+'reducing_ppm', 'Mostly carbon monoxide but could include H2S, Ammonia, Ethanol, Hydrogen, Methane, Propane, Iso-butane (ppm)', ['serial'])
NH3_PPM = Gauge(PREFIX+'NH3_PPM', 'mostly Ammonia but could also include Hydrogen, Ethanol, Propane, Iso-butane (ppm)', ['serial'])
LUX = Gauge(PREFIX+'lux', 'current ambient light level (lux)', ['serial'])
PROXIMITY = Gauge(PREFIX+'proximity', 'proximity, with larger numbers being closer proximity and vice versa', ['serial'])
PM1 = Gauge(PREFIX+'PM1', 'Particulate Matter of diameter less than 1 micron. Measured in micrograms per cubic metre (ug/m3)', ['serial'])
PM25 = Gauge(PREFIX+'PM25', 'Particulate Matter of diameter less than 2.5 microns. Measured in micrograms per cubic metre (ug/m3)', ['serial'])
PM10 = Gauge(PREFIX+'PM10', 'Particulate Matter of diameter less than 10 microns. Measured in micrograms per cubic metre (ug/m3)', ['serial'])
CPU_TEMPERATURE = Gauge(PREFIX+'cpu_temperature','CPU temperature measured (*C)', ['serial'])
BATTERY_VOLTAGE = Gauge(PREFIX+'battery_voltage','Voltage of the battery (Volts)', ['serial'])
BATTERY_PERCENTAGE = Gauge(PREFIX+'battery_percentage','Percentage of the battery remaining (%)', ['serial'])

OXIDISING_HIST = Histogram(PREFIX+'oxidising_measurements', 'Histogram of oxidising measurements', ['serial'], buckets=(0, 10000, 15000, 20000, 25000, 30000, 35000, 40000, 45000, 50000, 55000, 60000, 65000, 70000, 75000, 80000, 85000, 90000, 100000))
REDUCING_HIST = Histogram(PREFIX+'reducing_measurements', 'Histogram of reducing measurements', ['serial'], buckets=(0, 100000, 200000, 300000, 400000, 500000, 600000, 700000, 800000, 900000, 1000000, 1100000, 1200000, 1300000, 1400000, 1500000))
NH3_HIST = Histogram(PREFIX+'nh3_measurements', 'Histogram of nh3 measurements', ['serial'], buckets=(0, 10000, 110000, 210000, 310000, 410000, 510000, 610000, 710000, 810000, 910000, 1010000, 1110000, 1210000, 1310000, 1410000, 1510000, 1610000, 1710000, 1810000, 1910000, 2000000))

OXIDISING_PPM_HIST = Histogram(PREFIX+'oxidising_ppm_measurements', 'Histogram of oxidising ppm measurements', ['serial'], buckets=(0, 10000, 15000, 20000, 25000, 30000, 35000, 40000, 45000, 50000, 55000, 60000, 65000, 70000, 75000, 80000, 85000, 90000, 100000))
REDUCING_PPM_HIST = Histogram(PREFIX+'reducing_ppm_measurements', 'Histogram of reducing ppm measurements', ['serial'], buckets=(0, 100000, 200000, 300000, 400000, 500000, 600000, 700000, 800000, 900000, 1000000, 1100000, 1200000, 1300000, 1400000, 1500000))
NH3_PPM_HIST = Histogram(PREFIX+'nh3_ppm_measurements', 'Histogram of nh3 ppm measurements', ['serial'], buckets=(0, 10000, 110000, 210000, 310000, 410000, 510000, 610000, 710000, 810000, 910000, 1010000, 1110000, 1210000, 1310000, 1410000, 1510000, 1610000, 1710000, 1810000, 1910000, 2000000))

PM1_HIST = Histogram(PREFIX+'pm1_measurements', 'Histogram of Particulate Matter of diameter less than 1 micron measurements', ['serial'], buckets=(0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100))
PM25_HIST = Histogram(PREFIX+'pm25_measurements', 'Histogram of Particulate Matter of diameter less than 2.5 micron measurements', ['serial'], buckets=(0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100))
PM10_HIST = Histogram(PREFIX+'pm10_measurements', 'Histogram of Particulate Matter of diameter less than 10 micron measurements', ['serial'], buckets=(0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100))

NOISE_PROFILE_LOW_FREQ = Gauge(PREFIX+'noise_profile_low_freq', 'Noise profile of low frequency noise (db)', ['serial'])
NOISE_PROFILE_MID_FREQ = Gauge(PREFIX+'noise_profile_mid_freq', 'Noise profile of mid frequency noise (db)', ['serial'])
NOISE_PROFILE_HIGH_FREQ = Gauge(PREFIX+'noise_profile_high_freq', 'Noise profile of high frequency noise (db)', ['serial'])
NOISE_PROFILE_AMP = Gauge(PREFIX+'noise_profile_amp', 'Noise profile of amplitude (db)', ['serial'])

# delay between each write to lcd
WRITE_TO_LCD_TIME = int(os.getenv('WRITE_TO_LCD_TIME', '3'))

# Sometimes the sensors can't be read. Resetting the i2c
def reset_i2c():
    subprocess.run(['i2cdetect', '-y', '1'])
    time.sleep(2)

# Setup LC709203F battery monitor
if battery_sensor:
    if DEBUG:
        logging.info('## LC709203F battery monitor ##')
    try:
        if DEBUG:
            logging.info("Sensor IC version: {}".format(hex(sensor.ic_version)))
        # Set the battery pack size to 3000 mAh
        sensor.pack_size = PackSize.MAH3000
        sensor.init_RSOC()
        if DEBUG:
            logging.info("Battery size: {}".format(PackSize.string[sensor.pack_sizes]))
    except RuntimeError as exception:
        logging.error("Failed to read sensor with error: {}".format(exception))
        logging.info("Try setting the I2C clock speed to 10000Hz")

def get_cpu_temperature():
    """Get the temperature from the Raspberry Pi CPU"""
    with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
        temp = f.read()
        temp = int(temp) / 1000.0
        CPU_TEMPERATURE.labels(RBPI_SERIAL).set(temp)
        return temp

def get_temperature(factor_usr):
    """Get temperature from the weather sensor"""
    # Tuning factor for compensation. Decrease this number to adjust the
    # temperature down, and increase to adjust up
    raw_temp = bme280.get_temperature()

    factor = 2.25

    if factor_usr:
        factor = factor_usr

    cpu_temp = get_cpu_temperature()
    temperature = raw_temp - ((cpu_temp - raw_temp) / factor)

    TEMPERATURE.labels(RBPI_SERIAL).set(temperature)   # Set to a given value

def get_pressure():
    """Get pressure from the weather sensor"""
    try:
        pressure = bme280.get_pressure()
        PRESSURE.labels(RBPI_SERIAL).set(pressure)
    except IOError:
        logging.error("Could not get pressure readings. Resetting i2c.")
        reset_i2c()

def get_humidity(humidity_compensation):
    """Get humidity from the weather sensor"""
    try:
        humidity = bme280.get_humidity()
        HUMIDITY.labels(RBPI_SERIAL).set(humidity)
    except IOError:
        logging.error("Could not get humidity readings. Resetting i2c.")
        reset_i2c()

def get_gas():
    """Get all gas readings"""
    red_r0 = 200000
    ox_r0 = 20000
    nh3_r0 = 750000


    try:
        readings = gas.read_all()
        OXIDISING.labels(RBPI_SERIAL).set(readings.oxidising)
        OXIDISING_HIST.labels(RBPI_SERIAL).observe(readings.oxidising)

        REDUCING.labels(RBPI_SERIAL).set(readings.reducing)
        REDUCING_HIST.labels(RBPI_SERIAL).observe(readings.reducing)

        NH3.labels(RBPI_SERIAL).set(readings.nh3)
        NH3_HIST.labels(RBPI_SERIAL).observe(readings.nh3)

        ox_in_ppm = math.pow(10, math.log10(readings.oxidising/ox_r0) - 0.8129)
        OXIDISING_PPM.labels(RBPI_SERIAL).set(ox_in_ppm)
        OXIDISING_PPM_HIST.labels(RBPI_SERIAL).observe(ox_in_ppm)

        red_in_ppm = math.pow(10, -1.25 * math.log10(readings.reducing/red_r0) + 0.64)
        REDUCING_PPM.labels(RBPI_SERIAL).set(red_in_ppm)
        REDUCING_PPM_HIST.labels(RBPI_SERIAL).observe(red_in_ppm)

        nh3_in_ppm = math.pow(10, -1.8 * math.log10(readings.nh3/nh3_r0) - 0.163)
        NH3_PPM.labels(RBPI_SERIAL).set(nh3_in_ppm)
        NH3_PPM_HIST.labels(RBPI_SERIAL).observe(nh3_in_ppm)

    except IOError:
        logging.error("Could not get gas readings. Resetting i2c.")
        reset_i2c()

def get_noise_profile():
    """Get the noise profile"""
    try:
        low = noise.get_amplitude_at_frequency_range(20, 200)
        mid = noise.get_amplitude_at_frequency_range(200, 2000)
        high = noise.get_amplitude_at_frequency_range(2000, 8000)
        amp = noise.get_amplitude_at_frequency_range(20, 8000)
        NOISE_PROFILE_LOW_FREQ.labels(RBPI_SERIAL).set(low)
        NOISE_PROFILE_MID_FREQ.labels(RBPI_SERIAL).set(mid)
        NOISE_PROFILE_HIGH_FREQ.labels(RBPI_SERIAL).set(high)
        NOISE_PROFILE_AMP.labels(RBPI_SERIAL).set(amp)
    except IOError:
        logging.error("Could not get noise profile. Resetting i2c.")
        reset_i2c()

def get_light():
    """Get all light readings"""
    try:
       lux = ltr559.get_lux()
       prox = ltr559.get_proximity()

       LUX.labels(RBPI_SERIAL).set(lux)
       PROXIMITY.labels(RBPI_SERIAL).set(prox)
    except IOError:
        logging.error("Could not get lux and proximity readings. Resetting i2c.")
        reset_i2c()

def get_particulates():
    """Get the particulate matter readings"""
    try:
        pms_data = pms5003.read()
    except pmsReadTimeoutError:
        logging.warning("Timed out reading PMS5003.")
    except (IOError, pmsSerialTimeoutError):
        logging.warning("Could not get particulate matter readings.")
    else:
        PM1.labels(RBPI_SERIAL).set(pms_data.pm_ug_per_m3(1.0))
        PM25.labels(RBPI_SERIAL).set(pms_data.pm_ug_per_m3(2.5))
        PM10.labels(RBPI_SERIAL).set(pms_data.pm_ug_per_m3(10))

        PM1_HIST.labels(RBPI_SERIAL).observe(pms_data.pm_ug_per_m3(1.0))
        PM25_HIST.labels(RBPI_SERIAL).observe(pms_data.pm_ug_per_m3(2.5) - pms_data.pm_ug_per_m3(1.0))
        PM10_HIST.labels(RBPI_SERIAL).observe(pms_data.pm_ug_per_m3(10) - pms_data.pm_ug_per_m3(2.5))

def get_battery():
    """Get the battery voltage and percentage left"""
    try:
        voltage_reading = sensor.cell_voltage
        percentage_reading = sensor.cell_percent
        BATTERY_VOLTAGE.labels(RBPI_SERIAL).set(voltage_reading)
        BATTERY_PERCENTAGE.labels(RBPI_SERIAL).set(percentage_reading)
        if DEBUG:
            logging.info("Battery: {} Volts / {} %".format(sensor.cell_voltage, sensor.cell_percent))
    except (RuntimeError, OSError) as exception:
        logging.warning("Failed to read battery monitor with error: {}".format(exception))

def collect_all_data():
    """Collects all the data currently set"""
    sensor_data = {}
    sensor_data['temperature'] = TEMPERATURE.collect()[0].samples[0].value
    sensor_data['humidity'] = HUMIDITY.collect()[0].samples[0].value
    sensor_data['pressure'] = PRESSURE.collect()[0].samples[0].value
    sensor_data['oxidising'] = OXIDISING.collect()[0].samples[0].value
    sensor_data['reducing'] = REDUCING.collect()[0].samples[0].value
    sensor_data['nh3'] = NH3.collect()[0].samples[0].value
    sensor_data['oxidising_ppm'] = OXIDISING_PPM.collect()[0].samples[0].value
    sensor_data['reducing_ppm'] = REDUCING_PPM.collect()[0].samples[0].value
    sensor_data['nh3_ppm'] = NH3_PPM.collect()[0].samples[0].value
    sensor_data['lux'] = LUX.collect()[0].samples[0].value
    sensor_data['proximity'] = PROXIMITY.collect()[0].samples[0].value
    # sensor_data['pm1'] = PM1.collect()[0].samples[0].value
    # sensor_data['pm25'] = PM25.collect()[0].samples[0].value
    # sensor_data['pm10'] = PM10.collect()[0].samples[0].value
    sensor_data['cpu_temperature'] = CPU_TEMPERATURE.collect()[0].samples[0].value
    # sensor_data['battery_voltage'] = BATTERY_VOLTAGE.collect()[0].samples[0].value
    # sensor_data['battery_percentage'] = BATTERY_PERCENTAGE.collect()[0].samples[0].value
    # sensor_data['noise_profile_low_freq'] = NOISE_PROFILE_LOW_FREQ.collect()[0].samples[0].value
    # sensor_data['noise_profile_mid_freq'] = NOISE_PROFILE_MID_FREQ.collect()[0].samples[0].value
    # sensor_data['noise_profile_high_freq'] = NOISE_PROFILE_HIGH_FREQ.collect()[0].samples[0].value
    # sensor_data['noise_profile_amp'] = NOISE_PROFILE_AMP.collect()[0].samples[0].value

    return sensor_data

def write_to_lcd():
    """Write dta to eniro lcd"""
    got_first_data = False

    while True:
        # try:
            if not got_first_data:
                time.sleep(WRITE_TO_LCD_TIME)
                sensor_data = collect_all_data()
                got_first_data = True
            else:
                variables = [
                    "temperature",
                    "cpu_temperature",
                    "pressure",
                    "humidity",
                    "lux",
                    "oxidising_ppm",
                    "reducing_ppm",
                    "nh3_ppm",]

                units = [
                    "°C",
                    "°C",
                    "hPa",
                    "%",
                    "lx",
                    "ppm",
                    "ppm",
                    "ppm",]

                limits = [
                    [4, 18, 28, 35], # Temperature
                    [-20, 0, 55, 95], # CPU Temperature
                    [250, 650, 1013.25, 1025], # Pressure
                    [20, 30, 60, 70], # Humidity
                    [0, 0, 30000, 100000], # Lux
                    [300, 1100, 1600, 2600], # Oxidised
                    [300, 1100, 1600, 2600], # Reduced
                    [-1, 16, 64, 160],] # NH3

                # RGB palette for values on the combined screen
                palette = [
                    (0, 0, 255),           # Dangerously Low
                    (0, 255, 255),         # Low
                    (0, 255, 0),           # Normal
                    (255, 255, 0),         # High
                    (255, 0, 0),]          # Dangerously High

                for i in range(len(variables)):
                    sensor_data = collect_all_data()
                    variable = variables[i]
                    data_value = sensor_data[variable]
                    unit = units[i]
                    message = "{:.1f}{}".format(data_value, unit)
                    logging.debug('Writing to LCD: {}'.format(message))

                    img = Image.new('RGB', (WIDTH, HEIGHT), color=(0, 0, 0))
                    draw = ImageDraw.Draw(img)
                    font = ImageFont.truetype("/opt/UbuntuMonoNerdFontMono-Regular.ttf", 60)
                    font2 = ImageFont.truetype("/opt/UbuntuMonoNerdFontMono-Regular.ttf", 14)
                    size_x, size_y = draw.textsize(message, font)

                    while size_x > WIDTH:
                        font = ImageFont.truetype("/opt/UbuntuMonoNerdFontMono-Regular.ttf", font.size - 2)
                        size_x, size_y = draw.textsize(message, font)

                    lim = limits[i]
                    rgb = palette[0]
                    for j in range(len(lim)):
                        if data_value > lim[j]:
                            rgb = palette[j + 1]

                    draw.text((0,0), variable, font=font2, fill=(255, 255, 255))
                    draw.text((math.floor((WIDTH/2)-(size_x/2)), math.floor((HEIGHT)-(size_y))), message, font=font, fill=rgb)

                    st7735.display(img)
                    time.sleep(WRITE_TO_LCD_TIME)
        # except Exception as exception:
        #     logging.warning('Exception writing to LCD: {}'.format(exception))

def get_serial_number():
    """Get Raspberry Pi serial number to use as LUFTDATEN_SENSOR_UID"""
    with open('/proc/cpuinfo', 'r') as f:
        for line in f:
            if line[0:6] == 'Serial':
                return str(line.split(":")[1].strip())

def str_to_bool(value):
    if value.lower() in {'false', 'f', '0', 'no', 'n'}:
        return False
    elif value.lower() in {'true', 't', '1', 'yes', 'y'}:
        return True
    raise ValueError('{} is not a valid boolean value'.format(value))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-b", "--bind", metavar='ADDRESS', default='0.0.0.0', help="Specify alternate bind address [default: 0.0.0.0]")
    parser.add_argument("-p", "--port", metavar='PORT', default=8000, type=int, help="Specify alternate port [default: 8000]")
    parser.add_argument("-e", "--enviro", metavar='ENVIRO', type=str_to_bool, default='false', help="Device is an Enviro (not Enviro+) so don't fetch data from particulate sensor as it doesn't exist [default: false]")
    parser.add_argument("-t", "--temp", metavar='TEMPERATURE', type=float, help="The temperature compensation value to get better temperature results when the Enviro+ pHAT is too close to the Raspberry Pi board")
    parser.add_argument("-u", "--humid", metavar='HUMIDITY', type=float, help="The humidity compensation value to get better humidity results when the Enviro+ pHAT is too close to the Raspberry Pi board")
    parser.add_argument("-d", "--debug", metavar='DEBUG', type=str_to_bool, help="Turns on more vebose logging, showing sensor output and post responses [default: false]")
    parser.add_argument("-P", "--polling", metavar='POLLING', type=int, default=2, help="Polling interval in seconds, to fetch data from sensor [default: 2]")
    parser.add_argument("-L", "--lcd", metavar='LCD', type=str_to_bool, default='false', help="Display sensor data on LCD [default: false]")
    args = parser.parse_args()

    RBPI_SERIAL = get_serial_number()

    # Start up the server to expose the metrics.
    start_http_server(addr=args.bind, port=args.port)
    # Generate some requests.

    polling_interval = args.polling

    if args.debug:
        DEBUG = True

    if args.temp:
        logging.info("Using temperature compensation, reducing the output value by {}° to account for heat leakage from Raspberry Pi board".format(args.temp))

    if args.humid:
        logging.info("Using humidity compensation, increasing the output value by {}% to account for heat leakage from Raspberry Pi board".format(args.humid))

    if args.lcd:
        lcd_thread = Thread(target=write_to_lcd)
        lcd_thread.start()

    logging.info("Listening on http://{}:{}".format(args.bind, args.port))

    while True:
        get_temperature(args.temp)
        get_cpu_temperature()
        get_humidity(args.humid)
        get_pressure()
        get_light()
        get_gas()
        # get_noise_profile()

        if not args.enviro:
            get_gas()
            get_particulates()
        if DEBUG:
            logging.info('Sensor data: {}'.format(collect_all_data()))
        time.sleep(polling_interval)
