#!/usr/bin/env python3
import time
import subprocess
import ST7735
import serial
import os
import math
import logging
import board
import argparse
from threading import Thread
from prometheus_client import start_http_server, Gauge, Histogram
from pms5003 import PMS5003, ReadTimeoutError as pmsReadTimeoutError, SerialTimeoutError as pmsSerialTimeoutError
from PIL import Image, ImageDraw, ImageFont
from enviroplus.noise import Noise
from enviroplus import gas
from bme280 import BME280
from adafruit_lc709203f import LC709203F, PackSize

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

# ----------------------------------------------------------------------------------------------

logging.basicConfig(
    format='%(asctime)s.%(msecs)03d %(levelname)-8s %(message)s',
    level=logging.INFO,
    handlers=[logging.FileHandler("enviroplus_exporter.log"),
              logging.StreamHandler()],
    datefmt='%Y-%m-%d %H:%M:%S')

# ----------------------------------------------------------------------------------------------

bus = SMBus(1)
bme280 = BME280(i2c_dev=bus)
noise = Noise()

try:
    pms5003 = PMS5003()
except serial.serialutil.SerialException:
    logging.warning("Failed to initialise PMS5003.")

# Initialize display
st7735 = ST7735.ST7735(
    port=0,
    cs=1,
    dc=9,
    backlight=12,
    rotation=270,
    spi_speed_hz=10000000
)
st7735.begin()

# ----------------------------------------------------------------------------------------------

WIDTH = st7735.width
HEIGHT = st7735.height

DEBUG = os.getenv('DEBUG', 'false') == 'true'
RBPI_SERIAL = ""
PREFIX = os.getenv('PREFIX', 'enviroplus_')

TEMPERATURE = Gauge(PREFIX+'temperature','Temperature measured (*C)', ['serial'])
PRESSURE = Gauge(PREFIX+'pressure','Pressure measured (hPa)', ['serial'])
HUMIDITY = Gauge(PREFIX+'humidity','Relative humidity measured (%)', ['serial'])
OXIDISING = Gauge(PREFIX+'oxidising','Mostly nitrogen dioxide but could include NO and Hydrogen (Ohms)', ['serial'])
REDUCING = Gauge(PREFIX+'reducing', 'Mostly carbon monoxide but could include H2S, Ammonia, Ethanol, Hydrogen, Methane, Propane, Iso-butane (Ohms)', ['serial'])
NH3 = Gauge(PREFIX+'nh3', 'mostly Ammonia but could also include Hydrogen, Ethanol, Propane, Iso-butane (Ohms)', ['serial'])
OXIDISING_PPM = Gauge(PREFIX+'oxidising_ppm','Mostly nitrogen dioxide but could include NO and Hydrogen (ppm)', ['serial'])
REDUCING_PPM = Gauge(PREFIX+'reducing_ppm', 'Mostly carbon monoxide but could include H2S, Ammonia, Ethanol, Hydrogen, Methane, Propane, Iso-butane (ppm)', ['serial'])
NH3_PPM = Gauge(PREFIX+'nh3_ppm', 'mostly Ammonia but could also include Hydrogen, Ethanol, Propane, Iso-butane (ppm)', ['serial'])
LUX = Gauge(PREFIX+'lux', 'current ambient light level (lux)', ['serial'])
PROXIMITY = Gauge(PREFIX+'proximity', 'proximity, with larger numbers being closer proximity and vice versa', ['serial'])
PM1 = Gauge(PREFIX+'pm1', 'Particulate Matter of diameter less than 1 micron. Measured in micrograms per cubic metre (ug/m3)', ['serial'])
PM25 = Gauge(PREFIX+'pm25', 'Particulate Matter of diameter less than 2.5 microns. Measured in micrograms per cubic metre (ug/m3)', ['serial'])
PM10 = Gauge(PREFIX+'pm10', 'Particulate Matter of diameter less than 10 microns. Measured in micrograms per cubic metre (ug/m3)', ['serial'])
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

# ----------------------------------------------------------------------------------------------

# Sometimes the sensors can't be read. Resetting the i2c
def reset_i2c():
    subprocess.run(['i2cdetect', '-y', '1'])
    time.sleep(2)

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

# ----------------------------------------------------------------------------------------------

def get_cpu_temperature():
    """Get the temperature from the Raspberry Pi CPU"""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            temp = f.read()
            temp = int(temp) / 1000.0
            return temp
    except IOError:
        logging.error("Could not get CPU temperature.")
        return None

def collect_cpu_temperature(temperature):
    """Collect the temperature from the Raspberry Pi CPU"""
    if temperature:
        CPU_TEMPERATURE.labels(RBPI_SERIAL).set(temperature)

# ----------------------------------------------------------------------------------------------

def get_temperature(factor_usr):
    """Get temperature from the weather sensor"""
    try:
        raw_temp = bme280.get_temperature()

        factor = 1.5

        if factor_usr:
            factor = factor_usr

        cpu_temp = get_cpu_temperature()
        temperature = raw_temp - ((cpu_temp - raw_temp) / factor)

        return temperature
    except IOError:
        logging.error("Could not get temperature readings. Resetting i2c.")
        reset_i2c()
        return None

def collect_temperature(temperature):
    """Collect the temperature from the weather sensor"""
    if temperature:
        TEMPERATURE.labels(RBPI_SERIAL).set(temperature)

# ----------------------------------------------------------------------------------------------

def get_pressure():
    """Get pressure from the weather sensor"""
    try:
        pressure = bme280.get_pressure()
        PRESSURE.labels(RBPI_SERIAL).set(pressure)
        return pressure
    except IOError:
        logging.error("Could not get pressure readings. Resetting i2c.")
        reset_i2c()
        return None

def collect_pressure(pressure):
    """Collect the pressure from the weather sensor"""
    if pressure:
        PRESSURE.labels(RBPI_SERIAL).set(pressure)

# ----------------------------------------------------------------------------------------------

def get_humidity(humidity_compensation):
    """Get humidity from the weather sensor"""
    try:
        humidity = bme280.get_humidity()
        HUMIDITY.labels(RBPI_SERIAL).set(humidity)
        return humidity
    except IOError:
        logging.error("Could not get humidity readings. Resetting i2c.")
        reset_i2c()
        return None

def collect_humidity(humidity):
    """Collect the humidity from the weather sensor"""
    if humidity:
        HUMIDITY.labels(RBPI_SERIAL).set(humidity)

# ----------------------------------------------------------------------------------------------

def get_gas():
    """Get all gas readings"""
    red_r0 = 200000
    ox_r0 = 20000
    nh3_r0 = 750000

    try:
        readings = gas.read_all()
        ox_in_ppm = math.pow(10, math.log10(readings.oxidising/ox_r0) - 0.8129)
        red_in_ppm = math.pow(10, -1.25 * math.log10(readings.reducing/red_r0) + 0.64)
        nh3_in_ppm = math.pow(10, -1.8 * math.log10(readings.nh3/nh3_r0) - 0.163)
        return readings.oxidising, readings.reducing, readings.nh3, ox_in_ppm, red_in_ppm, nh3_in_ppm
    except IOError:
        logging.error("Could not get gas readings. Resetting i2c.")
        reset_i2c()
        return None, None, None, None, None, None

def collect_gas(data):
    """Collect the gas readings"""
    ox, red, nh3, ox_in_ppm, red_in_ppm, nh3_in_ppm = data
    if ox:
        OXIDISING.labels(RBPI_SERIAL).set(ox)
        OXIDISING_HIST.labels(RBPI_SERIAL).observe(ox)
    if red:
        REDUCING.labels(RBPI_SERIAL).set(red)
        REDUCING_HIST.labels(RBPI_SERIAL).observe(red)
    if nh3:
        NH3.labels(RBPI_SERIAL).set(nh3)
        NH3_HIST.labels(RBPI_SERIAL).observe(nh3)
    if ox_in_ppm:
        OXIDISING_PPM.labels(RBPI_SERIAL).set(ox_in_ppm)
        OXIDISING_PPM_HIST.labels(RBPI_SERIAL).observe(ox_in_ppm)
    if red_in_ppm:
        REDUCING_PPM.labels(RBPI_SERIAL).set(red_in_ppm)
        REDUCING_PPM_HIST.labels(RBPI_SERIAL).observe(red_in_ppm)
    if nh3_in_ppm:
        NH3_PPM.labels(RBPI_SERIAL).set(nh3_in_ppm)
        NH3_PPM_HIST.labels(RBPI_SERIAL).observe(nh3_in_ppm)

# ----------------------------------------------------------------------------------------------

# To be implemented in the future
# def get_noise_profile():
#     """Get the noise profile"""
#     try:
#         low = noise.get_amplitude_at_frequency_range(20, 200)
#         mid = noise.get_amplitude_at_frequency_range(200, 2000)
#         high = noise.get_amplitude_at_frequency_range(2000, 8000)
#         amp = noise.get_amplitude_at_frequency_range(20, 8000)
#         NOISE_PROFILE_LOW_FREQ.labels(RBPI_SERIAL).set(low)
#         NOISE_PROFILE_MID_FREQ.labels(RBPI_SERIAL).set(mid)
#         NOISE_PROFILE_HIGH_FREQ.labels(RBPI_SERIAL).set(high)
#         NOISE_PROFILE_AMP.labels(RBPI_SERIAL).set(amp)
#     except IOError:
#         logging.error("Could not get noise profile. Resetting i2c.")
#         reset_i2c()

def get_light():
    """Get all light readings"""
    try:
        lux = ltr559.get_lux()
        prox = ltr559.get_proximity()
        return lux, prox
    except IOError:
        logging.error("Could not get lux and proximity readings. Resetting i2c.")
        reset_i2c()
        return None, None

def collect_light(data):
    """Collect the light readings"""
    lux, prox = data
    if lux:
        LUX.labels(RBPI_SERIAL).set(lux)
    if prox:
        PROXIMITY.labels(RBPI_SERIAL).set(prox)

# ----------------------------------------------------------------------------------------------

def get_particulates():
    """Get the particulate matter readings"""
    try:
        pms_data = pms5003.read()
        return pms_data.pm_ug_per_m3(1.0), pms_data.pm_ug_per_m3(2.5), pms_data.pm_ug_per_m3(10)
    except pmsReadTimeoutError:
        logging.warning("Timed out reading PMS5003.")
        return None, None, None
    except (IOError, pmsSerialTimeoutError):
        logging.warning("Could not get particulate matter readings.")
        return None, None, None

def collect_particulates(data):
    """Collect the particulate matter readings"""
    pm1, pm25, pm10 = data
    if pm1:
        PM1.labels(RBPI_SERIAL).set(pm1)
        PM1_HIST.labels(RBPI_SERIAL).observe(pm1)
    if pm25:
        PM25.labels(RBPI_SERIAL).set(pm25)
        PM25_HIST.labels(RBPI_SERIAL).observe(pm25-pm1)
    if pm10:
        PM10.labels(RBPI_SERIAL).set(pm10)
        PM10_HIST.labels(RBPI_SERIAL).observe(pm10-pm25)

# ----------------------------------------------------------------------------------------------

def collect_all_data():
    """Collects all the data currently set"""
    sensor_data = {}
    sensor_data['temperature'] = TEMPERATURE.collect()[0].samples[0].value
    sensor_data['cpu_temperature'] = CPU_TEMPERATURE.collect()[0].samples[0].value
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
    # sensor_data['noise_profile_low_freq'] = NOISE_PROFILE_LOW_FREQ.collect()[0].samples[0].value
    # sensor_data['noise_profile_mid_freq'] = NOISE_PROFILE_MID_FREQ.collect()[0].samples[0].value
    # sensor_data['noise_profile_high_freq'] = NOISE_PROFILE_HIGH_FREQ.collect()[0].samples[0].value
    # sensor_data['noise_profile_amp'] = NOISE_PROFILE_AMP.collect()[0].samples[0].value

    return sensor_data

def write_to_lcd():
    """Write dta to eniro lcd"""
    got_first_data = False
    loading = 0

    while True:
        try:
            if not got_first_data:
                # wait
                time.sleep(1)

                # loading screen
                img = Image.new('RGB', (WIDTH, HEIGHT), color=(0, 0, 0))
                draw = ImageDraw.Draw(img)
                font = ImageFont.truetype("/opt/UbuntuMonoNerdFontMono-Regular.ttf", 60)
                font2 = ImageFont.truetype("/opt/UbuntuMonoNerdFontMono-Regular.ttf", 14)
                size_x, size_y = draw.textsize(message, font)

                while size_x > WIDTH:
                    font = ImageFont.truetype("/opt/UbuntuMonoNerdFontMono-Regular.ttf", font.size - 2)
                    size_x, size_y = draw.textsize(message, font)

                draw.text((0,0), "Loading", font=font2, fill=(0, 255, 0))
                message = "*"*loading
                draw.text((math.floor((WIDTH/2)-(size_x/2)), math.floor((HEIGHT)-(size_y))), message, font=font, fill=(0, 255, 0))
                loading = (loading + 1) % 4
                st7735.display(img)

                # collect data
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
                    [-1, -1, 1000, 2000], # Lux
                    [-1, -1, 1600, 2600], # Oxidised
                    [-1, -1, 1600, 2600], # Reduced
                    [-1, -1, 400, 600],] # NH3

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
        except Exception as exception:
            logging.warning('Exception writing to LCD: {}'.format(exception))

# ----------------------------------------------------------------------------------------------

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

    start_http_server(addr=args.bind, port=args.port)

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
        collect_temperature(get_temperature(args.temp))
        collect_cpu_temperature(get_cpu_temperature())
        collect_humidity(get_humidity(args.humid))
        collect_pressure(get_pressure())
        collect_light(get_light())
        collect_gas(get_gas())
        # collect_noise_profile(get_noise_profile())

        if not args.enviro:
            collect_light(get_light())
            collect_gas(get_gas())

        if DEBUG:
            logging.info('Sensor data: {}'.format(collect_all_data()))

        time.sleep(polling_interval)
