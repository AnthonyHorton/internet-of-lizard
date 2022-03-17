import board
import time

from adafruit_magtag.magtag import MagTag
import adafruit_mlx90614
import adafruit_sht4x
import adafruit_tca9548a

def percentage(voltage):
    # Approximate conversion of LiPo cell voltage to change percentage.
    # Equation taken from https://electronics.stackexchange.com/questions/435837/calculate-battery-percentage-on-lipo-battery
    return 123.0 * (1 - 1 / (1 + (voltage / 3.7)**80)**0.165)

def sleep_time(interval=300):
    sleep_duration = interval - (time.monotonic() - start)
    print("Going to sleep for {} seconds...".format(sleep_duration))
    magtag.exit_and_deep_sleep(sleep_duration)

start = time.monotonic()  # Note the time that we woke up
print("Woke up!")
errors = ""
magtag = MagTag() # Initalise the MagTag
print("MagTag initialised")
_ = magtag.peripherals.battery  # First reading after reset/wake seems to be no good, read & throw away.

# Set up the text locations on the e-ink display.
magtag.add_text(text_position = (int(0.16 * magtag.graphics.display.width) - 1,
                                 int(0.2 * magtag.graphics.display.height) - 1),
                text_scale=3,
                text_anchor_point=(0.5, 0.5))
magtag.add_text(text_position = (int(0.16 * magtag.graphics.display.width) - 1,
                                 int(0.6 * magtag.graphics.display.height) - 1),
                text_scale=3,
                text_anchor_point=(0.5, 0.5))
magtag.add_text(text_position = (int(0.51 * magtag.graphics.display.width) - 1,
                                 int(0.2 * magtag.graphics.display.height) - 1),
                text_scale=3,
                text_anchor_point=(0.5, 0.5))
magtag.add_text(text_position = (int(0.51 * magtag.graphics.display.width) - 1,
                                 int(0.6 * magtag.graphics.display.height) - 1),
                text_scale=3,
                text_anchor_point=(0.5, 0.5))
magtag.add_text(text_position = (int(0.86 * magtag.graphics.display.width) - 1,
                                 int(0.2 * magtag.graphics.display.height) - 1),
                text_scale=3,
                text_anchor_point=(0.5, 0.5))
magtag.add_text(text_position = (int(0.86 * magtag.graphics.display.width) - 1,
                                 int(0.6 * magtag.graphics.display.height) - 1),
                text_scale=3,
                text_anchor_point=(0.5, 0.5))
magtag.add_text(text_position = (int(0.5 * magtag.graphics.display.width) - 1,
                                 int(0.9 * magtag.graphics.display.height) - 1),
                text_scale=1,
                text_anchor_point=(0.5, 0.5))

# Connect to WiFi
try:
    magtag.network.connect()
    print("Connected")
except Exception as err:
    # WiFi is down! Panic, sleep, then try again.
    print("Error connecting to WiFi:\n{}".format(err))
    sleep_time = 300 - (time.monotonic() - start)
    sleep_time()  # Go back to sleep

# Initialise I2C, the multiplexer, & two multiplexer channels.
i2c = board.I2C()
try:
    i2c_mux = adafruit_tca9548a.TCA9548A(i2c)
    i2c_0 = adafruit_tca9548a.TCA9548A_Channel(i2c_mux, 0)
    i2c_1 = adafruit_tca9548a.TCA9548A_Channel(i2c_mux, 1)
except Exception as err:
    msg = "Error initialising TCA9548A:\n{}".format(err)
    print(msg)
    errors += (msg + "\n")
print("I2C initialised")

# Initialise the sensors, using the multiplexer channels.
try:
    sht_cool = adafruit_sht4x.SHT4x(i2c_0)
except Exception as err:
    msg = "Error initialising cool end SHT40:\n{}".format(err)
    print(msg)
    errors += (msg + "\n")
    sht_cool = None

try:
    sht_warm = adafruit_sht4x.SHT4x(i2c_1)
except Exception as err:
    msg = "Error initialising warm end SHT40:\n{}".format(err)
    print(msg)
    errors += (msg + "\n")
    sht_warm = None

try:
    mlx = adafruit_mlx90614.MLX90614(i2c)
except Exception as err:
    msg = "Error initialising MLX90614:\n{}".format(err)
    print(msg)
    errors += (msg + "\n")
    mlx = None

print("Sensors initialised")

# Get current time from Adafruit IO, then read the sensors
try:
    local_time = magtag.network.get_local_time(location="Australia/Sydney")
except Exception as err:
    msg = "Error getting local time:\n{}".format(err)
    print(msg)
    errors += (msg + "\n")
    local_time = None

if sht_cool:
    temperature_cool = sht_cool.temperature
    humidity_cool = sht_cool.relative_humidity
else:
    temperature_cool = None
    humidity_cool = None

if sht_warm:
    temperature_warm = sht_warm.temperature
    humidity_warm = sht_warm.relative_humidity
else:
    temperature_cool = None
    humidity_cool = None

if mlx:
    temperature_basking = mlx.object_temperature
else:
    temperature_basking = None

battery = percentage(magtag.peripherals.battery)
print("Sensors read")

# Push sensor readings to Adafruit IO
try:
    magtag.push_to_io("jerry.errors", errors if errors else "None")
    if temperature_cool:
        magtag.push_to_io("jerry.temperature-cool", temperature_cool)
    if humidity_cool:
        magtag.push_to_io("jerry.humidity-cool", humidity_cool)
    if temperature_warm:
        magtag.push_to_io("jerry.temperature-warm", temperature_warm)
    if humidity_warm:
        magtag.push_to_io("jerry.humidity-warm", humidity_warm)
    if temperature_basking:
        magtag.push_to_io("jerry.temperature-basking", temperature_basking)
    magtag.push_to_io("jerry.battery", battery)
    print("Readings pushed to Adafruit IO")
except Exception as err:
    msg = "Error pushing data to Adafruit IO:\n{}".format(err)
    print(msg)

# Set the text values and update the e-ink display.
if temperature_cool:
    magtag.set_text("{:4.1f}C".format(temperature_cool),
                    index=0, auto_refresh=False)
else:
    magtag.set_text("--.-C",
                    index=0, auto_refresh=False)    

if humidity_cool:
    magtag.set_text("{:2.0f}%".format(humidity_cool),
                    index=1, auto_refresh=False)
else:
    magtag.set_text("--%",
                    index=1, auto_refresh=False)   

if temperature_warm:
    magtag.set_text("{:4.1f}C".format(temperature_warm),
                    index=2, auto_refresh=False)
else:
    magtag.set_text("--.-C",
                    index=2, auto_refresh=False)

if humidity_warm:
    magtag.set_text("{:2.0f}%".format(humidity_warm),
                    index=3, auto_refresh=False)
else:
    magtag.set_text("--%",
                    index=3, auto_refresh=False)

if temperature_basking:
    magtag.set_text("{:4.1f}C".format(temperature_basking),
                    index=4, auto_refresh=False)
else:
    magtag.set_text("--.-C",
                    index=4, auto_refresh=False)

magtag.set_text("-.-",
                index=5, auto_refresh=False)

if local_time:
    magtag.set_text("Last update: {}   Battery: {:.0f}%".format(local_time.split(".")[0], battery),
                    index=6)
else:
    magtag.set_text("Last update: XXXX-XX-XX XX:XX:XX   Battery: {:.0f}%".format(battery),
                    index=6)

print("Display updated")

# Go into a deep sleep until it's 5 minutes since we last work up.
sleep_time()
