import board
import time
from analogio import AnalogIn

from adafruit_magtag.magtag import MagTag
import adafruit_mlx90614
import adafruit_sht4x
import adafruit_tca9548a

# Time between updates, in seconds.
INTERVAL = 300.0

# Location, for time zone.
LOCATION = "Australia/Sydney"

# Sensor configuration
UV_PIN = board.D10  # GPIO pin for UV sensor. Set to None if UV sensor not installed
COOL_CHANNEL = 0  # I2C multiplexor channel with cool end SHT40 sensor.
WARM_CHANNEL = 1  # I2C multiplexor channel with warm end SHT40 sensor.

# Constants for display layout
SENSOR_FONT = "NasalizationRg-Regular-30.pcf"
STATUS_FONT = "NasalizationRg-Regular-15.pcf"
FIRST_ROW_TOP = 20
SECOND_ROW_TOP = 65
THIRD_ROW_BOTTOM = 3
FIRST_COLUMN = 0.16
SECOND_COLUMN = 0.5
THIRD_COLUMN = 0.84


class GUVAS12SD:
    """
    Interface for Adafruit Analog UV sensor breadkout board GUVA-S12SD (product ID 1918).

    This is an analog sensor so the class just sets up the designated GPIO pin for analog input
    and provides properties to convert the signal to voltage, corresponding photodiode current, or
    UV index.

    Parameters
    ----------

    pin : int
        GPIO pin that the sensor breakout output is connected to, e.g. board.D10. If pin is set to
        None then no input pin will be set up all all properties will return None.
    zero_point : float, optional
        photodiode dark current, i.e the current with no incident UV, in nanoamps. Default 6.74nA.
    nanoamps_per_index: float, optional
        The amount the photodiode increases per unit increase in UV index. Default 20.92 nA/index.
    volts_per_nanoamp: float, optional
        Conversion factor from photodiode current to breakout output voltage. Default 0.0043 V/nA.

    Attributes
    ----------

    voltage : float
        Voltage measures on the designated pin, calculated as pin.value * 3.3 / 65535.0
    current : float
        Photodiode current in nanoAmps, calculated as voltage / volts_per_nanoamp
    uv_index : float
        UV index, calculated as (current - zero_point) / nanoamps_per_index

    Notes
    -----

    From the breakout product page (https://www.adafruit.com/product/1918) op-amp voltage =
    4.3 * photodiode current V/uA, & UV index = voltage / 0.1V, i.e UVI = 0.043 * current / nA.

    From the sensor datasheet (https://cdn-shop.adafruit.com/datasheets/1918guva.pdf)
    the photodiode current = ~(20.92 * UVI + 78.29) nA

    Experiment suggests a different zero point, 6.74 nA instead of 20.92 nA.

    By default this class uses the experimentally determined zero point and the 20.92 gradient
    measured from the datasheet plot.
    """
    def __init__(self,
                 pin,
                 zero_point=6.74,
                 nanoamps_per_index=20.92,
                 volts_per_nanoamp=0.0043):
        if pin is None:
            self._input = None
        else:
            self._input = AnalogIn(pin)
        self.zero_point = zero_point
        self.nanoamps_per_index = nanoamps_per_index
        self.volts_per_nanoamp = volts_per_nanoamp

    @property
    def voltage(self):
        if self._input is None:
            return None
        else:
            return self._input.value * 3.3 / 65535.0  # Voltage from op-amp.

    @property
    def current(self):
        if self._input is None:
            return None
        else:
            return self.voltage / self.volts_per_nanoamp  # Photodiode current in nA.

    @property
    def uv_index(self):
        if self._input is None:
            return None
        else:
            return (self.current - self.zero_point) / self.nanoamps_per_index  # Approximate UV index.


def percentage(voltage):
    """Approximate conversion of LiPo cell voltage to charge percentage.

    Equation taken from https://electronics.stackexchange.com/questions/435837/calculate-battery-percentage-on-lipo-battery

    Seems to be really inaccurate for these low current draw situations."""
    return 123.0 * (1 - 1 / (1 + (voltage / 3.7)**80)**0.165)


def setup_display(magtag):
    """Set up the text labels (locations) on the e-ink display."""
    magtag.add_text(text_position=(int(FIRST_COLUMN * magtag.graphics.display.width),
                                   FIRST_ROW_TOP),
                    text_scale=1,
                    text_anchor_point=(0.5, 0.0),
                    text_font=SENSOR_FONT)
    magtag.add_text(text_position=(int(FIRST_COLUMN * magtag.graphics.display.width),
                                   SECOND_ROW_TOP),
                    text_scale=1,
                    text_anchor_point=(0.5, 0.0),
                    text_font=SENSOR_FONT)
    magtag.add_text(text_position=(int(SECOND_COLUMN * magtag.graphics.display.width),
                                   FIRST_ROW_TOP),
                    text_scale=1,
                    text_anchor_point=(0.5, 0.0),
                    text_font=SENSOR_FONT)
    magtag.add_text(text_position=(int(SECOND_COLUMN * magtag.graphics.display.width),
                                   SECOND_ROW_TOP),
                    text_scale=1,
                    text_anchor_point=(0.5, 0.0),
                    text_font=SENSOR_FONT)
    magtag.add_text(text_position=(int(SECOND_COLUMN * magtag.graphics.display.width),
                                   FIRST_ROW_TOP),
                    text_scale=1,
                    text_anchor_point=(0.5, 0.0),
                    text_font=SENSOR_FONT)
    magtag.add_text(text_position=(int(THIRD_COLUMN * magtag.graphics.display.width),
                                   SECOND_ROW_TOP),
                    text_scale=1,
                    text_anchor_point=(0.5, 0.0),
                    text_font=SENSOR_FONT)
    magtag.add_text(text_position=(int(SECOND_COLUMN * magtag.graphics.display.width - 1),
                                   magtag.graphics.display.height - THIRD_ROW_BOTTOM),
                    text_scale=1,
                    text_anchor_point=(0.5, 1.0),
                    text_font=STATUS_FONT)
    print("Display set up.")


def connect_wifi(magtag):
    """Connect to WiFi using details in secrets.py, and check for errors."""
    try:
        magtag.network.connect()
        print("Connected")
    except Exception as err:
        # WiFi is down! Panic, print error message, and re-raise exception to trigger sleep.
        print("Error connecting to WiFi:\n{}".format(err))
        raise err


def setup_mux(i2c, errors):
    """Initialise the multiplexer, & two multiplexer channels."""
    try:
        i2c_mux = adafruit_tca9548a.TCA9548A(i2c)
        i2c_cool = adafruit_tca9548a.TCA9548A_Channel(i2c_mux, COOL_CHANNEL)
        i2c_warm = adafruit_tca9548a.TCA9548A_Channel(i2c_mux, WARM_CHANNEL)
    except Exception as err:
        # Record error and carry on as best we can.
        msg = "Error initialising TCA9548A:\n{}".format(err)
        print(msg)
        errors += (msg + "\n")
        return None, None
    else:
        print("I2C initialised")
        return i2c_cool, i2c_warm


def setup_i2c_sensor(sensor_class, i2c_bus, errors):
    """ Initialise one of the I2C connected sensors, returning None on error."""
    if i2c_bus is None:
        # This sensor uses the multipler and there was an error initialising that.
        return None
    try:
        sensor = sensor_class(i2c_bus)
    except Exception as err:
        # Error initialising this sensor, try to continue without it.
        msg = "Error initialising {}:\n{}".format(sensor_class.__name___, err)
        print(msg)
        errors += (msg + "\n")
        return None
    else:
        print("{} initialised.".format(sensor_class.__name__))
        return sensor


def get_local_time(magtag, errors):
    """Get current time from Adafruit IO."""
    try:
        local_time = magtag.network.get_local_time(location=LOCATION)
    except Exception as err:
        msg = "Error getting local time:\n{}".format(err)
        print(msg)
        errors += (msg + "\n")
        return None
    else:
        print("Got local time: {}".format(local_time))
        return local_time.split(".")[0]  # Trim off fractions of a second and timezone info


def safe_read(sensor, attribute_name, errors):
    """Reads I2C sensor while catching errors."""
    if sensor is None:
        # There was an error initialising the sensor, can't even try to read it.
        return None
    else:
        try:
            value = getattr(sensor, attribute_name)
        except Exception as err:
            msg = "Error reading {}.{}:\n{}".format(sensor.__name__, attribute_name, err)
            print(msg)
            errors += (msg + "\n")
            return None
        else:
            print("{}.{} returned {}".format(sensor.__name__, attribute_name, value))
            return value


def push_to_io(sensor_readings, magtag, errors):
    """Send sensor readings to Adafruit IO while catching errors."""
    for key, value in sensor_readings.items():
        if value is None:
            # There was a problem reading the sensor, there's no value to send.
            continue
        else:
            try:
                magtag.push_to_io(key, value)
            except Exception as err:
                msg = "Error sending {}:{} to Adafruit IO:\n{}".format(key, value, err)
                print(msg)
                errors += (msg + "\n")
    try:
        magtag.push_to_io("jerry.errors", errors if errors else "None")
    except Exception as err:
        print("Error sending errors {} to Adafruit IO:\n{}".format(errors, err))


def update_text(magtag, index, value, format_string, no_value_string, refresh=False):
    """Update sensor readings in display, flagging missing readings."""
    if value is None:
        # Mising sensor reading, put missing value text in display.
        magtag.set_text(no_value_string, index=index, auto_refresh=refresh)
    else:
        magtag.set_text(format_string.format(value), index=index, auto_refresh=refresh)


try:
    # Do everything inside a try ... except ... finally because whatever happens we want to be sure
    # that the MagTag will try again in 5 minutes.
    start = time.monotonic()  # Make a note of the time we woke up/restarted
    print("Woke up!")
    errors = ""  # Empty string to store any error messages

    magtag = MagTag()  # Initalise the MagTag
    print("MagTag initialised")
    # First Magag battery reading seems to be no good, so read it now and throw it away.
    _ = magtag.peripherals.battery

    setup_display(magtag)
    connect_wifi(magtag)

    # Initialise all the sensors.
    i2c = board.I2C()
    i2c_cool, i2c_warm = setup_mux(i2c, errors)
    sht_cool = setup_i2c_sensor(adafruit_sht4x.SHT4x, i2c_cool, errors)
    sht_warm = setup_i2c_sensor(adafruit_sht4x.SHT4x, i2c_warm, errors)
    mlx = setup_i2c_sensor(adafruit_mlx90614.MLX90614, i2c, errors)
    guva = GUVAS12SD(UV_PIN)

    local_time = get_local_time(magtag, errors)

    # Read all the sensors.
    temperature_cool = safe_read(sht_cool, "temperature", errors)
    humidity_cool = safe_read(sht_cool, "relative_humidity", errors)
    temperature_warm = safe_read(sht_warm, "temperature", errors)
    humidity_warm = safe_read(sht_warm, "relative_humidity", errors)
    temperature_basking = safe_read(mlx, "object_temperature", errors)
    uv_index = guva.uv_index
    battery = percentage(magtag.peripherals.battery)

    # Send everything to Adafruit IO
    push_to_io({"jerry.temperature-cool": temperature_cool,
                "jerry.humidity-cool": humidity_cool,
                "jerry.temperature-warm": temperature_warm,
                "jerry.humidity-warm": humidity_warm,
                "jerry.temperature-basking": temperature_basking,
                "jerry.uv-index": uv_index,
                "jerry.battery": battery},
               magtag,
               errors)

    # Update display
    update_text(magtag, 0, temperature_cool, "{:4.1f}C", "--.-C")
    update_text(magtag, 1, humidity_cool, "{:2.0f}%", "--%")
    update_text(magtag, 2, temperature_warm, "{:4.1f}C", "--.-C")
    update_text(magtag, 3, humidity_warm, "{:2.0f}%", "--%")
    update_text(magtag, 4, temperature_basking, "{:4.1f}C", "--.-C")
    update_text(magtag, 5, uv_index, "{:4.2f}" "-.--")
    update_text(magtag, 6, local_time, "Last update: {}", "Last update: XXXX-XX-XX XX:XX:XX",
                refresh=True)
    print("Display updated")

except Exception as err:
    # Unhandled exception. Just send it out the serial port and carry on to sleep.
    print("Exception: {}".format(err))

finally:
    # Go into a deep sleep and run again regardles.
    sleep_duration = INTERVAL - (time.monotonic() - start)
    if sleep_duration < 10:
        # Have had occasional errors due to somehow trying to sleep for negative durations,
        # so check to make sure sleep is for at least 10 seconds.
        sleep_duration = 10
    print("Going to sleep for {} seconds...".format(sleep_duration))
    magtag.exit_and_deep_sleep(sleep_duration)
