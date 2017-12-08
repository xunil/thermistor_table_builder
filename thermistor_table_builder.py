#!/usr/bin/env python
import numpy, re, serial, signal, sys, time
sys.path.append('/home/pi/max31855')

from max31855 import MAX31855, MAX31855Error
from RPi import GPIO

def signal_handler(signal, frame):
    print("Exiting due to signal {0}".format(signal))
    ttb.cleanup()

class ThermistorTableBuilder:
    temp_report_regex = re.compile(
        r'^\s*T:(?P<hotend_temp>[\d\s.]+)/(?P<hotend_target>[\d\s.]+) \((?P<hotend_adc>[\d\s.]+)\) '
        r'B:(?P<bed_temp>[\d\s.]+)/(?P<bed_target>[\d\s.]+) \((?P<bed_adc>[\d\s.]+)\) '
        r'@:(?P<hotend_pwm>[\d\s.]+) B@:(?P<bed_pwm>[\d\s.]+)$'
    )

    def __init__(self, thermocouple, serial_port, output_file, sample_count=300, max_tc_stddev=0.15, max_adc_stddev=0.3):
        self.thermocouple = thermocouple
        self.serial_port = serial_port
        self.output_file = output_file
        self.sample_count = sample_count
        self.max_tc_stddev = max_tc_stddev
        self.max_adc_stddev = max_adc_stddev

    def parse_temp_report(self, line):
        result = self.temp_report_regex.match(line)
        if result is not None:
            return {k: float(v) for k,v in result.groupdict().items()}
        return None

    def run(self, start, end, step):
        response = None
        while True:
            response = self.serial_port.readline()
            if len(response) == 0:
                break
        self.serial_port.write(b"M155 S1\n")
        self.serial_port.flush()
        time.sleep(0.1)
        response = self.serial_port.readline()
        if response.strip() != 'ok':
            print("Unexpected response {} from 3D printer!".format(response))
            self.cleanup()

        for temp in range(start, end, step):
            self.measure_at_temp(temp)
        self.cleanup()

    def measure_at_temp(self, target_temp):
        measurement_complete = False
        adc_measurements = [0.0]
        thermocouple_measurements = [0.0]
        timeouts = 0

        print("Commanding bed temperature of {0}C".format(target_temp))
        self.serial_port.write(b"M117 Measuring at {0}C...\n".format(target_temp))
        self.serial_port.write(b"M140 S{0}\n".format(target_temp))
        self.serial_port.flush()
        time.sleep(0.1)

        while not measurement_complete:
            response = self.serial_port.readline()
            if len(response) == 0:
                print("Timeout while reading from the 3D printer!")
                timeouts += 1
                if timeouts == 3:
                    print("Giving up after 3 timeouts.")
                    self.cleanup()
                else:
                    continue

            timeouts = 0
            response = response.strip()
            if response == '':
                continue
            if response == 'ok' or response == 'start' or response[0:4] == 'echo' or response[0] == ' ':
                print(response)
                continue

            measured_temp = self.thermocouple.get()
            metrics = self.parse_temp_report(response)
            if metrics is None:
                print("Unexpected response from 3D printer: {}".format(response))
                continue

            adc_stddev = numpy.std(adc_measurements)
            adc_median = numpy.median(adc_measurements)
            thermocouple_stddev = numpy.std(thermocouple_measurements)
            thermocouple_median = numpy.median(thermocouple_measurements)

            if thermocouple_stddev > 0.0 and (abs(measured_temp - thermocouple_median) / thermocouple_stddev) > 3.0:
                print("[{epoch}] Thermocouple measurement {measured_temp:3.2f} out of range, rejecting".format(
                    epoch=time.ctime(), measured_temp=measured_temp
                ))
                continue

            if adc_stddev > 0.0 and (abs(metrics['bed_adc'] - adc_median) / adc_stddev) > 3.0:
                print("[{epoch}] Thermocouple measurement {measured_temp:3.2f} out of range, rejecting".format(
                    epoch=time.ctime(), measured_temp=measured_temp
                ))
                continue

            thermocouple_measurements.append(measured_temp)
            adc_measurements.append(metrics['bed_adc'])
            if len(adc_measurements) > self.sample_count:
                adc_measurements.pop(0)
            if len(thermocouple_measurements) > self.sample_count:
                thermocouple_measurements.pop(0)

            print(
                "[{epoch}] target: {bed_target:3.2f} reported: {bed_temp:3.2f} "
                "measured: {measured_temp:3.2f} ADC: {bed_adc:4.2f} ADC stddev: {adc_stddev:3.2f} "
                "measured stddev: {thermocouple_stddev:3.2f}".format(
                    epoch=time.ctime(), measured_temp=measured_temp, adc_stddev=adc_stddev,
                    thermocouple_stddev=thermocouple_stddev, **metrics
                )
            )

            self.output_file.write(
                "{epoch},{target_temp:3.2f},{thermocouple_mean:3.2f},"
                "{thermocouple_stddev:3.2f},{adc_mean:4.2f},{adc_stddev:3.2f},\"\"\n".format(
                    epoch=time.time(),
                    target_temp=target_temp,
                    thermocouple_mean=numpy.mean(thermocouple_measurements),
                    thermocouple_stddev=thermocouple_stddev,
                    adc_mean=numpy.mean(adc_measurements),
                    adc_stddev=adc_stddev
                )
            )
            self.output_file.flush()

            if len(thermocouple_measurements) < self.sample_count or len(adc_measurements) < self.sample_count:
                continue  # Don't even consider values until a sufficient sample size has been collected

            if thermocouple_stddev < self.max_tc_stddev and adc_stddev < self.max_adc_stddev:
                # ADC stable for sample_count intervals
                print("[{epoch}] Temperature reading stabilized".format(epoch=time.ctime()))
                self.output_file.write(
                    "{epoch},{target_temp:3.2f},{thermocouple_mean:3.2f},"
                    "{thermocouple_stddev:3.2f},{adc_mean:4.2f},{adc_stddev:3.2f},\"Yes\"\n".format(
                        epoch=time.time(),
                        target_temp=target_temp,
                        thermocouple_mean=numpy.mean(thermocouple_measurements),
                        thermocouple_stddev=thermocouple_stddev,
                        adc_mean=numpy.mean(adc_measurements),
                        adc_stddev=adc_stddev
                    )
                )
                self.output_file.flush()
                measurement_complete = True

    def cleanup(self):
        self.serial_port.write(b"M117 Test complete\n")  # set LCD message
        self.serial_port.write(b"M140 S0\n")  # turn off bed heater
        self.serial_port.write(b"M155 S0\n")  # stop auto-reporting temperature
        self.serial_port.close()
        self.output_file.close()
        self.thermocouple.cleanup()
        sys.exit(0)


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print "Usage: {} start end step".format(sys.argv[0])
        sys.exit(0)

    cs_pin = 24
    clock_pin = 23
    data_pin = 22
    units = "c"
    tc = MAX31855(cs_pin, clock_pin, data_pin, units, GPIO.BOARD)

    printer_port = serial.Serial('/dev/ttyACM0', 115200, timeout=5)

    csv = open(time.strftime('taz_bed_%F-%H%M%S.csv'), 'w')
    csv.write("\"Time\",\"Target temp\",\"Mean measured temp\",\"Mean ADC\",\"ADC stddev\",\"Mean reading\"\n")

    ttb = ThermistorTableBuilder(tc, printer_port, csv)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGHUP, signal_handler)
    print("Signal handlers set up")

    ttb.run(*[int(x) for x in sys.argv[1:]])