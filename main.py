import time

from src.sensor import PressureSensor

if __name__ == '__main__':
    sensor = PressureSensor(verbose=True)

    def print_data():
        temp_value, pres_value = sensor.read()
        print("Temp: {:0.2f} C  P: {:0.2f} hPa ".format(temp_value, pres_value))

    try:
        while True:
            print_data()
            time.sleep(0.5)
    except KeyboardInterrupt:
        exit(0)
