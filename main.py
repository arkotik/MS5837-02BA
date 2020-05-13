import time
import src.sensor as ms5837

if __name__ == '__main__':
    sensor = ms5837.PressureSensor(verbose=True, osr=ms5837.OSR_2048)

    def print_data():
        temp_value, pres_value = sensor.read()
        print("Temp: {:0.2f} C  P: {:0.2f} hPa ".format(temp_value, pres_value))

    try:
        while True:
            print_data()
            time.sleep(1)
    except KeyboardInterrupt:
        exit(0)
