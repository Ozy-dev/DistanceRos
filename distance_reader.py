import serial
import time

def read_distance_data(port='/dev/ttyUSB0', baudrate=9600):
    """
    Reads and prints raw distance data from a sensor via serial connection.
    
    Args:
        port (str): Serial port where the sensor is connected.
        baudrate (int): Baud rate for the serial connection.
    """
    try:
        # Membuka koneksi serial ke sensor
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=2  # Timeout untuk membaca data
        )
        print(f"Connected to sensor at {port}")

        time.sleep(2)  # Tunggu hingga sensor stabil

        while True:
            # Membaca data dari sensor
            raw_data = ser.readline().decode('utf-8').strip()

            # Validasi data
            if raw_data:
                print("Raw Distance Data:", raw_data)  # Cetak data jarak mentah
            else:
                print("No data received. Check the sensor connection.")
    except serial.SerialException as e:
        print(f"Error connecting to sensor: {e}")
    except Exception as ex:
        print(f"An error occurred: {ex}")

if __name__ == "__main__":
    read_distance_data()
