import serial
import serial.tools.list_ports
import time
import threading
from protocol import Command, Message, parse_response

'''
Threading = Running multiple things at the same time
In this case, we want to be able to read from the serial port at the same time as the main GUI loop is running.
Without threading, the GUI would freeze whenever we try to read from the serial port

Useful resources:
Serial:
https://www.dfrobot.com/blog-814.html?srsltid=AfmBOoru55T-pd98m-CzB5BqIS1QL50RGA1VI2HIjJdCOiE3hs9ov3of
https://projects.raspberrypi.org/en/projects/nix-python-reading-serial-data
https://github.com/Rad-hi/ESP_Python_Serial

Threading:
https://realpython.com/intro-to-python-threading/
Claude
ChatGPT
Gemini
'''

def find_ports() -> list[str]:
    ports = serial.tools.list_ports.comports()  
    return [port.device for port in ports]  #List of port names, e.g. ["COM3", "COM4"]

class SerialConnection:
    def __init__(self, port:str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self._reader_thread = None
        self._running = False
        self._on_receive = None 
    
    def connect(self) -> bool:
        #Establishes a serial connection to the ESP32 and starts a background thread to continuously read incoming data
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout = 0.1)
            time.sleep(0.5)  # Give ESP32 time to reset after connection
            self._running = True
            self._reader_thread = threading.Thread(target=self._read_loop, daemon=True) #Daemon thread will close when the main program exits 
            self._reader_thread.start()
            return True
        except serial.SerialException as error:
            print(f"Failed to connect: {error}")
            return False
    
    def disconnect(self):
        self._running = False
        if self._reader_thread is not None: #Prevents trying to join a thread that was never started
            self._reader_thread.join(timeout=0.5)   #Waits for the reader thread to finish, but doesn't block indefinitely if it doesn't respond
        if self.ser and self.ser.is_open:   #Checks if port is open before trying to close it
            self.ser.close()
    
    def send(self, command: Command):
        """Send a Command to the ESP32."""
        if self.ser and self.ser.is_open:
            self.ser.write(command.to_bytes())

    #We read serial data on a background thread, and use a callback to tell the GUI when new data arrives
    def set_callback(self, callback):
        """Set the function to call when a line is received.
        callback(line: str) will be called from the reader thread."""
        self._on_receive = callback

    def _read_loop(self):
        #Background thread that reads lines from the serial port and calls the callback
        while self._running:
            try:
                if self.ser and self.ser.in_waiting:
                    raw_line = self.ser.readline().decode("utf-8", errors="replace").strip()
                    if raw_line and self._on_receive:
                        msg_object = parse_response(raw_line)
                        self._on_receive(msg_object) 
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"Error in read loop: {e}")
                break

    @property
    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open and self._running