'''
This file defines the serial protocol between the laptop GUI and the ESP32 microcontroller. 
All messages are ASCII encoded and terminated with a newline character ('\n')

GUI -> ESP32:
    SOL1:ON\n       -Turn on solenoid 1
    SOL1:OFF\n      -Turn off solenoid 1
    SOL2:ON\n       -Turn on solenoid 2
    SOL2:OFF\n      -Turn off solenoid 2
    SRV1:500\n      -Set servo 1 to 500 (range 500 - 2500 microseconds)
    SRV1:2500\n     -Set servo 1 to 2500
    SRV2:500\n      -Set servo 2 to 500
    SRV2:2500\n     -Set servo 2 to 2500    
    SRV3:500\n      -Set servo 3 to 500 
    SRV3:2500\n     -Set servo 3 to 2500 
    SRV4:500\n      -Set servo 4 to 500 
    SRV4:2500\n     -Set servo 4 to 2500
    STATUS\n        -Request current status

ESP32 -> GUI:
    OK\n                                                                  -Acknowledgement of received command
    ERROR:message\n                                                       -Error message if command is invalid
    FAULT:1:0\n                                                           -nFAULT status (channel:Solenoid value, 0=fault, 1=ok)
    FAULT:2:1\n
    SENSOR:PRESS1:3.45\n                                                  -Pressure in bar
    SENSOR:PRESS2:2.10\n
    SENSOR:TEMP1:85.3\n                                                   -Temperature in °C
    SENSOR:THRUST:142.7\n                                                 -Thrust in N
    STATUS:SOL1:ON:SOL2:OFF:SRV1:1500:SRV2:1500:SRV3:1500:SRV4:1500\n     -Current status of all components
    '''

from dataclasses import dataclass

# GUI -> ESP32

class Command:
    
    def to_bytes(self) -> bytes:
       return self.to_str().encode('utf-8') # Convert the string to bytes using UTF-8 (ASCII) encoding
    
    def to_str(self) -> str:
        raise NotImplementedError("Must implement to_str method in subclass")   #Safeguard to ensure subclasses implement this method

@dataclass
class SolenoidCommand(Command):
    channel: int    # 1 or 2
    state: bool     # True for ON, False for OFF
    
    def to_str(self) -> str:
        state = "ON" if self.state else "OFF"
        return f"SOL{self.channel}:{state}\n"

@dataclass
class ServoCommand(Command):
    """Set a servo to one of three positions.
 
    Hardware: ESP32 LEDC PWM output to FT5330M servo.
        500µs  = fully closed (0°)
        1500µs = centre (90°)
        2500µs = fully open (180°)
    """
    channel: int    # 1 to 4
    pulse_us: int   # 500, 1500, or 2500
 
    def __post_init__(self):
        if not (1 <= self.channel <= 4):
            raise ValueError(f"Servo channel must be 1–4, got {self.channel}")
        if self.pulse_us not in (500, 1500, 2500):
            raise ValueError(f"Pulse must be 500, 1500, or 2500, got {self.pulse_us}")
 
    def to_str(self) -> str:
        return f"SRV{self.channel}:{self.pulse_us}\n"

@dataclass
class StatusCommand(Command):   # Request for the ESP32 to broadcast its current solenoid and servo states.
    def to_str(self) -> str:
        return "STATUS\n"

#ESP32 -> GUI
class Message:
    pass

@dataclass
class FaultMessage(Message):
    channel: int    
    ok: bool    # True for OK, False for FAULT

@dataclass
class SensorMessage(Message):
    name: str   # e.g. "PRESS1", "TEMP1", "THRUST"
    value: float

@dataclass
class AcknowledgementMessage(Message):  
    pass

@dataclass
class ErrorMessage(Message):
    message: str  

@dataclass
class StatusMessage(Message): 
    raw: str

@dataclass
class UnknownMessage(Message):  #Safety net for any messages that don't match the expected formats
    raw: str

#Parser
#This function converts a string from the serial port from the ESP32 into a structured Message object that the GUI can easily work with

def parse_response(line:str) -> Message:
    line = line.strip()

    if line.startswith('FAULT:'):
        parts = line.split(':')
        return FaultMessage(channel = int(parts[1]), ok=parts[2] == '1')    #E.g. FAULT:1:0 -> channel=1, ok=False
    
    elif line.startswith('SENSOR'):
        parts = line.split(':')
        return SensorMessage(name=parts[1], value=float(parts[2]))  #E.g. SENSOR:PRESS1:3.45 -> name="PRESS1", value=3.45
    
    elif line == 'OK':
        return AcknowledgementMessage() 

    elif line.startswith('ERROR:'):
        return ErrorMessage(message = line[6:])  #E.g. ERROR:Invalid command -> message="Invalid command"

    elif line.startswith("STATUS:"):
        return StatusMessage(raw=line)

    else:
        return UnknownMessage(raw=line)