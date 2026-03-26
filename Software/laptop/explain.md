Serial Communications Logic: serial_comms.py
The Problem
Reading from a serial port is slow and blocking. The function ser.readline() sits and waits until data arrives.

The Risk: If executed on the main thread, your entire GUI freezes while waiting for the ESP32. Buttons won't respond and sliders won't move.

The Solution: Read serial data on a background thread and use a callback to notify the GUI when new data arrives.

What is a Callback?
A callback is a function you hand to another part of the program to be executed later.

Analogy: Think of it like leaving your phone number with a restaurant: "Call me when my table is ready." You don't stand at the door waiting; you go do other things, and they call you when there is an update.

Implementation in main.py
You "leave your number" with the serial connection by passing the function as an argument:

Python
# This function handles incoming messages
def on_serial_receive(self, msg):    
    print(msg)

# Registering the callback
self.serial.set_callback(self.on_serial_receive)
Implementation in serial_comms.py
The serial module simply stores that function for future use:

Python
def set_callback(self, callback):    
    # Stores the function reference for later
    self._on_receive = callback    
    # Now self._on_receive points to on_serial_receive
How _read_loop Uses the Callback
The _read_loop runs continuously on a background thread. Here is the logic in plain English:

Python
def _read_loop(self):
    while self._running:                          # Keep looping until told to stop
        if self.ser and self.ser.in_waiting:       # Is there data waiting in the USB buffer?
            raw_line = self.ser.readline()         # YES -> read it and decode it
            msg_object = parse_response(raw_line)  # Turn "FAULT:1:0" into FaultMessage object
            self._on_receive(msg_object)           # Call the GUI's function with the data
        else:
            time.sleep(0.01)                       # NO -> sleep 10ms, then check again
When self._on_receive(msg_object) is fired, it is effectively executing:
on_serial_receive(FaultMessage(channel=1, ok=False))

The Full Flow (End-to-End)
Here is exactly what happens when the ESP32 sends FAULT:1:0\n:

ESP32 sends bytes over USB.

Background thread (_read_loop) is spinning and checking ser.in_waiting.

ser.in_waiting > 0 triggers, so it calls ser.readline().

The loop gets "FAULT:1:0" and passes it to parse_response().

parse_response returns FaultMessage(channel=1, ok=False).

The loop calls self._on_receive(msg), which maps to on_serial_receive(msg).

on_serial_receive calls self.after(0, self._handle_message, msg).

Main GUI Thread runs _handle_message and updates the fault label to red.

[!IMPORTANT]
The self.after(0, ...) step is critical. It safely transitions execution from the background thread back to the main GUI thread. Tkinter (and most GUI frameworks) will crash or behave erratically if you attempt to update widgets directly from a background thread.