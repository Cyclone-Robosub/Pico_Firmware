from machine import Pin, PWM
import sys
import time

class PWMPin:
    def __init__(self, pinNumber:int, identifier:str):
        self._pinNumber = pinNumber
        self._identifier = identifier
        self._machinePWM = PWM(Pin(pinNumber))
    
    def pinNumber(self):
        return self._pinNumber

    def identifier(self):
        return self._identifier
    
    def setFrequency(self, frequency:int):
        self._machinePWM.freq(frequency)

    def setPWM(self, pulseWidth:int):
        self._machinePWM.duty_ns(pulseWidth * 1000)


class ThrusterMap:
    def __init__(self, PWMPins):
        self._PWMMap = {}
        self._indexMap = {}
        for i in range(len(PWMPins)):
            self._PWMMap[PWMPins[i].pinNumber()] = PWMPins[i]
            self._indexMap[i] = PWMPins[i].pinNumber()
        

    def length(self):
        return len(self._PWMMap)
    
    def setPWMByPin(self, pinNumber:int, PWMValue:int):
        self._PWMMap[pinNumber].setPWM(PWMValue)

    def setFrequencyByPin(self, pinNumber:int, frequency:int):
        self._PWMMap[pinNumber].setFrequency(frequency)

    def setPwmByIndex(self, index:int, PWMValue:int):
        self.setPWMByPin(self._indexMap[index], PWMValue)

    def setFrequencyByIndex(self, index:int, frequency:int):
        self.setFrequencyByPin(self._indexMap[index], frequency)
    

class Thrust_Control:
    def __init__(self, frequency=100):

        # Define PWM pins for each thruster
        pins = [
            PWMPin(8, "Thruster 0"),
            PWMPin(9, "Thruster 1"),
            PWMPin(6, "Thruster 2"),
            PWMPin(7, "Thruster 3"),
            PWMPin(13, "Thruster 4"),
            PWMPin(11, "Thruster 5"),
            PWMPin(12, "Thruster 6"),
            PWMPin(10, "Thruster 7")]
        self.thrusters = ThrusterMap(pins)
        
        # Set default frequency and duty cycle
        for i in range(self.thrusters.length()):
            self.thrusters.setFrequencyByIndex(i, frequency)
            self.thrusters.setPwmByIndex(i, 0) # Necessary for thruster init

def setPinState(pinNumber, words, thrusterControl:Thrust_Control):
    mode = words[0]
    if mode == "PWM":
        pulseWidth = int(words[1])
        if ((1100 <= pulseWidth <= 1900) or pulseWidth == 0):
            thrusterControl.thrusters.setPWMByPin(int(pinNumber), int(pulseWidth))
    elif mode == "Digital":
        digitalPinState= words[1]
        if (digitalPinState == "Low" or digitalPinState == "High"):
            # print(f"Setting pin {pinNumber} to digital pin state {digitalPinState}.")
            return

def softwareCrash(tc:Thrust_Control):
    current_time = str(time.time_ns())
    crash_log = "Crashes/Crash_at_" + current_time
    
    with open(crash_log, "w") as crash_file:
        crash_file.write(f"Code crash, occuring at {current_time}")
    for i in range(8):
        tc.thrusters.setPwmByIndex(i, 0)
    
    while True:
        ...

def createKillSwitchCallbackHardware(tc:Thrust_Control):
    def killSwitchGPIOHardware(pin):
        for i in range(8):
            tc.thrusters.setPwmByIndex(i, 0)
        current_time = str(time.time_ns())
        crash_log = "Crashes/Crash_at_" + current_time
        with open(crash_log, "w") as crash_file:
            crash_file.write(f"Hardware killswitch triggered, occuring at {current_time}")
        while True:
            if pin.value() == 1:
                machine.reset()

    return killSwitchGPIOHardware

def createKillSwitchCallbackSoftware(tc:Thrust_Control):
    def killSwitchGPIOSoftware(pin):
        for i in range(8):
            tc.thrusters.setPwmByIndex(i, 0)
        current_time = str(time.time_ns())
        crash_log = "Crashes/Crash_at_" + current_time
        with open(crash_log, "w") as crash_file:
            crash_file.write(f"Software killswitch triggered, occuring at {current_time}")
        while True:
            if pin.value() == 1:
                machine.reset()

    return killSwitchGPIOSoftware
        
def start(tc: Thrust_Control):
    for i in range(8):
        tc.thrusters.setPwmByIndex(i, 1500)
    # uart = machine.UART(1, 115200)
    # uart.init(115200, bits=8, parity=None, stop=1)
    led = Pin("LED", Pin.OUT)
    echo = False
    while True:
        # string = uart.readline()
        string = sys.stdin.readline()
        if (string != None and len(string) > 1):
            words = string.split()
            command = words[0]
            if len(words) == 2:
                if command == "echo":
                    if words[1] == "on":
                        led.toggle()
                        echo = True
                    elif words[1] == "off":
                        led.toggle()
                        echo = False
            if echo:
                sys.stdout.buffer.write(string)
            if len(words) > 2:
                led.toggle()
                if command == "Set":
                    setPinState(words[1], words[2:], tc)
                elif command == "Exit":
                    break

tc = Thrust_Control()
killPinHardware = Pin(15, mode=Pin.IN, pull=Pin.PULL_UP)
killPinSoftware = Pin(16, mode=Pin.IN, pull=Pin.PULL_UP)
if killPinHardware.value() == 0:
    hardwareKillFn = createKillSwitchCallbackHardware(tc)
    hardwareKillFn(killPinHardware)
if killPinSoftware.value() == 0:
    softwareKillFn = createKillSwitchCallbackSoftware(tc)
    softwareKillFn(killPinSoftware)
killPinHardware.irq(trigger=Pin.IRQ_FALLING, handler=createKillSwitchCallbackHardware(tc))
killPinSoftware.irq(trigger=Pin.IRQ_FALLING, handler=createKillSwitchCallbackSoftware(tc))
time.sleep(0.1) # Ensure that the thrusters get their 0 signal for some amount of time
try:
    start(tc)
except:
    softwareCrash(tc)
