from machine import Pin, PWM, Timer
import sys
import time

most_recent_ping = time.time()

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
            PWMPin(8, "Thruster 0"),  # pico pin 11
            PWMPin(9, "Thruster 1"),  # pico pin 12
            PWMPin(6, "Thruster 2"),  # pico pin 9
            PWMPin(7, "Thruster 3"),  # pico pin 10
            PWMPin(13, "Thruster 4"), # pico pin 17
            PWMPin(11, "Thruster 5"), # pico pin 15
            PWMPin(12, "Thruster 6"), # pico pin 16
            PWMPin(10, "Thruster 7")  # pico pin 14
        ]
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
    time.sleep(1) # Ensure that the thrusters get their 1500 signal for some amount of time
    led = Pin("LED", Pin.OUT)
    global most_recent_ping
    while True:
        string = sys.stdin.readline()
        if (string != None and len(string) > 1):
            words = string.split()
            command = words[0]
            if (len(words) == 1) and (words[0] == "ping"):
                most_recent_ping = time.time()
            if len(words) > 2:
                most_recent_ping = time.time()
                led.toggle()
                if command == "Set":
                    setPinState(words[1], words[2:], tc)
                elif command == "Exit":
                    break

def createHeartbeatCheckCallback(tc:Thrust_Control):
    def heartbeatCheck(dummy):
        if time.time() - most_recent_ping > 1:
            for i in range(8):
                tc.thrusters.setPwmByIndex(i, 1500)
    
    return heartbeatCheck

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

heartbeatCheckCallbackFn = createHeartbeatCheckCallback(tc)
tim = Timer(-1)
tim.init(mode=Timer.PERIODIC, period=1000, callback=heartbeatCheckCallbackFn) # 1000ms

time.sleep(1) # Ensure that the thrusters get their 0 signal for some amount of time
try:
    start(tc)
except:
    softwareCrash(tc)
