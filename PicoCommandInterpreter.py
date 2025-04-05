from machine import Pin, PWM
import sys
import time
import math

rev_pulse = 1100 * 1000
stop_pulse = 1500 * 1000
fwd_pulse_raw = 1900 * 1000 # dont use this one, it's output can't be replicated in reverse
rev_adj = 1 # thrusters are more powerful in fwd direction
fwd_pulse = int(fwd_pulse_raw * rev_adj)
frequency = 10
pwm_file = "pwm_file.csv"

zero_set = [0 for i in range(8)]
stop_set = [stop_pulse for i in range(8)]

fwd_set = [stop_pulse for i in range(4)] + [fwd_pulse, rev_pulse, fwd_pulse, rev_pulse]
crab_set = [stop_pulse for i in range(4)] + [fwd_pulse, fwd_pulse, rev_pulse, rev_pulse] 
down_set =  [fwd_pulse, rev_pulse, fwd_pulse, rev_pulse] + [stop_pulse for i in range(4)]

barrell = [fwd_pulse, fwd_pulse, fwd_pulse, fwd_pulse] + [stop_pulse for i in range(4)]
summer = [rev_pulse, fwd_pulse, fwd_pulse, rev_pulse ] + [stop_pulse for i in range(4)]
spin_set = [stop_pulse for i in range(4)] + [fwd_pulse for i in range(4)]

torpedo = [fwd_pulse, fwd_pulse, fwd_pulse, fwd_pulse] + [fwd_pulse, rev_pulse, fwd_pulse, rev_pulse]



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
        print("Pin: ", self._machinePWM, " Frequency: ", frequency)
        self._machinePWM.freq(frequency)

    def setPWM(self, pulseWidth:int):
        # print("Pin ", self._pinNumber, " set to ", pulseWidth)
        print(self._machinePWM)
        print(pulseWidth)
        self._machinePWM.duty_ns(pulseWidth)



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
    

class Plant:
    def __init__(self):
                # Thruster positions
        self.thruster_positions = [
            [ 0.2535, -0.2035,  0.042 ],
            [ 0.2535,  0.2035,  0.042 ],
            [-0.2545, -0.2035,  0.042 ],
            [-0.2545,  0.2035,  0.042 ],
            [ 0.1670, -0.1375, -0.049 ],
            [ 0.1670,  0.1375, -0.049 ],
            [-0.1975, -0.1165, -0.049 ],
            [-0.1975,  0.1165, -0.049 ]]

        # Thruster directions
        sin45 = math.sin(math.pi / 4)
        self.thruster_directions = [
            [  0,     0,    1  ],
            [  0,     0,   -1  ],
            [  0,     0,    1  ],
            [  0,     0,   -1  ],
            [-sin45, -sin45,  0  ],
            [ sin45, -sin45,  0  ],
            [-sin45,  sin45,  0  ],
            [ sin45, sin45,   0  ]]

        # Thruster torques
        self.thruster_torques = [self.cross_product(self.thruster_positions[i], self.thruster_directions[i]) for i in range(8)]

        # Compute wrench matrix (6x8)
        self.wrench_matrix_transposed = [[0] * 6 for _ in range(8)]
        for i in range(8):
            self.wrench_matrix_transposed[i][0:3] = self.thruster_directions[i]  
            self.wrench_matrix_transposed[i][3:6] = self.thruster_torques[i]     

        # Transpose to get wrench matrix (6x8)
        self.wrench_matrix = self.transpose_matrix(self.wrench_matrix_transposed)

    def pwm_force_scalar(self, x):
        x = x / 1000
        if 1100 <= x < 1460:
            force = (-1.24422882971549e-8) * x**3 + (4.02057100632393e-5) * x**2 - 0.0348619861030835 * x + 3.90671429105423
        elif 1460 <= x <= 1540:
            force = 0
        elif 1540 < x <= 1900:
            force = (-1.64293565374284e-8) * x**3 + (9.45962838560648e-5) * x**2 - 0.170812079190679 * x + 98.7232373648272
        else:
            raise ValueError("PWM value out of valid range (1100-1900)")
        return force
    
    def pwm_force(self, pwm_set):
        thruster_forces = [self.pwm_force_scalar(pwm_set[i]) for i in range(len(pwm_set))]
        force = self.matrix_vector_multiply(self.wrench_matrix, thruster_forces)
        print(force)
    
    @staticmethod
    def transpose_matrix(matrix):
        """Transposes a 2D list (matrix)."""
        return [[row[i] for row in matrix] for i in range(len(matrix[0]))]

    @staticmethod
    def matrix_vector_multiply(matrix, vector):
        """Multiplies a matrix (list of lists) by a vector (list)."""
        return [sum(matrix[i][j] * vector[j] for j in range(len(vector))) for i in range(len(matrix))]
    
    @staticmethod
    def cross_product(a, b):
        """Computes the cross product of two 3D vectors a and b."""
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]

class Thrust_Control:
    def __init__(self, frequency=10):

        self.plant = Plant()
        # Define PWM pins for each thruster
        pins = [
            PWMPin(4, "Unknown thruster"),
            PWMPin(5, "Unknown thruster"),
            PWMPin(2, "Unknown thruster"),
            PWMPin(3, "Unknown thruster"),
            PWMPin(9, "Unknown thruster"),
            PWMPin(7, "Unknown thruster"),
            PWMPin(8, "Unknown thruster"),
            PWMPin(6, "Unknown thruster")]
        self.thrusters = ThrusterMap(pins)
        
        # Set default frequency and duty cycle
        for i in range(self.thrusters.length()):
            self.thrusters.setFrequencyByIndex(i, frequency)
            self.thrusters.setPwmByIndex(i, 0)

        
    def pwm(self, pwm_set):
        
        if (len(pwm_set) != self.thrusters.length()):
            print("Wrong length for pwm set\n")
            return
        
        pwm_set = [int(i) for i in pwm_set]

        log_file = open(pwm_file, 'a')
        start = str(time.time_ns())
        for i in range(len(pwm_set)):
            self.thrusters.setPwmByIndex(i, pwm_set[i])
        end = str(time.time_ns())
        
        string = start + "," + end + "," + ",".join(map(str, pwm_set)) + "\n"
        log_file.write(string)
        print(string)
        log_file.close()

    def singlePwm(self, pinNumber, pwmValue):
        log_file = open(pwm_file, 'a')
        start = str(time.time_ns())
        self.thrusters.setPWMByPin(pinNumber, pwmValue)
        end = str(time.time_ns())
        
        string = start + "," + end + "," + str(pwmValue) + "\n"
        log_file.write(string)
        print(string)
        log_file.close()

        
        
    
    def scaled_pwm(self, pwm_set, scale):
        
        
        new_pwm = [ scale * (i - stop_pulse) + stop_pulse for i in pwm_set]
        
        #if scale < 0:
        #    signs = [-1 * new_pwm[i] / abs(new_pwm[i]) for i in range(len(new_pwm))]
        #    new_pwm = [(new_pwm[i] - stop_pulse) * rev_adj**signs[i] + stop_pulse for i in range(len(new_pwm))]
        
        return new_pwm

    
    def timed_pwm(self, time_s, pwm_set, scale = 1):
        if scale != 1:
            pwm_set = self.scaled_pwm(pwm_set, scale)
            
            
        self.pwm(pwm_set)
        time.sleep(time_s)
        self.pwm(stop_set)
        
    def read(self):
        logf = open(pwm_file, "r")
        file_contents = logf.read()
        print(file_contents)
        logf.close()
                
    def reaction(self, pwm_set, scale=1):
        pwm = [ scale * (i - stop_pulse) + stop_pulse for i in pwm_set]
        self.plant.pwm_force(pwm)

def configurePin(pinNumber, words):
    mode = words[0]
    if mode == "HardPwm":
        print(f"Configuring pin {pinNumber} to Hardware PWM.")
    elif mode == "SoftPwm":
        print(f"Configuring pin {pinNumber} to Software PWM.")
    elif mode == "Digital":
        print(f"Configuring pin {pinNumber} to digital pin.")

def setPinState(pinNumber, words, thrusterControl:Thrust_Control):
    mode = words[0]
    if mode == "PWM":
        pulseWidth = int(words[1])
        if (1100 <= pulseWidth <= 1900):
            thrusterControl.singlePwm(pinNumber, pulseWidth)
    elif mode == "Digital":
        digitalPinState= words[1]
        if (digitalPinState == "Low" or digitalPinState == "High"):
            print(f"Setting pin {pinNumber} to digital pin state {digitalPinState}.")
        
tc = Thrust_Control()
tc.pwm(zero_set)
tc.pwm(torpedo)

# uart = machine.UART(1, 115200)
# uart.init(115200, bits=8, parity=None, stop=1)
# while True:
#     # string = uart.readline()
#     string = sys.stdin.readline()
#     if (string != None and len(string) >= 1):
#         words = string.split()
#         command = words[0]
#         if words[0] == "Configure":
#             configurePin(words[1], words[2:])
#         elif words[0] == "Set":
#             setPinState(words[1], words[2:], tc)
#         elif words[0] == "Exit":
#             break
