import math
from util import clamp

import wpilib
import wpilib.drive
import wpimath.controller
import ctre
import rev

from networktables import NetworkTables
from wpimath.controller import PIDController
from collections import namedtuple

# Create the structure of the config: SmartDashboard prefix, Encoder's zero point, Drive motor inverted, Allow reverse
ModuleConfig = namedtuple('ModuleConfig', ['sd_prefix', 'zero', 'inverted', 'allow_reverse'])

MAX_VOLTAGE = 5 # Absolute encoder measures from 0V to 5V

class SwerveModule:

    def __init__(self, _driveMotor, _rotateMotor, _encoder, _config):
        
        self.driveMotor = _driveMotor
        self.rotateMotor = _rotateMotor
        self.cfg = _config

        self.encoder = _encoder
        # Config -- change this to reflect how our config is formatted. We will upon testing of the entire drivetrain figure out which need to be inverted.
        self.sd_prefix = self.cfg.sd_prefix or 'Module'
        self.encoder_zero = self.cfg.zero or 0 #idk the point of this, maybe useful for other encoder type
        self.inverted = self.cfg.inverted or False 
        self.allow_reverse = self.cfg.allow_reverse or True #def allow reverse always, so you can maybe remove this

        self.moduleFlipped = False

        # SmartDashboard
        self.sd = NetworkTables.getTable('SmartDashboard')
        self.debugging = self.sd.getEntry('drive/drive/debugging')

        # Motor
        self.driveMotor.setInverted(self.inverted)
        self.rotateMotor.setInverted(False)

        self._requested_angle = 0 # change this to something like 'requested angle' or 'requested encoder value', whatever makes more sense
        self._requested_speed = 0 #class variable which execute() passes to the drive motor at the end of the robot loop

        # PID Controller
        # kP = 1.5, kI = 0.0, kD = 0.0
        self._pid_controller = PIDController(0.005, 0.00001, 0.00001) #swap this stuff for CANSparkMax pid controller -- see example from last year shooter
        self._pid_controller.enableContinuousInput(0, 360)
        self._pid_controller.setTolerance(0.5, 0.5) # may need to tweak this with PID testing

        self.sd.putNumber('kP', self._pid_controller.getP())
        self.sd.putNumber('kI', self._pid_controller.getI())
        self.sd.putNumber('kD', self._pid_controller.getD())
        

    def get_current_angle(self):
        """
        :returns: the voltage position after the zero
        """
        angle = (self.encoder.getAbsolutePosition() - self.encoder_zero) % 360

        if self.moduleFlipped:
            angle = (angle + 180) % 360

        return angle

    def flush(self): # rewrite this (although it isnt used anywhere) to reset the encoder to 0 and zero out the speed, if you want.
        """
        Flush the modules requested speed and voltage.
        Resets the PID controller.
        """
        self._requested_angle = self.encoder_zero
        self._requested_speed = 0
        self._pid_controller.reset()

    
    @staticmethod
    def voltage_to_degrees(voltage): #make this like TICKS TO DEG or smth
        """
        Convert a given voltage value to degrees.
        :param voltage: a voltage value between 0 and 5
        :returns: the degree value between 0 and 359
        """
        deg = (voltage / 5) * 360

        if deg < 0:
            deg += 360

        return deg

    @staticmethod
    def voltage_to_rad(voltage): #same, but TICKS TO RAD
        """
        Convert a given voltage value to rad.
        :param voltage: a voltage value between 0 and 5
        :returns: the radian value betwen 0 and 2pi
        """
        return (voltage / 5) * 2 * math.pi

    @staticmethod
    def degree_to_voltage(degree): #DEG TO TICK
        """
        Convert a given degree to voltage.
        :param degree: a degree value between 0 and 360
        :returns" the voltage value between 0 and 5
        """
        return (degree / 360) * 5

    def _set_deg(self, value): #This one weird. Dont do mod stuff if possible - it messes up PID calculations
        """
        Round the value to within 360. Set the requested rotate position (requested voltage).
        Intended to be used only by the move function.
        """
        self._requested_angle = value % 360

    def move(self, speed, deg): #this is all good mostly
        """
        Set the requested speed and rotation of passed.
        :param speed: requested speed of wheel from -1 to 1
        :param deg: requested angle of wheel from 0 to 359 (Will wrap if over or under)
        """
        # deg %= 360 # mod 360, may want to change

        if self.allow_reverse: #addresses module-flipping
            """
            If the difference between the requested degree and the current degree is
            more than 90 degrees, don't turn the wheel 180 degrees. Instead reverse the speed.
            """
            diff = abs(deg - self.get_current_angle())

            if (diff > 180):
                diff = 360 - diff

            if diff > 90: #make this with the new tick-degree methods
                self.moduleFlipped = not self.moduleFlipped
                
            if self.moduleFlipped:
                speed *= -1
                #deg += 180
                #deg %= 360

        self._requested_speed = speed 
        self._set_deg(deg)

    def debug(self): #can use logging/SD if useful
        """
        Print debugging information about the module to the log.
        """
        print(self.sd_prefix, '; requested_speed: ', self._requested_speed, ' requested_angle: ', self._requested_angle)

    def execute(self):
        """
        Use the PID controller to get closer to the requested position.
        Set the speed requested of the drive motor.
        Called every robot iteration/loop.
        """

        self._pid_controller.setP(self.sd.getNumber('kP', 0))
        self._pid_controller.setI(self.sd.getNumber('kI', 0))
        self._pid_controller.setD(self.sd.getNumber('kD', 0))

        # Calculate the error using the current voltage and the requested voltage.
        # DO NOT use the #self.get_voltage function here. It has to be the raw voltage.
        error = self._pid_controller.calculate(self.get_current_angle(), self._requested_angle) #Make this an error in ticks instead of voltage

        # Set the output 0 as the default value
        output = 0
        # If the error is not tolerable, set the output to the error.

        # Else, the output will stay at zero.
        if not self._pid_controller.atSetpoint():
            # Use max-min to clamped the output between -1 and 1. The CANSparkMax PID controller does this automatically, so idk if this is necessary
            output = clamp(error)

        print('ERROR = ' + str(error) + ', OUTPUT = ' + str(output))

        # Put the output to the dashboard
        self.sd.putNumber('drive/%s/output' % self.sd_prefix, output)
        # Set the output as the rotateMotor's voltage
        self.rotateMotor.set(output) # will replace this with a set SETPOINT rather than actually setting the speed
        #SparkMax PID controller will take care of actually running the motors with PID values you instantiate it with

        # Set the requested speed as the driveMotor's voltage
        self.driveMotor.set(self._requested_speed)

        self.update_smartdash()

    def testMove(self, driveInput, rotateInput):
        self.driveMotor.set(clamp(driveInput))
        self.rotateMotor.set(clamp(rotateInput))

    def update_smartdash(self):
        """
        Output a bunch on internal variables for debugging purposes.
        """
        self.sd.putNumber('drive/%s/degrees' % self.sd_prefix, self.get_current_angle())

        if self.debugging.getBoolean(False):

            self.sd.putNumber('drive/%s/requested_speed' % self.sd_prefix, self._requested_speed)
            self.sd.putNumber('drive/%s/encoder position' % self.sd_prefix, self.encoder.getAbsolutePosition())
            self.sd.putNumber('drive/%s/encoder_zero' % self.sd_prefix, self.encoder_zero)

            self.sd.putNumber('drive/%s/PID Setpoint' % self.sd_prefix, self._pid_controller.getSetpoint())
            self.sd.putNumber('drive/%s/PID Error' % self.sd_prefix, self._pid_controller.getPositionError())
            self.sd.putBoolean('drive/%s/PID isAligned' % self.sd_prefix, self._pid_controller.atSetpoint())

            self.sd.putBoolean('drive/%s/allow_reverse' % self.sd_prefix, self.allow_reverse)