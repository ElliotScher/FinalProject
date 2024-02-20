# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Elliot Scher                                                 #
# 	Created:      2/20/2024, 5:13:19 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import * # type: ignore

# Brain should be defined by default
brain = Brain()

controller = Controller()

timer = Timer()

class States:
    IDLE = 0
    INIT = 1
    FOLLOW_LINE_ODOMETRY = 2
    FACE_DIRECTION = 3
    FIND_FRUIT = 4
    PICK_FRUIT = 5
    FOLLOW_LINE_ULTRASONIC = 6
    FIND_BASKET = 7
    DUMP_FRUIT = 8
    FIND_LINE = 9
    CHANGE_FRUIT = 10

class FRUIT_TYPE:
    LIME = Signature(3, -5513, -3161, -4337, -3823, -2027, -2925, 2.5, 0)
    LEMON = Signature(1, 3231, 3841, 3536, -3355, -2697, -3026, 5.5, 0)
    TANGERINE = Signature(2, 7447, 8539, 7993, -1977, -1525, -1751, 9, 0)

class DevicePorts:
    
    FL_DRIVE = Ports.PORT20
    FR_DRIVE = Ports.PORT9
    BL_DRIVE = Ports.PORT17
    BR_DRIVE = Ports.PORT10

    GYRO = Ports.PORT5

    LEFT_LIFT = Ports.PORT18
    RIGHT_LIFT = Ports.PORT7

    GATE = Ports.PORT6

    ULTRASONIC = brain.three_wire_port.a
    VISION = Ports.PORT1

    FL_LINE = brain.three_wire_port.g
    FR_LINE = brain.three_wire_port.b

    BUTTON = brain.three_wire_port.d

class Constants:
        
        LOOP_PERIOD_MS = 20
        
        WHEEL_DIAMETER_IN = 4
        WHEEL_CIRCUMFERENCE_IN = WHEEL_DIAMETER_IN * math.pi

        DRIVE_BASE_RADIUS_IN = 7.75
        DRIVE_BASE_CIRCUMFERENCE_IN = DRIVE_BASE_RADIUS_IN * 2 * math.pi

        MOTOR_MAX_SPEED_RPM = 200

        # where phi is the angle between the x/y axis and the wheel vectors, which is always some multiple of pi/4
        SEC_PHI = 2 / math.sqrt(2)

        DRIVE_TRAIN_IDLE_MODE = BRAKE

        DRIVE_TRANSLATION_KP = 0.16

        DRIVE_ROTATION_KP = 0.8

        DRIVE_MAX_SPEED_METERS_PER_SEC = 0.2

        DRIVE_FIND_LINE_SPEED_METERS_PER_SEC = 0.2

        DRIVE_FOLLOW_LINE_SPEED_METERS_PER_SEC = 0.2
        ODOM_TOLERANCE_METERS = 0.0025

        ODOM_X_POSITIVE_DRIFT = 194 / 200
        ODOM_X_NEGATIVE_DRIFT = 207 / 200
        ODOM_Y_POSITIVE_DRIFT = 180 / 200
        ODOM_Y_NEGATIVE_DRIFT = 211 / 200

        ODOM_Y_DRIFT_PER_POSITIVE_X_TRANSLATION = 5 / 200
        ODOM_Y_DRIFT_PER_NEGATIVE_X_TRANSLATION = -17 / 200

        LINE_REFLECTIVITY_THRESHOLD = 0.5

        FOLLOW_LINE_KP = 0
        FOLLOW_LINE_KD = 0


        LEFT_LINE_Y_METERS = 0 # FIXME
        RIGTH_LINE_Y_METERS = 0 # FIXME

        LIFT_SPEED = 150

        BRIGHTNESS = 41

        FRUIT_CENTERING_TOLERANCE = 0.01



class Odometry:

    def __init__(self, x = 0.0, y = 0.0, theta = 0.0):
        self.xMeters = x
        self.yMeters = y
        self.thetaRad = theta
        
    prevTranslationMeters = 0.0
    def update(self, fieldOrientedTranslationRad: float, translationMeters: float, headingRad: float):
        translationDeltaMeters = abs(translationMeters - self.prevTranslationMeters)
        self.prevTranslationMeters = translationMeters

        # As the wheels approach parallel/orthoginal to the drive direction, we don't need to compensate for carpet drift
        robotRelativeTranslationRad = fieldOrientedTranslationRad + headingRad
        driftCompPercentage = abs(math.cos((2 * robotRelativeTranslationRad) % math.pi))

        xDelta = translationDeltaMeters * math.cos(fieldOrientedTranslationRad)
        yDelta = translationDeltaMeters * math.sin(fieldOrientedTranslationRad)

        if(yDelta > 0.0):
            yDelta *= (Constants.ODOM_Y_POSITIVE_DRIFT ** driftCompPercentage)
        else:
            yDelta *= (Constants.ODOM_Y_NEGATIVE_DRIFT ** driftCompPercentage)

        if(xDelta > 0.0):
            xDelta *= (Constants.ODOM_X_POSITIVE_DRIFT ** driftCompPercentage)
            yDelta += ((xDelta * Constants.ODOM_Y_DRIFT_PER_POSITIVE_X_TRANSLATION) * driftCompPercentage)
        else:
            xDelta *= (Constants.ODOM_X_NEGATIVE_DRIFT ** driftCompPercentage)
            yDelta += ((xDelta * Constants.ODOM_Y_DRIFT_PER_NEGATIVE_X_TRANSLATION) * driftCompPercentage)

        self.xMeters += xDelta
        self.yMeters += yDelta
        self.thetaRad = headingRad





        
class LineSensorArray:
    __prevError = 0.0

    def __init__(self, leftSensorPort: Triport.TriportPort, rightSensorPort: Triport.TriportPort):
        self.leftSensor = Line(leftSensorPort)
        self.rightSensor = Line(rightSensorPort)
        
        self.periodic()

    def periodic(self):
        timer.event(self.periodic, Constants.LOOP_PERIOD_MS)

        if self.onLine():
            self.__lastSensorOnLine = None
        elif self.rightSensor.reflectivity() > Constants.LINE_REFLECTIVITY_THRESHOLD:
            self.__lastSensorOnLine = RIGHT
        elif self.leftSensor.reflectivity() > Constants.LINE_REFLECTIVITY_THRESHOLD:
            self.__lastSensorOnLine = LEFT

        self.__rate = (self.getError() - self.__prevError) / Constants.LOOP_PERIOD_MS
        self.__prevError = self.getError()

    def hasLine(self):
        '''Line is detected by at least one sensor'''
        return self.rightSensor.reflectivity() > Constants.LINE_REFLECTIVITY_THRESHOLD \
            or self.leftSensor.reflectivity() > Constants.LINE_REFLECTIVITY_THRESHOLD
    
    def onLine(self):
        '''Line is detected by both sensors'''
        return self.rightSensor.reflectivity() > Constants.LINE_REFLECTIVITY_THRESHOLD \
            and self.leftSensor.reflectivity() > Constants.LINE_REFLECTIVITY_THRESHOLD

    def getError(self) -> float:
        return self.rightSensor.reflectivity() - self.leftSensor.reflectivity()
    
    def getRate(self) -> float:
        return self.__rate
    
    def getLastSensorOnLine(self):
        '''Returns RIGHT for right sensor, LEFT for leftsensor, or None if they are both on the line'''
        return self.__lastSensorOnLine




class Vision:
    vision = Vision(DevicePorts.VISION, Constants.BRIGHTNESS, FRUIT_TYPE.LEMON, FRUIT_TYPE.LEMON, FRUIT_TYPE.TANGERINE)
    currentSnapshot = None
    previousSnapshot = None
    currentFruit = None
    def __init__(self) -> None:
        self.periodic()

    def periodic(self):
        self.previousSnapshot = self.currentSnapshot
        self.currentSnapshot = self.vision.take_snapshot(self.currentFruit)
        timer.event(self.periodic, Constants.LOOP_PERIOD_MS)

    def changeFruit(self, fruit: Signature):
        self.currentFruit = fruit

    def hasFruit(self):
        return self.currentSnapshot

    def getXOffset(self):
        return (160 - self.vision.largest_object().centerX)

    def getYOffset(self):
        return self.vision.largest_object().centerY


class Drive:

    flDrive = Motor(DevicePorts.FL_DRIVE)
    frDrive = Motor(DevicePorts.FR_DRIVE)
    blDrive = Motor(DevicePorts.BL_DRIVE)
    brDrive = Motor(DevicePorts.BR_DRIVE)

    flDrive.set_stopping(Constants.DRIVE_TRAIN_IDLE_MODE)
    frDrive.set_stopping(Constants.DRIVE_TRAIN_IDLE_MODE)
    blDrive.set_stopping(Constants.DRIVE_TRAIN_IDLE_MODE)
    brDrive.set_stopping(Constants.DRIVE_TRAIN_IDLE_MODE)

    gyro = Inertial(DevicePorts.GYRO)

    odometry = Odometry(0.0, 0.0, 0.0)

    def __init__(self, heading = 0.0, calibrateGyro = False):
        if calibrateGyro:
            self.gyro.calibrate()
            while self.gyro.is_calibrating():
                pass
            print("GYRO CALIBRATED")
            self.gyro.set_heading(heading)
        
        self.periodic()

    @staticmethod
    def __revolutionsToMeters(revolutions) -> float:
        return (revolutions * Constants.WHEEL_CIRCUMFERENCE_IN) / (39.3701)

    @staticmethod
    def __metersPerSecToRPM(speedMetersPerSec) -> float:
        return (speedMetersPerSec * 39.3701 * 60) / (Constants.WHEEL_CIRCUMFERENCE_IN)
    
    @staticmethod
    def __rpmToMetersPerSecond(speedMetersPerSec) -> float:
        return (speedMetersPerSec * Constants.WHEEL_CIRCUMFERENCE_IN) / (39.3701 * 60)
    
    @staticmethod
    def __radPerSecToRPM(speedRadPerSec) -> float:
        return (speedRadPerSec * Constants.WHEEL_CIRCUMFERENCE_IN * 60) / (2 * math.pi * Constants.WHEEL_CIRCUMFERENCE_IN)

    def periodic(self):
        timer.event(self.periodic, Constants.LOOP_PERIOD_MS)

        self.odometry.update(
            self.getActualDirectionOfTravelRad(),
            self.getDistanceTraveledMeters(),
            self.gyro.heading(RotationUnits.REV) * 2 * math.pi
        )

    def applyDesaturated(self, flSpeedRPM, frSpeedRPM, blSpeedRPM, brSpeedRPM):
        fastestSpeedRPM = max(abs(flSpeedRPM), abs(frSpeedRPM), abs(blSpeedRPM), abs(brSpeedRPM))
        if(fastestSpeedRPM > Constants.MOTOR_MAX_SPEED_RPM):
            ratio = Constants.MOTOR_MAX_SPEED_RPM / fastestSpeedRPM

            flSpeedRPM *= ratio
            frSpeedRPM *= ratio
            blSpeedRPM *= ratio
            brSpeedRPM *= ratio
        
        self.flDrive.spin(FORWARD, round(flSpeedRPM, 4))
        self.frDrive.spin(FORWARD, round(frSpeedRPM, 4))
        self.blDrive.spin(FORWARD, round(blSpeedRPM, 4))
        self.brDrive.spin(FORWARD, round(brSpeedRPM, 4))

    def stop(self):
        self.flDrive.stop()
        self.frDrive.stop()
        self.blDrive.stop()
        self.brDrive.stop()

    def applySpeeds(self, directionRad: float, translationSpeedMetersPerSec: float, rotationSpeedRadPerSec: float, fieldOriented = True):
        translationRPM = self.__metersPerSecToRPM(translationSpeedMetersPerSec)
        rotationRPM = -self.__radPerSecToRPM(rotationSpeedRadPerSec)

        # offset lateral direction by gyro heading for field-oriented control
        if fieldOriented:
            directionRad += self.gyro.heading(RotationUnits.REV) * 2 * math.pi

        # find the x and y components of the direction vector mapped to a wheel vector
        coeffRPM = Constants.SEC_PHI * translationRPM
        xProjectionRPM = coeffRPM * math.sin(directionRad)
        yProjectionRPM = coeffRPM * math.cos(directionRad)

        self.applyDesaturated(
            rotationRPM - (xProjectionRPM - yProjectionRPM),
            rotationRPM - (xProjectionRPM + yProjectionRPM),
            rotationRPM + (xProjectionRPM + yProjectionRPM),
            rotationRPM + (xProjectionRPM - yProjectionRPM)
        )

    def getActualDirectionOfTravelRad(self, fieldOriented = True) -> float:
        xFL = self.flDrive.velocity() * math.cos(7 * math.pi / 4)
        yFL = self.flDrive.velocity() * math.sin(7 * math.pi / 4)

        xFR = self.blDrive.velocity() * math.cos(math.pi / 4)
        yFR = self.blDrive.velocity() * math.sin(math.pi / 4)
        
        xBL = self.frDrive.velocity() * math.cos(5 * math.pi / 4)
        yBL = self.frDrive.velocity() * math.sin(5 * math.pi / 4)

        xBR = self.brDrive.velocity() * math.cos(3 * math.pi / 4)
        yBR = self.brDrive.velocity() * math.sin(3 * math.pi / 4)

        xSumVectors = xFL + xFR + xBL + xBR
        ySumVectors = yFL + yFR + yBL + yBR

        direction = math.atan2(ySumVectors, xSumVectors)

        if fieldOriented:
            direction -= self.gyro.heading(RotationUnits.REV) * 2 * math.pi

        return direction
    
    def getDistanceTraveledMeters(self):
        xFL = self.__revolutionsToMeters(self.flDrive.position()) * math.cos(7 * math.pi / 4)
        yFL = self.__revolutionsToMeters(self.flDrive.position()) * math.sin(7 * math.pi / 4)

        xFR = self.__revolutionsToMeters(self.blDrive.position()) * math.cos(math.pi / 4)
        yFR = self.__revolutionsToMeters(self.blDrive.position()) * math.sin(math.pi / 4)
        
        xBL = self.__revolutionsToMeters(self.frDrive.position()) * math.cos(5 * math.pi / 4)
        yBL = self.__revolutionsToMeters(self.frDrive.position()) * math.sin(5 * math.pi / 4)

        xBR = self.__revolutionsToMeters(self.brDrive.position()) * math.cos(3 * math.pi / 4)
        yBR = self.__revolutionsToMeters(self.brDrive.position()) * math.sin(3 * math.pi / 4)

        xSumVectors = xFL + xFR + xBL + xBR
        ySumVectors = yFL + yFR + yBL + yBR

        magnitude = magnitude = self.__rpmToMetersPerSecond(math.sqrt(xSumVectors**2 + ySumVectors**2) / 4)

        return magnitude
    
    def getActualSpeedMetersPerSec(self) -> float:
        xFL = self.flDrive.velocity() * math.cos(7 * math.pi / 4)
        yFL = self.flDrive.velocity() * math.sin(7 * math.pi / 4)

        xFR = self.blDrive.velocity() * math.cos(math.pi / 4)
        yFR = self.blDrive.velocity() * math.sin(math.pi / 4)
        
        xBL = self.frDrive.velocity() * math.cos(5 * math.pi / 4)
        yBL = self.frDrive.velocity() * math.sin(5 * math.pi / 4)

        xBR = self.brDrive.velocity() * math.cos(3 * math.pi / 4)
        yBR = self.brDrive.velocity() * math.sin(3 * math.pi / 4)

        xSumVectors = xFL + xFR + xBL + xBR
        ySumVectors = yFL + yFR + yBL + yBR

        magnitude = self.__rpmToMetersPerSecond(math.sqrt(xSumVectors**2 + ySumVectors**2) / 4)

        return magnitude
    
    def driveToPosition(self, xMeters: float, yMeters: float, headingRad: float):
        xError = xMeters - self.odometry.xMeters
        yError = yMeters - self.odometry.yMeters

        xEffort = xError * Constants.DRIVE_TRANSLATION_KP
        yEffort = yError * Constants.DRIVE_TRANSLATION_KP

        direction = math.atan2(yEffort, xEffort)
        magnitude = math.sqrt(xError**2 + yError**2)

        thetaError = headingRad - self.odometry.thetaRad

        if thetaError > math.pi:
            thetaError -= 2 * math.pi
        elif thetaError < -math.pi:
            thetaError += 2 * math.pi

        thetaEffort = thetaError * Constants.DRIVE_ROTATION_KP

        if(abs(magnitude) > Constants.DRIVE_MAX_SPEED_METERS_PER_SEC):
            magnitude = math.copysign(Constants.DRIVE_MAX_SPEED_METERS_PER_SEC, magnitude)

        self.applySpeeds(direction, magnitude, thetaEffort, True)

    def followLine(self, odomXMetersTarget: float, odomLineYMeters: float, speedMetersPerSec: float, lineSensorArray: LineSensorArray, findLineIfOff = True):
        if lineSensorArray.hasLine():
            self.odometry.yMeters = odomLineYMeters

            driveEffort = (odomXMetersTarget - self.odometry.xMeters) * Constants.DRIVE_TRANSLATION_KP
            if(abs(driveEffort) < speedMetersPerSec):
                driveEffort = math.copysign(speedMetersPerSec, driveEffort)

            rotEffort = lineSensorArray.getError() * Constants.FOLLOW_LINE_KP + lineSensorArray.getRate() * Constants.FOLLOW_LINE_KD

            self.applySpeeds(0, driveEffort, rotEffort, False)
        elif findLineIfOff:
            if lineSensorArray.getLastSensorOnLine() == RIGHT:
                self.applySpeeds(0, Constants.DRIVE_FIND_LINE_SPEED_METERS_PER_SEC, 0, True)
            elif lineSensorArray.getLastSensorOnLine() == LEFT:
                self.applySpeeds(0, -Constants.DRIVE_FIND_LINE_SPEED_METERS_PER_SEC, 0, True)



class Lift:
    leftLift = Motor(DevicePorts.LEFT_LIFT, False)
    rightLift = Motor(DevicePorts.RIGHT_LIFT, True)

    ultrasonic = Sonar(DevicePorts.ULTRASONIC)

    LOW_DISTANCE = 0.0
    MID_DISTANCE = 0.0
    HIGH_DISTANCE = 0.0

    PINION_CIRCUMFERENCE_IN = 0.5

    STOW_POSITION = 0.025
    LOW_POSITION = 0.15
    MID_POSITION = 3.0
    HIGH_POSITION = 5.5

    leftLift.set_velocity(Constants.LIFT_SPEED, RPM)
    rightLift.set_velocity(Constants.LIFT_SPEED, RPM)

    def __init__(self):
        self.periodic()

    def periodic(self):
        timer.event(self.periodic, Constants.LOOP_PERIOD_MS)


    def setStowPosition(self):
        self.leftLift.spin_to_position(self.STOW_POSITION, TURNS, False)
        self.rightLift.spin_to_position(self.STOW_POSITION, TURNS, True)

    def setLowPosition(self):
        self.leftLift.spin_to_position(self.LOW_POSITION, TURNS, False)
        self.rightLift.spin_to_position(self.LOW_POSITION, TURNS, True)

    def setMidPosition(self):
        self.leftLift.spin_to_position(self.MID_POSITION, TURNS, False)
        self.rightLift.spin_to_position(self.MID_POSITION, TURNS, True)

    def setHighPosition(self):
        self.leftLift.spin_to_position(self.HIGH_POSITION, TURNS, False)
        self.rightLift.spin_to_position(self.HIGH_POSITION, TURNS, True)

    def setUltrasonicPosition(self):
        if (abs(self.ultrasonic.distance(INCHES) - self.LOW_POSITION) < 0.01):
            self.setLowPosition()
        if (abs(self.ultrasonic.distance(INCHES) - self.MID_POSITION) < 0.01):
            self.setMidPosition()
        if (abs(self.ultrasonic.distance(INCHES) - self.HIGH_POSITION) < 0.01):
            self.setHighPosition()

    def stop(self):
        self.leftLift.stop()




class Gate:
    leftGate = Motor(DevicePorts.GATE, False)


    # DEGREES
    LOCKED_POSITION = 0.0
    UNLOCKED_POSITION = 90.0

    def __init__(self) -> None:
        self.periodic()

    def periodic(self):
        timer.event(self.periodic, Constants.LOOP_PERIOD_MS)

    def setLockedPosition(self):
        self.leftGate.spin_to_position(self.LOCKED_POSITION, DEGREES)

    def setUnlockedPosition(self):
        self.leftGate.spin_to_position(self.UNLOCKED_POSITION, DEGREES)


state = 0

drive = Drive()
lift = Lift()
gate = Gate()
vision = Vision()

frontLine = LineSensorArray(DevicePorts.FL_LINE, DevicePorts.FR_LINE)
button = Bumper(DevicePorts.BUTTON)


def strafe():
    global state
    if (vision.hasFruit()):
        drive.stop()
    else:
        drive.applySpeeds(math.pi / 2, 0.1, 0, True)

def center():
    global state
    Kp = 0.001
    if (vision.hasFruit()):
        error = vision.getXOffset()
    else:
        error = 0
    effort = error * Kp
    print(effort)
    drive.applySpeeds(math.pi / 2, effort, 0, True)



currentState = States.IDLE
rowCount = 0

def handleButton():
    global currentState
    if currentState == States.IDLE:
        currentState = States.INIT
        sleep(500)
    elif currentState == States.INIT:
        currentState = States.FOLLOW_LINE_ODOMETRY
        sleep(500)
    else:
        currentState = States.INIT
        sleep(500)

def IDLE():
    global currentState
    drive.stop()
    lift.stop()

def INIT():
    global currentState
    # init things here

def FOLLOW_LINE_ODOMETRY(line: TurnType.TurnType, odom_x_target: float):
    global currentState
    drive.odometry.yMeters = Constants.LEFT_LINE_Y_METERS if line == LEFT else Constants.RIGTH_LINE_Y_METERS
    drive.followLine(odom_x_target, drive.odometry.yMeters, 0.2, frontLine, True)
    if (abs(drive.odometry.xMeters - odom_x_target) < Constants.ODOM_TOLERANCE_METERS):
        drive.stop()
        currentState = States.FACE_DIRECTION

def FACE_DIRECTION(direction: DirectionType):
    global currentState
    Kp = 1
    target = 0 if direction == DirectionType.FORWARD else math.pi
    error = target - drive.odometry.thetaRad
    effort = error * Kp
    drive.applySpeeds(0, 0, effort, False)
    if (abs(error) < 0.1):
        drive.stop()
        currentState = States.FIND_FRUIT

def FIND_FRUIT(fromLine: TurnType.TurnType):
    global currentState
    direction = (3 * math.pi / 2) if fromLine == LEFT else math.pi / 2
    if (vision.hasFruit()):
        Kp = 0.001
        if (vision.hasFruit()):
            error = vision.getXOffset()
        else:
            error = 0
        effort = error * Kp
        drive.applySpeeds(direction, effort, 0, True)
        if (abs(error < Constants.FRUIT_CENTERING_TOLERANCE)):
            drive.stop()
            currentState = States.PICK_FRUIT
    else:
        drive.applySpeeds(direction, 0.2, 0, True)


def robotPeriodic():
    timer.event(robotPeriodic, Constants.LOOP_PERIOD_MS)
    











robotPeriodic()