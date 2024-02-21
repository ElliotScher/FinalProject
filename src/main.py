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
    FACE_FORWARD = 3
    FIND_FRUIT = 4
    PICK_FRUIT = 5
    FACE_BACKWARD = 6
    FOLLOW_LINE_ULTRASONIC = 7
    FIND_BASKET = 8
    DUMP_FRUIT = 9
    FIND_LINE = 10
    CHANGE_FRUIT = 11

class FRUIT_TYPE:
    LIME = Signature(3, -5513, -3161, -4337, -3823, -2027, -2925, 2.5, 0)
    LEMON = Signature(1, 3231, 3841, 3536, -3355, -2697, -3026, 5.5, 0)
    TANGERINE = Signature(2, 7447, 8539, 7993, -1977, -1525, -1751, 9, 0)

class DevicePorts:
    
    FL_DRIVE = Ports.PORT17
    FR_DRIVE = Ports.PORT10
    BL_DRIVE = Ports.PORT20
    BR_DRIVE = Ports.PORT9

    GYRO = Ports.PORT8

    LEFT_LIFT = Ports.PORT18
    RIGHT_LIFT = Ports.PORT7

    GATE = Ports.PORT6

    ULTRASONIC = brain.three_wire_port.a
    VISION = Ports.PORT1

    FL_LINE = brain.three_wire_port.g
    FR_LINE = brain.three_wire_port.b

    BUTTON = brain.three_wire_port.d

class RobotConstants:
        
        LOOP_PERIOD_MS = 20
        
        WHEEL_DIAMETER_IN = 4
        WHEEL_CIRCUMFERENCE_IN = WHEEL_DIAMETER_IN * math.pi

        DRIVE_BASE_RADIUS_IN = 7.75
        DRIVE_BASE_CIRCUMFERENCE_IN = DRIVE_BASE_RADIUS_IN * 2 * math.pi

        MOTOR_MAX_SPEED_RPM = 200

        # where phi is the angle between the x/y axis and the wheel vectors, which is always some multiple of pi/4
        SEC_PHI = 2 / math.sqrt(2)

        DRIVE_TRAIN_IDLE_MODE = BRAKE

        DRIVE_TRANSLATION_KP = 1.5

        DRIVE_ROTATION_KP = 1.5

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

        LIFT_SPEED = 150

        BRIGHTNESS = 41

        FRUIT_CENTERING_TOLERANCE_PX = 15 #px

        APPROACH_DISTANCE = 0.25



class FieldConstants:
    LEFT_LINE_Y_METERS = 2.205
    RIGHT_LINE_Y_METERS = 0.28

    FIRST_ROW_X_METERS = 0.6875


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
            yDelta *= (RobotConstants.ODOM_Y_POSITIVE_DRIFT ** driftCompPercentage)
        else:
            yDelta *= (RobotConstants.ODOM_Y_NEGATIVE_DRIFT ** driftCompPercentage)

        if(xDelta > 0.0):
            xDelta *= (RobotConstants.ODOM_X_POSITIVE_DRIFT ** driftCompPercentage)
            yDelta += ((xDelta * RobotConstants.ODOM_Y_DRIFT_PER_POSITIVE_X_TRANSLATION) * driftCompPercentage)
        else:
            xDelta *= (RobotConstants.ODOM_X_NEGATIVE_DRIFT ** driftCompPercentage)
            yDelta += ((xDelta * RobotConstants.ODOM_Y_DRIFT_PER_NEGATIVE_X_TRANSLATION) * driftCompPercentage)

        self.xMeters += xDelta
        self.yMeters += yDelta
        self.thetaRad = headingRad





class LineSensorArray:
    __prevError = 0.0

    def __init__(self, leftSensorPort: Triport.TriportPort, rightSensorPort: Triport.TriportPort):
        self.leftSensor = Line(leftSensorPort)
        self.rightSensor = Line(rightSensorPort)
        
    def periodic(self):

        if self.onLine():
            self.__lastSensorOnLine = None
        elif self.rightSensor.reflectivity() > RobotConstants.LINE_REFLECTIVITY_THRESHOLD:
            self.__lastSensorOnLine = RIGHT
        elif self.leftSensor.reflectivity() > RobotConstants.LINE_REFLECTIVITY_THRESHOLD:
            self.__lastSensorOnLine = LEFT

        self.__rate = (self.getError() - self.__prevError) / (RobotConstants.LOOP_PERIOD_MS / 1000)
        self.__prevError = self.getError()

    def hasLine(self):
        '''Line is detected by at least one sensor'''
        return self.rightSensor.reflectivity() > RobotConstants.LINE_REFLECTIVITY_THRESHOLD \
            or self.leftSensor.reflectivity() > RobotConstants.LINE_REFLECTIVITY_THRESHOLD
    
    def onLine(self):
        '''Line is detected by both sensors'''
        return self.rightSensor.reflectivity() > RobotConstants.LINE_REFLECTIVITY_THRESHOLD \
            and self.leftSensor.reflectivity() > RobotConstants.LINE_REFLECTIVITY_THRESHOLD

    def getError(self) -> float:
        return self.rightSensor.reflectivity() - self.leftSensor.reflectivity()
    
    def getRate(self) -> float:
        return self.__rate
    
    def getLastSensorOnLine(self):
        '''Returns RIGHT for right sensor, LEFT for leftsensor, or None if they are both on the line'''
        return self.__lastSensorOnLine





class Drive:

    flDrive = Motor(DevicePorts.FL_DRIVE)
    frDrive = Motor(DevicePorts.FR_DRIVE)
    blDrive = Motor(DevicePorts.BL_DRIVE)
    brDrive = Motor(DevicePorts.BR_DRIVE)

    flDrive.set_stopping(RobotConstants.DRIVE_TRAIN_IDLE_MODE)
    frDrive.set_stopping(RobotConstants.DRIVE_TRAIN_IDLE_MODE)
    blDrive.set_stopping(RobotConstants.DRIVE_TRAIN_IDLE_MODE)
    brDrive.set_stopping(RobotConstants.DRIVE_TRAIN_IDLE_MODE)

    gyro = Inertial(DevicePorts.GYRO)

    odometry = Odometry(0.15875, FieldConstants.RIGHT_LINE_Y_METERS, 0.0)

    def __init__(self, heading = 0.0, calibrateGyro = False):
        if calibrateGyro:
            self.gyro.calibrate()
            while self.gyro.is_calibrating():
                pass
            print("GYRO CALIBRATED")
            self.gyro.set_heading(heading)
        

    @staticmethod
    def __revolutionsToMeters(revolutions) -> float:
        return (revolutions * RobotConstants.WHEEL_CIRCUMFERENCE_IN) / (39.3701)

    @staticmethod
    def __metersPerSecToRPM(speedMetersPerSec) -> float:
        return (speedMetersPerSec * 39.3701 * 60) / (RobotConstants.WHEEL_CIRCUMFERENCE_IN)
    
    @staticmethod
    def __rpmToMetersPerSecond(speedMetersPerSec) -> float:
        return (speedMetersPerSec * RobotConstants.WHEEL_CIRCUMFERENCE_IN) / (39.3701 * 60)
    
    @staticmethod
    def __radPerSecToRPM(speedRadPerSec) -> float:
        return (speedRadPerSec * RobotConstants.WHEEL_CIRCUMFERENCE_IN * 60) / (2 * math.pi * RobotConstants.WHEEL_CIRCUMFERENCE_IN)
    
    def calcThetaControlRadPerSec(self, targetHeadingRad: float) -> float:
        thetaError = targetHeadingRad - self.odometry.thetaRad

        if thetaError > math.pi:
            thetaError -= 2 * math.pi
        elif thetaError < -math.pi:
            thetaError += 2 * math.pi

        return thetaError * RobotConstants.DRIVE_ROTATION_KP

    def periodic(self):
        self.odometry.update(
            self.getActualDirectionOfTravelRad(),
            self.getDistanceTraveledMeters(),
            self.gyro.heading(RotationUnits.REV) * 2 * math.pi
        )

    def applyDesaturated(self, flSpeedRPM, frSpeedRPM, blSpeedRPM, brSpeedRPM):
        fastestSpeedRPM = max(abs(flSpeedRPM), abs(frSpeedRPM), abs(blSpeedRPM), abs(brSpeedRPM))
        if(fastestSpeedRPM > RobotConstants.MOTOR_MAX_SPEED_RPM):
            ratio = RobotConstants.MOTOR_MAX_SPEED_RPM / fastestSpeedRPM

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
        coeffRPM = RobotConstants.SEC_PHI * translationRPM
        xProjectionRPM = coeffRPM * math.sin(directionRad)
        yProjectionRPM = coeffRPM * math.cos(directionRad)

        self.applyDesaturated(
            rotationRPM - (xProjectionRPM - yProjectionRPM),
            rotationRPM - (xProjectionRPM + yProjectionRPM),
            rotationRPM + (xProjectionRPM + yProjectionRPM),
            rotationRPM + (xProjectionRPM - yProjectionRPM)
        )

    def applySpeedsCartesian(self, xSpeedMetersPerSec: float, ySpeedMetersPerSec: float, rotationSpeedRadPerSec: float, fieldOriented = True):
        direction = math.atan2(ySpeedMetersPerSec, xSpeedMetersPerSec)
        magnitude = math.sqrt(xSpeedMetersPerSec ** 2 + ySpeedMetersPerSec ** 2)
        self.applySpeeds(direction, magnitude, rotationSpeedRadPerSec, fieldOriented)

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

        xEffort = xError * RobotConstants.DRIVE_TRANSLATION_KP
        yEffort = yError * RobotConstants.DRIVE_TRANSLATION_KP

        direction = math.atan2(yEffort, xEffort)
        magnitude = math.sqrt(xError**2 + yError**2)

        if abs(magnitude) > RobotConstants.DRIVE_MAX_SPEED_METERS_PER_SEC:
            magnitude = math.copysign(RobotConstants.DRIVE_MAX_SPEED_METERS_PER_SEC, magnitude)

        
        self.applySpeeds(direction, magnitude, self.calcThetaControlRadPerSec(headingRad), True)


    def followLine(self, odomXMetersTarget: float, odomLineYMeters: float, lineSensorArray: LineSensorArray, headingRad = 0.0, findLineIfOff = True):
        driveEffort = (odomXMetersTarget - self.odometry.xMeters) * RobotConstants.DRIVE_TRANSLATION_KP
        if abs(driveEffort) > RobotConstants.DRIVE_FOLLOW_LINE_SPEED_METERS_PER_SEC:
            driveEffort = math.copysign(RobotConstants.DRIVE_FOLLOW_LINE_SPEED_METERS_PER_SEC, driveEffort)
        
        if lineSensorArray.hasLine():
            self.odometry.yMeters = odomLineYMeters

            strafeEffort = lineSensorArray.getError() * RobotConstants.FOLLOW_LINE_KP + lineSensorArray.getRate() * RobotConstants.FOLLOW_LINE_KD

            self.applySpeedsCartesian(driveEffort, strafeEffort, self.calcThetaControlRadPerSec(headingRad), True)
        elif findLineIfOff:
            if lineSensorArray.getLastSensorOnLine() == RIGHT:
                self.applySpeedsCartesian(driveEffort, -RobotConstants.DRIVE_FIND_LINE_SPEED_METERS_PER_SEC, self.calcThetaControlRadPerSec(headingRad), True)
            elif lineSensorArray.getLastSensorOnLine() == LEFT:
                self.applySpeedsCartesian(driveEffort, RobotConstants.DRIVE_FIND_LINE_SPEED_METERS_PER_SEC, self.calcThetaControlRadPerSec(headingRad), True)
            else:
                self.stop()





class Vision:
    vision = Vision(DevicePorts.VISION, RobotConstants.BRIGHTNESS, FRUIT_TYPE.LIME, FRUIT_TYPE.LEMON, FRUIT_TYPE.TANGERINE)
    currentSnapshot = None
    previousSnapshot = None
    currentFruit = FRUIT_TYPE.LEMON

    def periodic(self):
        self.previousSnapshot = self.currentSnapshot
        self.currentSnapshot = self.vision.take_snapshot(self.currentFruit)

    def changeFruit(self, fruit: Signature):
        self.currentFruit = fruit

    def hasFruit(self):
        return self.currentSnapshot

    def getXOffset(self):
        return (160 - self.vision.largest_object().centerX)

    def getYOffset(self):
        return self.vision.largest_object().centerY





class Lift:
    leftLift = Motor(DevicePorts.LEFT_LIFT, False)
    rightLift = Motor(DevicePorts.RIGHT_LIFT, True)

    ultrasonic = Sonar(DevicePorts.ULTRASONIC)

    LOW_DISTANCE = 0.0
    MID_DISTANCE = 0.0
    HIGH_DISTANCE = 0.0

    PINION_CIRCUMFERENCE_IN = 0.5

    STOW_POSITION = 0.025
    LOW_POSITION = 0.2
    MID_POSITION = 3.0
    HIGH_POSITION = 5.5

    leftLift.set_velocity(RobotConstants.LIFT_SPEED, RPM)
    rightLift.set_velocity(RobotConstants.LIFT_SPEED, RPM)

    def periodic(self):
        pass

    def setStowPosition(self):
        self.leftLift.spin_to_position(self.STOW_POSITION, TURNS, False)
        self.rightLift.spin_to_position(self.STOW_POSITION, TURNS, False)

    def setLowPosition(self):
        self.leftLift.spin_to_position(self.LOW_POSITION, TURNS, False)
        self.rightLift.spin_to_position(self.LOW_POSITION, TURNS, False)

    def setMidPosition(self):
        self.leftLift.spin_to_position(self.MID_POSITION, TURNS, False)
        self.rightLift.spin_to_position(self.MID_POSITION, TURNS, False)

    def setHighPosition(self):
        self.leftLift.spin_to_position(self.HIGH_POSITION, TURNS, False)
        self.rightLift.spin_to_position(self.HIGH_POSITION, TURNS, False)

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
        pass

    def periodic(self):
        pass

    def setLockedPosition(self):
        self.leftGate.spin_to_position(self.LOCKED_POSITION, DEGREES)

    def setUnlockedPosition(self):
        self.leftGate.spin_to_position(self.UNLOCKED_POSITION, DEGREES)


drive = Drive(False)
lift = Lift()
gate = Gate()
vision = Vision()

frontLine = LineSensorArray(DevicePorts.FL_LINE, DevicePorts.FR_LINE)
button = Bumper(DevicePorts.BUTTON)


currentState = States.FOLLOW_LINE_ODOMETRY
previousState = States.INIT
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

def FOLLOW_LINE_ODOMETRY(line: TurnType.TurnType, xOdomTarget: float):
    global currentState
    global previousState
    drive.followLine(xOdomTarget, FieldConstants.LEFT_LINE_Y_METERS if line == LEFT else FieldConstants.RIGHT_LINE_Y_METERS, frontLine)
    if (abs(drive.odometry.xMeters - xOdomTarget) < RobotConstants.ODOM_TOLERANCE_METERS):
        drive.stop()
        currentState = States.FACE_FORWARD
        previousState = States.FOLLOW_LINE_ODOMETRY

def FACE_FORWARD():
    global currentState
    global previousState
    target = 0
    error = target - drive.odometry.thetaRad
    effort = error * RobotConstants.DRIVE_ROTATION_KP
    drive.applySpeeds(0, 0, effort, False)
    if (abs(error) < 0.1):
        drive.stop()
        currentState = States.FIND_FRUIT
        previousState = States.FACE_FORWARD

def FIND_FRUIT(fromLine: TurnType.TurnType):
    global currentState
    global previousState

    xError = FieldConstants.FIRST_ROW_X_METERS - drive.odometry.xMeters
    xEffort = xError * RobotConstants.DRIVE_TRANSLATION_KP
    if(abs(xEffort) > 0.1):
        xEffort = math.copysign(0.1, xEffort)
    
    print(xError, xEffort)

    if (vision.hasFruit()):
        Kp = 0.001
        yError = 0 - vision.getXOffset()
        yEffort = yError * Kp
        drive.applySpeedsCartesian(xEffort, yEffort, drive.calcThetaControlRadPerSec(0), True)
        if (abs(yError) < RobotConstants.FRUIT_CENTERING_TOLERANCE_PX):
            drive.stop()
            currentState = States.PICK_FRUIT
            previousState = States.FIND_FRUIT
    else:
        drive.applySpeedsCartesian(xEffort, (0.1 if fromLine == RIGHT else -0.1), drive.calcThetaControlRadPerSec(0), True)
        if (frontLine.hasLine() and previousState != States.FOLLOW_LINE_ODOMETRY):
            drive.stop()
            currentState = States.FACE_FORWARD
            previousState = States.FIND_FRUIT


def PICK_FRUIT():
    global currentState
    global previousState
    lift.setLowPosition()
    if (lift.leftLift.position() >= 0.2):
        desiredX = FieldConstants.FIRST_ROW_X_METERS + RobotConstants.APPROACH_DISTANCE
        desiredY = drive.odometry.yMeters
        desiredTheta = drive.odometry.thetaRad
        if (abs(desiredX - drive.odometry.xMeters) > 0.01):
            drive.driveToPosition(desiredX, desiredY, desiredTheta)
        else:
            drive.stop()
            lift.setStowPosition()
            if (lift.leftLift.position() >= 0.025):
                currentState = States.FIND_FRUIT
                previousState = States.PICK_FRUIT




drive.gyro.calibrate()
while drive.gyro.is_calibrating():
    pass
print("GYRO CALIBRATED")
drive.gyro.set_heading(0.0)



def robotPeriodic():
    timer.event(robotPeriodic, RobotConstants.LOOP_PERIOD_MS)
    drive.periodic()
    lift.periodic()
    gate.periodic()
    vision.periodic()
    frontLine.periodic()
    print(currentState)
    if (currentState == States.FOLLOW_LINE_ODOMETRY):
        FOLLOW_LINE_ODOMETRY(RIGHT, FieldConstants.FIRST_ROW_X_METERS)
    if (currentState == States.FACE_FORWARD):
        FACE_FORWARD()
    if (currentState == States.FIND_FRUIT):
        FIND_FRUIT(RIGHT)
    if (currentState == States.PICK_FRUIT):
        PICK_FRUIT()

    # print("x: " + str(drive.odometry.xMeters), "y: " + str(drive.odometry.yMeters), "theta: " + str(drive.odometry.thetaRad))
    
robotPeriodic()