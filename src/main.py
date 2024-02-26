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
    FOLLOW_LINE_SONAR = 6
    FIND_BASKET = 7
    DUMP_FRUIT = 8
    FIND_LINE = 9

    @staticmethod
    def toString(state: int) -> str:
        if state == States.IDLE:
            return "IDLE"
        elif state == States.INIT:
            return "INIT"
        elif state == States.FOLLOW_LINE_ODOMETRY:
            return "FOLLOW_LINE_ODOOMETRY"
        elif state == States.FACE_DIRECTION:
            return "FACE_DIRECTION"
        elif state == States.FIND_FRUIT:
            return "FIND_FRUIT"
        elif state == States.PICK_FRUIT:
            return "PICK_FRUIT"
        elif state == States.FOLLOW_LINE_SONAR:
            return "FOLLOW_LINE_SONAR"
        elif state == States.FIND_BASKET:
            return "FIND_BASKET"
        elif state == States.FIND_LINE:
            return "FIND_LINE"
        else:
            return ":("
        
    @staticmethod
    def printTransition():
        print(States.toString(previousState), "->", States.toString(currentState))
        

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

    VISION = Ports.PORT1

    FL_LINE = brain.three_wire_port.f
    FR_LINE = brain.three_wire_port.b

    DRIVE_SONAR = brain.three_wire_port.g

    LIFT_SONAR = brain.three_wire_port.c

    BUTTON = brain.three_wire_port.a

class RobotConstants:
        
        LOOP_PERIOD_MSEC = 20
        
        WHEEL_DIAMETER_IN = 4
        WHEEL_CIRCUMFERENCE_IN = WHEEL_DIAMETER_IN * math.pi

        DRIVE_BASE_RADIUS_IN = 7.75
        DRIVE_BASE_CIRCUMFERENCE_IN = DRIVE_BASE_RADIUS_IN * 2 * math.pi

        DRIVE_BASE_WIDTH_METERS = 0.3175

        MOTOR_MAX_SPEED_RPM = 200

        # where phi is the angle between the x/y axis and the wheel vectors, which is always some multiple of pi/4
        SEC_PHI = 2 / math.sqrt(2)

        DRIVE_TRAIN_IDLE_MODE = BRAKE

        DRIVE_TRANSLATION_KP = 2.0

        DRIVE_ROTATION_KP = 7.5

        DRIVE_MAX_SPEED_METERS_PER_SEC = 0.2

        DRIVE_FIND_LINE_SPEED_METERS_PER_SEC = 0.2

        DRIVE_FOLLOW_LINE_SPEED_METERS_PER_SEC = 0.2

        ODOM_TOLERANCE_METERS = 0.01

        ODOM_X_POSITIVE_DRIFT = 194 / 200
        ODOM_X_NEGATIVE_DRIFT = 220 / 200
        ODOM_Y_POSITIVE_DRIFT = 180 / 200
        ODOM_Y_NEGATIVE_DRIFT = 211 / 200

        ODOM_Y_DRIFT_PER_POSITIVE_X_TRANSLATION = -10 / 200
        ODOM_Y_DRIFT_PER_NEGATIVE_X_TRANSLATION = -17 / 200

        LINE_REFLECTIVITY_THRESHOLD = 15

        FOLLOW_LINE_KP = 0.001
        FOLLOW_LINE_KD = 0.00003

        LIFT_SPEED = 150

        VISION_BRIGHTNESS = 41

        FRUIT_CENTERING_TOLERANCE_PX = 10

        FRUIT_APPROACH_DISTANCE_METERS = 0.15

        DUMP_TIME_MSEC = 3000

        MAX_TURN_SPEED_RAD_PER_SEC = 1.5

class FieldConstants:
    LEFT_LINE_Y_METERS = 2.205
    RIGHT_LINE_Y_METERS = 0.28

    FIRST_ROW_X_METERS = 0.65
    SECOND_ROW_X_METERS = 1.4

    DISTANCE_TO_BASKET_WALL_METERS = 0.45

    LIME_BASKET_Y_METERS = 0.71
    LEMON_BASKET_Y_METERS = 1.4
    TANGERINE_BASKET_Y_METERS = 1.81



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
        driftCompPercentage = 1 # abs(math.cos((2 * robotRelativeTranslationRad) % math.pi))

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

        if headingRad < -math.pi:
            headingRad += 2 * math.pi
        elif headingRad > math.pi:
            headingRad -= 2 * math.pi

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

        self.__rate = (self.getError() - self.__prevError) / (RobotConstants.LOOP_PERIOD_MSEC / 1000)
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
        return self.leftSensor.reflectivity() - self.rightSensor.reflectivity()
    
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

    sonar = Sonar(DevicePorts.DRIVE_SONAR)

    odometry = Odometry(0.0, 0.0, 0.0)

    def __init__(self):        
        self.gyroCalibrated = False

    def calibrateGyro(self, heading = 0.0):
        if not self.gyro.is_calibrating():
            self.gyro.calibrate()
        else:
            self.gyroCalibrated = True
        
        self.gyro.set_heading(heading)
        self.odometry.thetaRad = heading

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
    
    def getSonarDistanceMeters(self) -> float:
        return self.sonar.distance(DistanceUnits.CM) / 100

    def calcThetaControlRadPerSec(self, targetHeadingRad: float) -> float:
        thetaError = targetHeadingRad - self.odometry.thetaRad

        if thetaError > math.pi:
            thetaError -= (2 * math.pi)
        elif thetaError < -math.pi:
            thetaError += (2 * math.pi)

        effort = -thetaError - self.odometry.thetaRad

        if abs(effort) > RobotConstants.MAX_TURN_SPEED_RAD_PER_SEC:
            effort = math.copysign(RobotConstants.MAX_TURN_SPEED_RAD_PER_SEC, effort)

        return -thetaError * RobotConstants.DRIVE_ROTATION_KP

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

    def followLineToOdom(self, odomXMetersTarget: float, odomLineYMeters: float, lineSensorArray: LineSensorArray, headingRad = 0.0, findLineIfOff = True):
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

    def wiggle(self):
        if (self.gyro.heading() < 0):
            self.applySpeeds(0, 0, math.pi, True)
        else:
            self.applySpeeds(0, 0, -math.pi, True)

    def followLineToSonar(self, sonarTargetMeters: float, odomLineYMeters: float, lineSensorArray: LineSensorArray, headingRad = 0.0, findLineIfOff = True):
        driveEffort = (sonarTargetMeters - self.getSonarDistanceMeters()) * RobotConstants.DRIVE_TRANSLATION_KP
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
    vision = Vision(DevicePorts.VISION, RobotConstants.VISION_BRIGHTNESS, FRUIT_TYPE.LIME, FRUIT_TYPE.LEMON, FRUIT_TYPE.TANGERINE)
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

    sonar = Sonar(DevicePorts.LIFT_SONAR)

    LOW_DISTANCE = 0.0
    MID_DISTANCE = 0.0
    HIGH_DISTANCE = 0.0

    PINION_CIRCUMFERENCE_IN = 0.5

    STOW_POSITION = 0.025
    LOW_POSITION = 0.2
    MID_POSITION = 3.0
    HIGH_POSITION = 5.5

    target = 0

    leftLift.set_velocity(RobotConstants.LIFT_SPEED, RPM)
    rightLift.set_velocity(RobotConstants.LIFT_SPEED, RPM)

    def periodic(self):
        self.leftLift.spin_to_position(self.target, TURNS, False)
        self.rightLift.spin_to_position(self.target, TURNS, False)

    def setStowPosition(self):
        self.target = self.STOW_POSITION

    def setLowPosition(self):
        self.target = self.LOW_POSITION

    def setMidPosition(self):
        self.target = self.MID_POSITION

    def setHighPosition(self):
        self.target = self.HIGH_POSITION

    def setUltrasonicPosition(self):
        if (abs(self.sonar.distance(INCHES) - self.LOW_POSITION) < 0.01):
            self.setLowPosition()
        if (abs(self.sonar.distance(INCHES) - self.MID_POSITION) < 0.01):
            self.setMidPosition()
        if (abs(self.sonar.distance(INCHES) - self.HIGH_POSITION) < 0.01):
            self.setHighPosition()

    def stop(self):
        self.leftLift.stop()

    def atTarget(self):
        return (abs(self.target - self.leftLift.position(TURNS)) < 0.05)


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
        self.leftGate.spin_to_position(self.LOCKED_POSITION, DEGREES, False)

    def setUnlockedPosition(self):
        self.leftGate.spin_to_position(self.UNLOCKED_POSITION, DEGREES, False)


drive = Drive()
lift = Lift()
gate = Gate()
vision = Vision()

frontLine = LineSensorArray(DevicePorts.FL_LINE, DevicePorts.FR_LINE)
button = Bumper(DevicePorts.BUTTON)


currentState = States.IDLE
previousState = States.IDLE
rowCount = 0

def IDLE():
    drive.stop()
    lift.stop()

def INIT():
    global rowCount
    vision.changeFruit(FRUIT_TYPE.LIME)
    gate.setLockedPosition()
    if not drive.gyroCalibrated:
        drive.calibrateGyro()
    drive.odometry.xMeters = RobotConstants.DRIVE_BASE_WIDTH_METERS / 2
    if drive.gyro.is_calibrating():
        print("GYRO CALIBRATING")
    else:
        print("GYRO CALIBRATED")
    rowCount = 0


def FOLLOW_LINE_ODOMETRY(line: TurnType.TurnType, xOdomTarget: float):
    global currentState, previousState

    drive.followLineToOdom(xOdomTarget, FieldConstants.LEFT_LINE_Y_METERS if line == LEFT else FieldConstants.RIGHT_LINE_Y_METERS, frontLine)
    if (abs(drive.odometry.xMeters - xOdomTarget) < RobotConstants.ODOM_TOLERANCE_METERS and line == RIGHT):
        drive.stop()
        currentState = States.FACE_DIRECTION
        previousState = States.FOLLOW_LINE_ODOMETRY
    if (abs(drive.odometry.xMeters - xOdomTarget) < RobotConstants.ODOM_TOLERANCE_METERS and line == LEFT):
        drive.stop()
        currentState = States.FACE_DIRECTION
        previousState = States.FOLLOW_LINE_ODOMETRY
        States.printTransition()

def FACE_DIRECTION(direction: DirectionType.DirectionType):
    global currentState, previousState

    if direction == FORWARD:
        target = 0
    else:
        target = math.pi
    
    error = target - drive.odometry.thetaRad
    drive.applySpeeds(0, 0, drive.calcThetaControlRadPerSec(target), False)
    if (abs(error) < 0.1):
        drive.stop()
        if previousState == States.FOLLOW_LINE_ODOMETRY:
            currentState = States.FIND_FRUIT
            previousState = States.FACE_DIRECTION
        elif previousState == States.FIND_FRUIT:
            currentState = States.FOLLOW_LINE_SONAR
            previousState = States.FACE_DIRECTION
        States.printTransition()

def FIND_FRUIT(fromLine: TurnType.TurnType):
    global currentState, previousState, rowCount

    if rowCount == 0:
        xError = FieldConstants.FIRST_ROW_X_METERS - drive.odometry.xMeters
    if rowCount == 1:
        xError = FieldConstants.SECOND_ROW_X_METERS - drive.odometry.xMeters
    xEffort = xError * RobotConstants.DRIVE_TRANSLATION_KP
    if(abs(xEffort) > 0.1):
        xEffort = math.copysign(0.1, xEffort)
    
    if (vision.hasFruit() and previousState):
        Kp = 0.001
        yError = -vision.getXOffset()
        yEffort = yError * Kp if rowCount % 2 == 0 else yError * -Kp
        drive.applySpeedsCartesian(xEffort, yEffort, drive.calcThetaControlRadPerSec(0 if rowCount % 2 == 0 else math.pi), True)
        if (abs(yError) < RobotConstants.FRUIT_CENTERING_TOLERANCE_PX):
            drive.stop()
            currentState = States.PICK_FRUIT
            previousState = States.FIND_FRUIT
            States.printTransition()
            lift.setMidPosition()
    else:
        drive.applySpeedsCartesian(xEffort, (0.1 if fromLine == RIGHT else -0.1), drive.calcThetaControlRadPerSec(0 if rowCount % 2 == 0 else math.pi), True)
        if (frontLine.hasLine() and previousState != States.FACE_DIRECTION):
            drive.stop()
            currentState = States.FACE_DIRECTION
            previousState = States.FIND_FRUIT
            States.printTransition()


def PICK_FRUIT():
    global currentState, previousState, rowCount

    desiredY = drive.odometry.yMeters
    desiredTheta = 0 if rowCount % 2 == 0 else math.pi
    if lift.atTarget() and lift.target == lift.MID_POSITION:
        if rowCount == 0:
            desiredX = FieldConstants.FIRST_ROW_X_METERS + RobotConstants.FRUIT_APPROACH_DISTANCE_METERS
        elif rowCount == 1:
            desiredX = FieldConstants.SECOND_ROW_X_METERS - RobotConstants.FRUIT_APPROACH_DISTANCE_METERS
        if (abs(desiredX - drive.odometry.xMeters) > 0.01):
            drive.driveToPosition(desiredX, desiredY, desiredTheta)
        else:
            drive.stop()
            lift.setStowPosition()
    elif lift.atTarget() and lift.target == lift.STOW_POSITION:
        if rowCount == 0:
            desiredX = FieldConstants.FIRST_ROW_X_METERS
        elif rowCount == 1:
            desiredX = FieldConstants.SECOND_ROW_X_METERS
        if (abs(desiredX - drive.odometry.xMeters) > 0.01):
            drive.driveToPosition(desiredX, desiredY, desiredTheta)
        else:
            drive.stop()
            currentState = States.FIND_FRUIT
            previousState = States.PICK_FRUIT
            States.printTransition()
    else:
        drive.stop()

def FOLLOW_LINE_SONAR(line: TurnType.TurnType, sonarTargetMeters: float):
    global currentState, previousState
    drive.followLineToSonar(sonarTargetMeters, FieldConstants.LEFT_LINE_Y_METERS if line == LEFT else FieldConstants.RIGHT_LINE_Y_METERS, frontLine)
    if (abs(drive.getSonarDistanceMeters() - sonarTargetMeters) < RobotConstants.ODOM_TOLERANCE_METERS):
        drive.stop()
        drive.odometry.xMeters = drive.getSonarDistanceMeters() + (RobotConstants.DRIVE_BASE_WIDTH_METERS / 2)
        currentState = States.FIND_BASKET
        previousState = States.FOLLOW_LINE_SONAR
        States.printTransition()

def FIND_BASKET(fruit: Signature):
    global currentState, previousState

    yTargetMeters = 0
    if fruit == FRUIT_TYPE.LIME:
        yTargetMeters = FieldConstants.LIME_BASKET_Y_METERS
    elif fruit == FRUIT_TYPE.LEMON:
        yTargetMeters = FieldConstants.LEMON_BASKET_Y_METERS
    elif fruit == FRUIT_TYPE.TANGERINE:
        yTargetMeters = FieldConstants.TANGERINE_BASKET_Y_METERS
    
    drive.driveToPosition(FieldConstants.DISTANCE_TO_BASKET_WALL_METERS + (RobotConstants.DRIVE_BASE_WIDTH_METERS / 2), yTargetMeters, 0)

    if(abs(drive.odometry.yMeters - yTargetMeters) < 0.05):
        drive.stop()
        currentState = States.DUMP_FRUIT
        previousState = States.FIND_BASKET
        States.printTransition()
        dumpTimer.reset()

dumpTimer = Timer()
def DUMP_FRUIT():
    global currentState, previousState, rowCount

    if dumpTimer.time() < RobotConstants.DUMP_TIME_MSEC:
        drive.applySpeeds(math.pi, 0.2, 0, True)
        gate.setUnlockedPosition()
    else:
        drive.stop()
        gate.setLockedPosition()
        currentState = States.FIND_LINE
        previousState = States.DUMP_FRUIT
        rowCount += 1
        States.printTransition()

def FIND_LINE():
    global currentState, previousState, rowCount
    drive.applySpeeds(3 * math.pi / 2, 0.1, 0, True)
    if (frontLine.hasLine()):
        drive.odometry.xMeters = drive.getSonarDistanceMeters()
        currentState = States.FOLLOW_LINE_ODOMETRY
        previousState = States.FIND_LINE


def robotPeriodic():
    timer.event(robotPeriodic, RobotConstants.LOOP_PERIOD_MSEC)

    drive.periodic()
    lift.periodic()
    gate.periodic()
    vision.periodic()
    frontLine.periodic()

    print(States.toString(currentState))

    if currentState == States.IDLE:
        IDLE()
    elif currentState == States.INIT:
        INIT()
    elif currentState == States.FOLLOW_LINE_ODOMETRY:
        if rowCount == 0:
            FOLLOW_LINE_ODOMETRY(RIGHT, FieldConstants.FIRST_ROW_X_METERS)
        elif rowCount == 1:
            FOLLOW_LINE_ODOMETRY(RIGHT, FieldConstants.SECOND_ROW_X_METERS)
    elif currentState == States.FACE_DIRECTION:
        if rowCount % 2 == 0 or previousState == States.FIND_FRUIT:
            FACE_DIRECTION(FORWARD)
        else:
            FACE_DIRECTION(REVERSE)
    elif currentState == States.FIND_FRUIT:
        FIND_FRUIT(RIGHT)
    elif currentState == States.PICK_FRUIT:
        PICK_FRUIT()
    elif currentState == States.FOLLOW_LINE_SONAR:
        FOLLOW_LINE_SONAR(LEFT, FieldConstants.DISTANCE_TO_BASKET_WALL_METERS)
    elif currentState == States.FIND_BASKET:
        FIND_BASKET(FRUIT_TYPE.LIME)
    elif currentState == States.DUMP_FRUIT:
        DUMP_FRUIT()
    elif currentState == States.FIND_LINE:
        FIND_LINE()
        
    # print("x: " + str(drive.odometry.xMeters), "y: " + str(drive.odometry.yMeters), "theta: " + str(drive.odometry.thetaRad))
    
def handleButtonPress():
    global currentState
    global previousState

    if currentState == States.IDLE:
        drive.gyroCalibrated = False
        currentState = States.INIT
        previousState = States.IDLE
    elif currentState == States.INIT:
        if not drive.gyro.is_calibrating():
            currentState = States.FOLLOW_LINE_ODOMETRY
            previousState = States.INIT
    else:
        previousState = currentState
        currentState = States.IDLE

button.pressed(handleButtonPress)
robotPeriodic()