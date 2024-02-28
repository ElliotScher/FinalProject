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
    APPROACH_FRUIT = 5
    PICK_FRUIT = 6
    FOLLOW_LINE_SONAR = 7
    FIND_BASKET = 8
    DUMP_FRUIT = 9
    FIND_LINE = 10
    YIPEE = 11

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
        elif state == States.APPROACH_FRUIT:
            return "APPROACH_FRUIT"
        elif state == States.PICK_FRUIT:
            return "PICK_FRUIT"
        elif state == States.FOLLOW_LINE_SONAR:
            return "FOLLOW_LINE_SONAR"
        elif state == States.FIND_BASKET:
            return "FIND_BASKET"
        elif state == States.DUMP_FRUIT:
            return "DUMP_FRUIT"
        elif state == States.FIND_LINE:
            return "FIND_LINE"
        elif state == States.YIPEE:
            return "YIPEEEE!!!"
        else:
            return ":("
        
    @staticmethod
    def printTransition():
        print(States.toString(previousState), "->", States.toString(currentState))
        

class FruitTypes:
    LIME = Signature(1, -6103, -4963, -5532, -3505, -2365, -2936, 2.5, 0)
    LEMON =  Signature(2, 3593, 4389, 3991, -3215, -2695, -2955, 2.5, 0)
    TANGERINE = Signature(3, 3759, 4791, 4275, -831, -311, -571, 6.2, 0)

    @staticmethod
    def toString(fruitType: Signature) -> str:
        if fruitType == FruitTypes.LIME:
            return "LIME"
        elif fruitType == FruitTypes.LEMON:
            return "LEMON"
        elif fruitType == FruitTypes.TANGERINE:
            return "TANGERINE"
        else:
            return ":("

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

        DRIVE_ROTATION_KP = 5.0

        DRIVE_MAX_SPEED_METERS_PER_SEC = 0.2

        DRIVE_FIND_LINE_SPEED_METERS_PER_SEC = 0.2

        DRIVE_FOLLOW_LINE_SPEED_METERS_PER_SEC = 0.2

        DRIVE_APPROACH_FRUIT_SPEED_METERS_PER_SEC = 0.1

        ODOM_TOLERANCE_METERS = 0.02
        HEADING_TOLERANCE_RAD = 0.3

        ODOM_X_POSITIVE_DRIFT = 194 / 200
        ODOM_X_NEGATIVE_DRIFT = 230 / 200
        ODOM_Y_POSITIVE_DRIFT = 180 / 200
        ODOM_Y_NEGATIVE_DRIFT = 211 / 200

        ODOM_Y_DRIFT_PER_POSITIVE_X_TRANSLATION = -10 / 200
        ODOM_Y_DRIFT_PER_NEGATIVE_X_TRANSLATION = -17 / 200

        ODOM_X_DRIFT_PER_POSITIVE_Y_TRANSLATION = 2 / 200
        ODOM_X_DRIFT_PER_NEGATIVE_Y_TRANSLATION = 10 / 200

        LINE_REFLECTIVITY_THRESHOLD = 17

        FOLLOW_LINE_KP = 0.0002
        FOLLOW_LINE_KD = 0.000006

        LIFT_SPEED = 150

        VISION_BRIGHTNESS = 85

        FRUIT_CENTERING_KP = 0.0024

        FRUIT_CENTERING_TOLERANCE_PX = 10

        FRUIT_APPROACH_DISTANCE_METERS = 0.15

        DUMP_DRIVE_TIME_MSEC = 500
        DUMP_TIME_MSEC = 4000
        
        DUMP_TURN_SPEED_RAD_PER_SEC = math.pi / 2

        MAX_TURN_SPEED_RAD_PER_SEC = math.pi

class FieldConstants:
    LEFT_LINE_Y_METERS = 2.205
    RIGHT_LINE_Y_METERS = 0.28

    FIRST_ROW_X_METERS = 0.65
    SECOND_ROW_X_METERS = 1.25
    THIRD_ROW_X_METERS = 1.25
    FOURTH_ROW_X_METERS = 2.03
    FIFTH_ROW_X_METERS = 2.03
    SIXTH_ROW_X_METERS = 2.96

    DISTANCE_TO_BASKET_WALL_METERS = 0.45

    LIME_BASKET_Y_METERS = 0.71
    TANGERINE_BASKET_Y_METERS = 1.4
    LEMON_BASKET_Y_METERS = 1.81

    BASKET_DEPTH_METERS = 0.23

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
            xDelta *= (RobotConstants.ODOM_X_POSITIVE_DRIFT ** math.cos(headingRad))
            yDelta += ((xDelta * RobotConstants.ODOM_Y_DRIFT_PER_POSITIVE_X_TRANSLATION) * driftCompPercentage)
        else:
            xDelta *= (RobotConstants.ODOM_X_NEGATIVE_DRIFT ** math.cos(headingRad))
            yDelta += ((xDelta * RobotConstants.ODOM_Y_DRIFT_PER_POSITIVE_X_TRANSLATION) * driftCompPercentage)

        if yDelta > 0.0:
            xDelta += ((yDelta * RobotConstants.ODOM_X_DRIFT_PER_POSITIVE_Y_TRANSLATION * math.cos(headingRad)))
        else:
            xDelta += ((yDelta * RobotConstants.ODOM_X_DRIFT_PER_NEGATIVE_Y_TRANSLATION * math.cos(headingRad)))


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

    gyro = Inertial(DevicePorts.GYRO)

    sonar = Sonar(DevicePorts.DRIVE_SONAR)

    odometry = Odometry(0.0, 0.0, 0.0)

    def __init__(self):        
        self.gyroStartedCalibrating = False
        self.gyroHasCalibrated = False

    def calibrateGyro(self, heading = 0.0):
        if not self.gyro.is_calibrating() and not self.gyroStartedCalibrating:
            self.gyro.calibrate()
            print("GYRO CALIBRATING")
            self.gyroStartedCalibrating = True
        elif self.gyroStartedCalibrating and not self.gyro.is_calibrating() and not self.gyroHasCalibrated:
            print("GYRO CALIBRATED")
            self.gyroHasCalibrated = True
        
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
        return (speedRadPerSec * RobotConstants.DRIVE_BASE_CIRCUMFERENCE_IN * 60) / (2 * math.pi * RobotConstants.WHEEL_CIRCUMFERENCE_IN)
    
    def getSonarDistanceMeters(self) -> float:
        return self.sonar.distance(DistanceUnits.CM) / 100

    def calcThetaControlRadPerSec(self, targetHeadingRad: float) -> float:
        thetaError = self.odometry.thetaRad - targetHeadingRad

        if thetaError > math.pi:
            thetaError -= (2 * math.pi)
        elif thetaError < -math.pi:
            thetaError += (2 * math.pi)

        thetaEffort = thetaError * RobotConstants.DRIVE_ROTATION_KP

        if abs(thetaEffort) > RobotConstants.MAX_TURN_SPEED_RAD_PER_SEC:
            thetaEffort = math.copysign(RobotConstants.MAX_TURN_SPEED_RAD_PER_SEC, thetaEffort)

        return thetaEffort

    def periodic(self):
        self.odometry.update(
            self.getActualDirectionOfTravelRad(),
            self.getDistanceTraveledMeters(),
            self.gyro.heading(RotationUnits.REV) * 2 * math.pi
        )

        if currentState == States.IDLE:
            self.flDrive.set_stopping(BrakeType.COAST)
            self.frDrive.set_stopping(BrakeType.COAST)
            self.blDrive.set_stopping(BrakeType.COAST)
            self.brDrive.set_stopping(BrakeType.COAST)
        else:
            self.flDrive.set_stopping(RobotConstants.DRIVE_TRAIN_IDLE_MODE)
            self.frDrive.set_stopping(RobotConstants.DRIVE_TRAIN_IDLE_MODE)
            self.blDrive.set_stopping(RobotConstants.DRIVE_TRAIN_IDLE_MODE)
            self.brDrive.set_stopping(RobotConstants.DRIVE_TRAIN_IDLE_MODE)

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
    
    def driveToPosition(self, xMeters: float, yMeters: float, headingRad: float, maxSpeedMetersPerSec = RobotConstants.DRIVE_MAX_SPEED_METERS_PER_SEC):
        xError = xMeters - self.odometry.xMeters
        yError = yMeters - self.odometry.yMeters

        xEffort = xError * RobotConstants.DRIVE_TRANSLATION_KP
        yEffort = yError * RobotConstants.DRIVE_TRANSLATION_KP

        direction = math.atan2(yEffort, xEffort)
        magnitude = math.sqrt(xError**2 + yError**2)

        if abs(magnitude) > maxSpeedMetersPerSec:
            magnitude = math.copysign(maxSpeedMetersPerSec, magnitude)

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

    def wiggle(self):
        if self.odometry.thetaRad > 0 and self.odometry.thetaRad < math.pi:
            self.applySpeeds(0, 0, RobotConstants.DUMP_TURN_SPEED_RAD_PER_SEC, True)
        else:
            self.applySpeeds(0, 0, -RobotConstants.DUMP_TURN_SPEED_RAD_PER_SEC, True)



class Vision:
    vision = Vision(DevicePorts.VISION, RobotConstants.VISION_BRIGHTNESS, FruitTypes.LIME, FruitTypes.LEMON, FruitTypes.TANGERINE)
    currentSnapshot = None
    previousSnapshot = None
    currentFruit = FruitTypes.LEMON

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
        return (106 - self.vision.largest_object().centerY) + 30




class Lift:
    leftLift = Motor(DevicePorts.LEFT_LIFT, False)
    rightLift = Motor(DevicePorts.RIGHT_LIFT, True)

    sonar = Sonar(DevicePorts.LIFT_SONAR)

    LOW_DISTANCE_METERS = 0.0
    MID_DISTANCE_METERS = 0.126
    HIGH_DISTANCE_METERS = 0.234

    SONAR_TOLERANCE_METERS = 0.05

    PINION_CIRCUMFERENCE_IN = 0.5

    STOW_POSITION = 0.025
    DEPLOY_FLAP_POSITION = 0.5
    LOW_POSITION = 0.2
    MID_POSITION = 3.0
    HIGH_POSITION = 5.2

    targetPosition = 0

    leftLift.set_velocity(RobotConstants.LIFT_SPEED, RPM)
    rightLift.set_velocity(RobotConstants.LIFT_SPEED, RPM)

    def __init__(self):
        self.hasStartedZeroing = False
        self.hasZeroed = False
        self.leftLift.reset_position()
        self.rightLift.reset_position()

    def periodic(self):
        if currentState == States.IDLE:
            self.leftLift.stop()
            self.rightLift.stop()
        elif (self.hasStartedZeroing and self.hasZeroed) or (not self.hasStartedZeroing and not self.hasZeroed):
            self.leftLift.spin_to_position(self.targetPosition, TURNS, False)
            self.rightLift.spin_to_position(self.targetPosition, TURNS, False)

    def zero(self):
        if not self.hasStartedZeroing and not self.hasZeroed:
            self.leftLift.spin(REVERSE, 50)
            self.rightLift.spin(REVERSE, 50)
            self.hasStartedZeroing = True
            self.targetPosition = Lift.STOW_POSITION
            print("LIFT ZEROING")
        elif self.hasStartedZeroing and not self.hasZeroed:
            if self.leftLift.torque() > 0.3:
                self.leftLift.stop()
                self.leftLift.reset_position()
            
            if self.rightLift.torque() > 0.5:
                self.rightLift.stop()
                self.rightLift.reset_position()
            
            if self.leftLift.command() == 0 and self.rightLift.command() == 0:
                self.hasZeroed = True
                print("LIFT ZEROED")

    def setStowPosition(self):
        self.targetPosition = self.STOW_POSITION

    def setFlapDeployPosition(self):
        self.targetPosition = self.DEPLOY_FLAP_POSITION

    def setLowPosition(self):
        self.targetPosition = self.LOW_POSITION

    def setMidPosition(self):
        self.targetPosition = self.MID_POSITION

    def setHighPosition(self):
        self.targetPosition = self.HIGH_POSITION

    def setSonarPosition(self):
        if abs(self.getSonarDistanceMeters() - self.MID_DISTANCE_METERS) < self.SONAR_TOLERANCE_METERS:
            self.setMidPosition()
            print("height: MID", self.getSonarDistanceMeters())
        elif abs(self.getSonarDistanceMeters() - self.HIGH_DISTANCE_METERS) < self.SONAR_TOLERANCE_METERS:
            self.setHighPosition()
            print("height: HIGH", self.getSonarDistanceMeters())
        else:
            self.setLowPosition()
            print("height: LOW", self.getSonarDistanceMeters())
        

    def stop(self):
        self.leftLift.stop()

    def atTarget(self):
        return (abs(self.targetPosition - self.leftLift.position(TURNS)) < 0.05)
    
    def getSonarDistanceMeters(self) -> float:
        sonarMeters = self.sonar.distance(DistanceUnits.CM) / 100
        if sonarMeters >= 999:
            sonarMeters = 0
        return sonarMeters

class Gate:
    gate = Motor(DevicePorts.GATE, False)

    targetRad = 0

    LOCKED_RAD = 0.0
    UNLOCKED_RAD = math.pi / 2

    def __init__(self):
        pass

    def periodic(self):
        global currentState
        if currentState == States.IDLE:
            self.gate.stop()
        else:
            self.gate.spin_to_position(self.targetRad / (2 * math.pi), TURNS, False)

    def setLockedPosition(self):
        self.targetRad = self.LOCKED_RAD

    def setUnlockedPosition(self):
        self.targetRad = self.UNLOCKED_RAD

    def atTarget(self) -> bool:
        return abs((self.gate.position(TURNS) * 2 * math.pi) - self.targetRad) < 0.04


drive = Drive()
lift = Lift()
gate = Gate()
vision = Vision()

frontLine = LineSensorArray(DevicePorts.FL_LINE, DevicePorts.FR_LINE)
button = Bumper(DevicePorts.BUTTON)


currentState = States.IDLE
previousState = States.IDLE
targetRow = 0
pickFruitXTarget = 0

def IDLE():
    drive.stop()
    lift.stop()

def INIT():
    global targetRow
    gate.setLockedPosition()
    drive.calibrateGyro()
    drive.odometry.xMeters = RobotConstants.DRIVE_BASE_WIDTH_METERS / 2
    targetRow = 4
    vision.changeFruit(FruitTypes.LEMON)

    if not lift.hasStartedZeroing and lift.targetPosition != Lift.DEPLOY_FLAP_POSITION:
        lift.setFlapDeployPosition()
        print("DEPLOYING FLAP")
    elif lift.atTarget() and not lift.hasStartedZeroing:
        print("FLAP DEPLOYED")
        lift.zero()
    elif lift.atTarget() or lift.hasStartedZeroing:
        lift.zero()


def FOLLOW_LINE_ODOMETRY(line: TurnType.TurnType, xOdomTarget: float):
    global currentState, previousState
    
    drive.followLineToOdom(xOdomTarget, FieldConstants.LEFT_LINE_Y_METERS if line == LEFT else FieldConstants.RIGHT_LINE_Y_METERS, frontLine)
    if (abs(drive.odometry.xMeters - xOdomTarget) < RobotConstants.ODOM_TOLERANCE_METERS and line == RIGHT):
        drive.stop()
        currentState = States.FACE_DIRECTION
        previousState = States.FOLLOW_LINE_ODOMETRY
        States.printTransition()
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
    if (abs(error) < RobotConstants.HEADING_TOLERANCE_RAD):
        drive.stop()
        if previousState == States.FOLLOW_LINE_ODOMETRY:
            currentState = States.FIND_FRUIT
            previousState = States.FACE_DIRECTION
        elif previousState == States.FIND_FRUIT:
            currentState = States.FOLLOW_LINE_SONAR
            previousState = States.FACE_DIRECTION
        States.printTransition()

def FIND_FRUIT(fromLine: TurnType.TurnType):
    global currentState, previousState, targetRow, currentFruitYOffset

    xError = 0
    if targetRow == 0:
        xError = FieldConstants.FIRST_ROW_X_METERS - drive.odometry.xMeters
    elif targetRow == 1:
        xError = FieldConstants.SECOND_ROW_X_METERS - drive.odometry.xMeters
    elif targetRow == 2:
        xError = FieldConstants.THIRD_ROW_X_METERS - drive.odometry.xMeters
    elif targetRow == 3:
        xError = FieldConstants.FOURTH_ROW_X_METERS - drive.odometry.xMeters
    elif targetRow == 4:
        xError = FieldConstants.FIFTH_ROW_X_METERS - drive.odometry.xMeters
    elif targetRow == 5:
        xError = FieldConstants.SIXTH_ROW_X_METERS - drive.odometry.xMeters

    xEffort = xError * RobotConstants.DRIVE_TRANSLATION_KP
    if(abs(xEffort) > RobotConstants.DRIVE_APPROACH_FRUIT_SPEED_METERS_PER_SEC):
        xEffort = math.copysign(RobotConstants.DRIVE_APPROACH_FRUIT_SPEED_METERS_PER_SEC, xEffort)
    
    if vision.hasFruit():
        yError = vision.getXOffset() * (-1 if targetRow % 2 == 0 else 1)
        yEffort = yError * RobotConstants.FRUIT_CENTERING_KP
        if abs(yEffort) > RobotConstants.DRIVE_APPROACH_FRUIT_SPEED_METERS_PER_SEC:
            yEffort = math.copysign(RobotConstants.DRIVE_APPROACH_FRUIT_SPEED_METERS_PER_SEC, yEffort)

        drive.applySpeedsCartesian(xEffort, yEffort, drive.calcThetaControlRadPerSec(0 if targetRow % 2 == 0 else math.pi), True)
        if (abs(yError) < RobotConstants.FRUIT_CENTERING_TOLERANCE_PX):
            drive.stop()
            currentState = States.APPROACH_FRUIT
            previousState = States.FIND_FRUIT
            States.printTransition()
    else:    
        drive.applySpeedsCartesian(xEffort, (RobotConstants.DRIVE_MAX_SPEED_METERS_PER_SEC if fromLine == RIGHT else -RobotConstants.DRIVE_MAX_SPEED_METERS_PER_SEC), drive.calcThetaControlRadPerSec(0 if targetRow % 2 == 0 else math.pi), True)
        yDistanceTraveledMeters = abs((FieldConstants.RIGHT_LINE_Y_METERS if fromLine == RIGHT else FieldConstants.LEFT_LINE_Y_METERS) - drive.odometry.yMeters)
        if (frontLine.hasLine() and yDistanceTraveledMeters > 0.05):
            drive.stop()
            currentState = States.FACE_DIRECTION
            previousState = States.FIND_FRUIT
            States.printTransition()

def APPROACH_FRUIT():
    global currentState, previousState, targetRow, pickFruitXTarget

    if vision.hasFruit():
        xError = vision.getYOffset() * (1 if targetRow % 2 == 0 else -1)
        xEffort = xError * RobotConstants.FRUIT_CENTERING_KP
        if abs(xEffort) > RobotConstants.DRIVE_APPROACH_FRUIT_SPEED_METERS_PER_SEC:
            xEffort = math.copysign(RobotConstants.DRIVE_APPROACH_FRUIT_SPEED_METERS_PER_SEC, xEffort)
        
        yError = vision.getXOffset() * (-1 if targetRow % 2 == 0 else 1)
        yEffort = yError * RobotConstants.FRUIT_CENTERING_KP
        if abs(yEffort) > RobotConstants.DRIVE_APPROACH_FRUIT_SPEED_METERS_PER_SEC:
            yEffort = math.copysign(RobotConstants.DRIVE_APPROACH_FRUIT_SPEED_METERS_PER_SEC, yEffort)

        drive.applySpeedsCartesian(xEffort, yEffort, drive.calcThetaControlRadPerSec(0 if targetRow % 2 == 0 else math.pi), True)

        if abs(vision.getYOffset()) < RobotConstants.FRUIT_CENTERING_TOLERANCE_PX:
            drive.stop()
            pickFruitXTarget = drive.odometry.xMeters + (RobotConstants.FRUIT_APPROACH_DISTANCE_METERS if targetRow % 2 == 0 else -RobotConstants.FRUIT_APPROACH_DISTANCE_METERS)
            lift.setSonarPosition()
            currentState = States.PICK_FRUIT
            previousState = States.APPROACH_FRUIT
            States.printTransition()
    else:
        desiredX = drive.odometry.xMeters
        desiredY = drive.odometry.yMeters
        desiredTheta = 0 if targetRow % 2 == 0 else math.pi
        if targetRow == 0:
            desiredX = FieldConstants.FIRST_ROW_X_METERS
        elif targetRow == 1:
            desiredX = FieldConstants.SECOND_ROW_X_METERS
        elif targetRow == 2:
            desiredX = FieldConstants.THIRD_ROW_X_METERS
        elif targetRow == 3:
            desiredX = FieldConstants.FOURTH_ROW_X_METERS
        elif targetRow == 4:
            xError = FieldConstants.FIFTH_ROW_X_METERS - drive.odometry.xMeters
        elif targetRow == 5:
            xError = FieldConstants.FIFTH_ROW_X_METERS - drive.odometry.xMeters
        
        if (abs(desiredX - drive.odometry.xMeters) > RobotConstants.ODOM_TOLERANCE_METERS):
            drive.driveToPosition(desiredX, desiredY, desiredTheta, RobotConstants.DRIVE_APPROACH_FRUIT_SPEED_METERS_PER_SEC)
        else:
            drive.stop()
            currentState = States.FIND_FRUIT
            previousState = States.APPROACH_FRUIT
            States.printTransition()

def PICK_FRUIT():
    global currentState, previousState, targetRow, pickFruitXTarget

    desiredY = drive.odometry.yMeters
    desiredTheta = 0 if targetRow % 2 == 0 else math.pi
    desiredX = pickFruitXTarget
    if lift.atTarget() and lift.targetPosition != lift.STOW_POSITION:
        if (abs(desiredX - drive.odometry.xMeters) > RobotConstants.ODOM_TOLERANCE_METERS):
            drive.driveToPosition(desiredX, desiredY, desiredTheta, RobotConstants.DRIVE_APPROACH_FRUIT_SPEED_METERS_PER_SEC)
        else:
            lift.setStowPosition()
    elif lift.atTarget() and lift.targetPosition == lift.STOW_POSITION:
        desiredX = drive.odometry.xMeters
        if targetRow == 0:
            desiredX = FieldConstants.FIRST_ROW_X_METERS
        elif targetRow == 1:
            desiredX = FieldConstants.SECOND_ROW_X_METERS
        elif targetRow == 2:
            desiredX = FieldConstants.THIRD_ROW_X_METERS
        elif targetRow == 3:
            desiredX = FieldConstants.FOURTH_ROW_X_METERS
        elif targetRow == 4:
            desiredX = FieldConstants.FIFTH_ROW_X_METERS
        elif targetRow == 5:
            desiredX = FieldConstants.SIXTH_ROW_X_METERS
        if (abs(desiredX - drive.odometry.xMeters) > RobotConstants.ODOM_TOLERANCE_METERS):
            drive.driveToPosition(desiredX, desiredY, desiredTheta, RobotConstants.DRIVE_APPROACH_FRUIT_SPEED_METERS_PER_SEC)
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
        print("x odometry updated:", drive.odometry.xMeters)
        print("theta odometry updated", drive.odometry.thetaRad)
        currentState = States.FIND_BASKET
        previousState = States.FOLLOW_LINE_SONAR
        States.printTransition()

def FIND_BASKET(fruit: Signature):
    global currentState, previousState

    yTargetMeters = 0
    if fruit == FruitTypes.LIME:
        yTargetMeters = FieldConstants.LIME_BASKET_Y_METERS
    elif fruit == FruitTypes.LEMON:
        yTargetMeters = FieldConstants.LEMON_BASKET_Y_METERS
    elif fruit == FruitTypes.TANGERINE:
        yTargetMeters = FieldConstants.TANGERINE_BASKET_Y_METERS
    
    drive.driveToPosition(FieldConstants.DISTANCE_TO_BASKET_WALL_METERS + (RobotConstants.DRIVE_BASE_WIDTH_METERS / 2), yTargetMeters, 0)

    if(abs(drive.odometry.yMeters - yTargetMeters) < 0.05):
        drive.stop()
        currentState = States.DUMP_FRUIT
        previousState = States.FIND_BASKET
        States.printTransition()
        dumpTimer.reset()
        timer.event(setDumpFruitXOdom, int((3 * RobotConstants.DUMP_DRIVE_TIME_MSEC) + RobotConstants.DUMP_TIME_MSEC))

def setDumpFruitXOdom():
    drive.odometry.xMeters = FieldConstants.BASKET_DEPTH_METERS + (RobotConstants.DRIVE_BASE_WIDTH_METERS / 2)
    # drive.gyro.set_heading(0)
    # drive.odometry.thetaRad = 0
    print("x odometry updated:", drive.odometry.xMeters)
    # print("theta odometry updated", drive.odometry.thetaRad)

dumpTimer = Timer()
def DUMP_FRUIT():
    global currentState, previousState, targetRow

    if dumpTimer.time() < RobotConstants.DUMP_DRIVE_TIME_MSEC:
        drive.applySpeeds(math.pi, 0.2, drive.calcThetaControlRadPerSec(0), True)
    elif dumpTimer.time() < 3 * RobotConstants.DUMP_DRIVE_TIME_MSEC / 2:
        gate.setUnlockedPosition()
    elif dumpTimer.time() < RobotConstants.DUMP_DRIVE_TIME_MSEC + RobotConstants.DUMP_TIME_MSEC:
        drive.wiggle()
    elif dumpTimer.time() < (2 * RobotConstants.DUMP_DRIVE_TIME_MSEC) + RobotConstants.DUMP_TIME_MSEC:
        drive.applySpeeds(math.pi, 0.15, 0, True)
    else:
        targetXMeters = FieldConstants.DISTANCE_TO_BASKET_WALL_METERS + (RobotConstants.DRIVE_BASE_WIDTH_METERS / 2)
        drive.driveToPosition(targetXMeters, drive.odometry.yMeters, 0)
        if abs(drive.odometry.xMeters - targetXMeters) < RobotConstants.ODOM_TOLERANCE_METERS:
            drive.stop()
            gate.setLockedPosition()
            # drive.odometry.xMeters = FieldConstants.DISTANCE_TO_BASKET_WALL_METERS + (RobotConstants.DRIVE_BASE_WIDTH_METERS / 2)
            currentState = States.FIND_LINE
            previousState = States.DUMP_FRUIT
            States.printTransition()

def FIND_LINE():
    global currentState, previousState, targetRow
    # xError = (FieldConstants.DISTANCE_TO_BASKET_WALL_METERS + (RobotConstants.DRIVE_BASE_WIDTH_METERS / 2)) - drive.odometry.xMeters
    # xEffort = xError * RobotConstants.DRIVE_TRANSLATION_KP
    drive.applySpeedsCartesian(0, -RobotConstants.DRIVE_FIND_LINE_SPEED_METERS_PER_SEC, drive.calcThetaControlRadPerSec(0), True)
    if (frontLine.hasLine()):
        drive.stop()
        targetRow += 1
        targetFruit = FruitTypes.LIME
        if targetRow <= 1:
            targetFruit = FruitTypes.LIME
        elif targetRow <= 3:
            targetFruit = FruitTypes.TANGERINE
        elif targetRow <= 5:
            targetFruit = FruitTypes.LEMON

        if targetRow <= 5:
            vision.changeFruit(targetFruit)
            currentState = States.FOLLOW_LINE_ODOMETRY
            previousState = States.FIND_LINE
            print("target row:", targetRow)
            print("target fruit:", FruitTypes.toString(targetFruit))
            print("x odom:", drive.odometry.xMeters)
            States.printTransition()
        else:
            drive.stop()
            currentState = States.YIPEE
            previousState = States.FIND_LINE
            print("all fruits complete")
            States.printTransition()

def YIPEE():
    drive.applySpeeds(0, 0, RobotConstants.MAX_TURN_SPEED_RAD_PER_SEC)
    if lift.atTarget():
        if lift.targetPosition == Lift.STOW_POSITION:
            lift.setMidPosition()
        else:
            lift.setStowPosition() 
    
    if gate.atTarget():
        if gate.targetRad == Gate.LOCKED_RAD:
            gate.setUnlockedPosition()
        else:
            gate.setLockedPosition()

def robotPeriodic():
    timer.event(robotPeriodic, RobotConstants.LOOP_PERIOD_MSEC)

    drive.periodic()
    lift.periodic()
    gate.periodic()
    vision.periodic()
    frontLine.periodic()

    if currentState == States.IDLE:
        IDLE()
    elif currentState == States.INIT:
        INIT()
    elif currentState == States.FOLLOW_LINE_ODOMETRY:
        if targetRow == 0:
            FOLLOW_LINE_ODOMETRY(RIGHT, FieldConstants.FIRST_ROW_X_METERS)
        elif targetRow == 1:
            FOLLOW_LINE_ODOMETRY(RIGHT, FieldConstants.SECOND_ROW_X_METERS)
        elif targetRow == 2:
            FOLLOW_LINE_ODOMETRY(RIGHT, FieldConstants.THIRD_ROW_X_METERS)
        elif targetRow == 3:
            FOLLOW_LINE_ODOMETRY(RIGHT, FieldConstants.FOURTH_ROW_X_METERS)
        elif targetRow == 4:
            FOLLOW_LINE_ODOMETRY(RIGHT, FieldConstants.FIFTH_ROW_X_METERS)
        elif targetRow == 5:
            FOLLOW_LINE_ODOMETRY(RIGHT, FieldConstants.SIXTH_ROW_X_METERS)
    elif currentState == States.FACE_DIRECTION:
        if targetRow % 2 == 0 or previousState == States.FIND_FRUIT:
            FACE_DIRECTION(FORWARD)
        else:
            FACE_DIRECTION(REVERSE)
    elif currentState == States.FIND_FRUIT:
        FIND_FRUIT(RIGHT)
    elif currentState == States.APPROACH_FRUIT:
        APPROACH_FRUIT()
    elif currentState == States.PICK_FRUIT:
        PICK_FRUIT()
    elif currentState == States.FOLLOW_LINE_SONAR:
        FOLLOW_LINE_SONAR(LEFT, FieldConstants.DISTANCE_TO_BASKET_WALL_METERS)
    elif currentState == States.FIND_BASKET:
        FIND_BASKET(vision.currentFruit)
    elif currentState == States.DUMP_FRUIT:
        DUMP_FRUIT()
    elif currentState == States.FIND_LINE:
        FIND_LINE()
    elif currentState == States.YIPEE:
        YIPEE()
        
    # print(lift.getSonarDistanceMeters())
    # print("x: " + str(drive.odometry.xMeters), "y: " + str(drive.odometry.yMeters), "theta: " + str(drive.odometry.thetaRad))
    
def handleButtonPress():
    global currentState
    global previousState

    if currentState == States.IDLE:
        drive.gyroStartedCalibrating = False
        drive.gyroHasCalibrated = False
        lift.hasStartedZeroing = False
        lift.hasZeroed = False
        currentState = States.INIT
        previousState = States.IDLE
        lift.rightLift.reset_position()
        lift.leftLift.reset_position()
        States.printTransition()
    elif currentState == States.INIT:
        if drive.gyroHasCalibrated and lift.hasZeroed:
            currentState = States.FOLLOW_LINE_ODOMETRY
            previousState = States.INIT
            dumpTimer.reset()
            timer.event(setDumpFruitXOdom, int((3 * RobotConstants.DUMP_DRIVE_TIME_MSEC) + RobotConstants.DUMP_TIME_MSEC))

            print("target row:", str(targetRow))
            print("target fruit", FruitTypes.toString(vision.currentFruit))
            States.printTransition()
    else:
        previousState = currentState
        currentState = States.IDLE
        States.printTransition()

button.pressed(handleButtonPress)

print("-> IDLE")
robotPeriodic()