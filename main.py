from vex import *
brain=Brain()
brain_inertial = Inertial()
LeftMotor = Motor(Ports.PORT6, False)
RightMotor = Motor(Ports.PORT1, True)
Flywheel_motor_a = Motor(Ports.PORT3, True)
Flywheel_motor_b = Motor(Ports.PORT2, False)
Flywheel = MotorGroup(Flywheel_motor_a, Flywheel_motor_b)
BlueDispenser = Motor(Ports.PORT4, True)
Ramp = Motor(Ports.PORT5, True)
touchled_11 = Touchled(Ports.PORT11)

vexcode_brain_precision = 0
vexcode_console_precision = 0
ForwardkP = 0
ForwardkI = 0
ForwardkD = 0
TurnkP = 0
TurnkI = 0
TurnkD = 0
error = 0
integral = 0
derivative = 0
preverror = 0
shootkP = 0
shootkI = 0
shootkD = 0
shooterror = 0
shootintegral = 0
shootderivative = 0
shootpreverror = 0
deltatime = 0.02
mmforward = 0
motorpower = 0
shootpower = 0
rampshuffle = Event()
flywheelpid = Event()
discshuffle = False
shooting = False

def Forward_distance(Forward_distance__distance):
    global ForwardkP, ForwardkI, ForwardkD, TurnkP, TurnkI, TurnkD, error, integral, derivative, preverror, shootkP, shootkI, shootkD, shooterror, shootintegral, shootderivative, shootpreverror, mmforward, motorpower, shootpower, vexcode_brain_precision, vexcode_console_precision, deltatime
    error = 0
    integral = 0
    derivative = 0
    preverror = 0
    LeftMotor.set_position(0, DEGREES)
    RightMotor.set_position(0, DEGREES)
    mmforward = Forward_distance__distance * 25.4
    error = mmforward - ((LeftMotor.position(DEGREES) + RightMotor.position(DEGREES)) / 2)
    while not abs(error) <= 3:
        error = mmforward - ((LeftMotor.position(DEGREES) + RightMotor.position(DEGREES)) / 2)
        integral += error * deltatime
        derivative = (error - preverror) / deltatime
        preverror = error
        motorpower = (ForwardkP * error + ForwardkI * integral) + ForwardkD * derivative
        if motorpower > 127:
            motorpower = 127
        elif motorpower < -127:
            motorpower = -127
        LeftMotor.set_velocity(motorpower, RPM)
        RightMotor.set_velocity(motorpower, RPM)
        LeftMotor.spin(FORWARD)
        RightMotor.spin(FORWARD)
        brain.screen.print(error)
        brain.screen.new_line()
        wait(20, MSEC)
    LeftMotor.stop()
    RightMotor.stop()
    wait(30, MSEC)
    
def Turn_degrees(Turn_degrees__degrees):
    global ForwardkP, ForwardkI, ForwardkD, TurnkP, TurnkI, TurnkD, error, integral, derivative, preverror, shootkP, shootkI, shootkD, shooterror, shootintegral, shootderivative, shootpreverror, mmforward, motorpower, shootpower, vexcode_brain_precision, vexcode_console_precision, deltatime
    error = 0
    integral = 0
    derivative = 0
    preverror = 0
    error = Turn_degrees__degrees - brain_inertial.rotation()
    while not abs(error) <= 2:
        error = Turn_degrees__degrees - brain_inertial.rotation()
        integral += error * deltatime
        derivative = (error - preverror) / deltatime
        preverror = error
        motorpower = (TurnkP * error + TurnkI * integral) + TurnkD * derivative
        if motorpower > 127:
            motorpower = 127
        elif motorpower < -127:
            motorpower = -127
        LeftMotor.set_velocity(motorpower, RPM)
        RightMotor.set_velocity(motorpower, RPM)
        LeftMotor.spin(FORWARD)
        RightMotor.spin(REVERSE)
        brain.screen.print(error)
        brain.screen.new_line()
        wait(20, MSEC)
    LeftMotor.stop()
    RightMotor.stop()
    wait(30, MSEC)
    
def Curve_distance(Curve_distance__distance, Curve_distance__distance_before_turn, Curve_distance__degrees, Curve_distance__correction_strength):
    global ForwardkP, ForwardkI, ForwardkD, TurnkP, TurnkI, TurnkD, error, integral, derivative, preverror, shootkP, shootkI, shootkD, shooterror, shootintegral, shootderivative, shootpreverror, mmforward, motorpower, shootpower, vexcode_brain_precision, vexcode_console_precision, deltatime
    errorDrive = 0
    integralDrive = 0
    derivativeDrive = 0
    preverrorDrive = 0
    errorAngle = 0
    integralAngle = 0
    derivativeAngle = 0
    preverrorAngle = 0
    motorpowerDrive = 0
    motorpowerAngle = 0
    LeftMotor.set_position(0, DEGREES)
    RightMotor.set_position(0, DEGREES)
    mmforward = Curve_distance__distance * 25.4
    errorDrive = mmforward - ((LeftMotor.position(DEGREES) + RightMotor.position(DEGREES)) / 2)
    errorAngle = Curve_distance__degrees - brain_inertial.rotation()
    while not abs(errorDrive) <= 5:
        errorDrive = mmforward - ((LeftMotor.position(DEGREES) + RightMotor.position(DEGREES)) / 2)
        integralDrive += errorDrive * deltatime
        derivativeDrive = (errorDrive - preverrorDrive) / deltatime
        preverrorDrive = errorDrive
        motorpowerDrive = ((ForwardkP * errorDrive + ForwardkI * integralDrive) + ForwardkD * derivativeDrive)
        errorAngle = Curve_distance__degrees - brain_inertial.rotation()
        integralAngle += errorAngle * deltatime
        derivativeAngle = (errorAngle - preverrorAngle) / deltatime
        preverrorAngle = errorAngle
        if abs(LeftMotor.position(DEGREES) * 0.0393701) < Curve_distance__distance_before_turn:
            angleOutput = 0
        else:
            angleOutput = 1
        motorpowerAngle = ((TurnkP * errorAngle + TurnkI * integralAngle) + TurnkD * derivativeAngle) * Curve_distance__correction_strength * angleOutput
        if motorpowerDrive > 127:
            motorpowerDrive = 127
        elif motorpowerDrive < -127:
            motorpowerDrive = -127
        if motorpowerAngle > 127:
            motorpowerAngle = 127
        elif motorpowerAngle < -127:
            motorpowerAngle = -127
        LeftMotor.set_velocity(motorpowerDrive + motorpowerAngle, RPM)
        RightMotor.set_velocity(motorpowerDrive - motorpowerAngle, RPM)
        LeftMotor.spin(FORWARD)
        RightMotor.spin(FORWARD)
        wait(20, MSEC)
    LeftMotor.stop()
    RightMotor.stop()
    wait(30, MSEC)
    
    
def Curve_distance2(Curve_distance__distance, Curve_distance__distance_before_turn, Curve_distance__degrees, Curve_distance__correction_strength):
    global ForwardkP, ForwardkI, ForwardkD, TurnkP, TurnkI, TurnkD, error, integral, derivative, preverror, shootkP, shootkI, shootkD, shooterror, shootintegral, shootderivative, shootpreverror, mmforward, motorpower, shootpower, vexcode_brain_precision, vexcode_console_precision, deltatime
    errorDrive = 0
    integralDrive = 0
    derivativeDrive = 0
    preverrorDrive = 0
    errorAngle = 0
    integralAngle = 0
    derivativeAngle = 0
    preverrorAngle = 0
    motorpowerDrive = 0
    motorpowerAngle = 0
    LeftMotor.set_position(0, DEGREES)
    RightMotor.set_position(0, DEGREES)
    mmforward = Curve_distance__distance * 25.4
    errorDrive = mmforward - ((LeftMotor.position(DEGREES) + RightMotor.position(DEGREES)) / 2)
    errorAngle = Curve_distance__degrees - brain_inertial.rotation()
    while not abs(errorDrive) <= 10:
        errorDrive = mmforward - ((LeftMotor.position(DEGREES) + RightMotor.position(DEGREES)) / 2)
        integralDrive += errorDrive * deltatime
        derivativeDrive = (errorDrive - preverrorDrive) / deltatime
        preverrorDrive = errorDrive
        motorpowerDrive = ((ForwardkP * errorDrive + ForwardkI * integralDrive) + ForwardkD * derivativeDrive)
        errorAngle = Curve_distance__degrees - brain_inertial.rotation()
        integralAngle += errorAngle * deltatime
        derivativeAngle = (errorAngle - preverrorAngle) / deltatime
        preverrorAngle = errorAngle
        if abs(LeftMotor.position(DEGREES) * 0.0393701) < Curve_distance__distance_before_turn:
            angleOutput = 0
        else:
            angleOutput = 1
        motorpowerAngle = ((TurnkP * errorAngle + TurnkI * integralAngle) + TurnkD * derivativeAngle) * Curve_distance__correction_strength * angleOutput
        if motorpowerDrive > 127:
            motorpowerDrive = 127
        elif motorpowerDrive < -127:
            motorpowerDrive = -127
        if motorpowerAngle > 127:
            motorpowerAngle = 127
        elif motorpowerAngle < -127:
            motorpowerAngle = -127
        LeftMotor.set_velocity(motorpowerDrive + motorpowerAngle, RPM)
        RightMotor.set_velocity(motorpowerDrive - motorpowerAngle, RPM)
        LeftMotor.spin(FORWARD)
        RightMotor.spin(FORWARD)
        wait(20, MSEC)
    LeftMotor.stop()
    RightMotor.stop()
    wait(30, MSEC)

def Shoot_times(Shoot_times__times):
    global ForwardkP, ForwardkI, ForwardkD, TurnkP, TurnkI, TurnkD, error, integral, derivative, preverror, shootkP, shootkI, shootkD, shooterror, shootintegral, shootderivative, shootpreverror, mmforward, motorpower, shootpower, vexcode_brain_precision, vexcode_console_precision, shooting
    Flywheel.spin(FORWARD)
    shooting = True
    flywheelpid.broadcast()
    for x in range(0, Shoot_times__times):
        Ramp.spin(FORWARD)
        wait(0.8, SECONDS)
        Ramp.stop()
        wait(0.3, SECONDS)
        Ramp.spin(REVERSE)
        wait(0.5, SECONDS)
        Ramp.stop()
    Ramp.stop()

def onevent_rampshuffle_0():
    global ForwardkP, ForwardkI, ForwardkD, TurnkP, TurnkI, TurnkD, error, integral, derivative, preverror, shootkP, shootkI, shootkD, shooterror, shootintegral, shootderivative, shootpreverror, mmforward, motorpower, shootpower, vexcode_brain_precision, vexcode_console_precision, discshuffle, shooting
    Ramp.spin_to_position(50, DEGREES)
    while discshuffle == True:
        Ramp.spin_to_position(50, DEGREES)
        wait(100, MSEC)
        Ramp.spin_to_position(170, DEGREES)
        wait(100, MSEC)
    Ramp.stop()
    discshuffle = False

def onevent_flywheelpid_0():
    global ForwardkP, ForwardkI, ForwardkD, TurnkP, TurnkI, TurnkD, error, integral, derivative, preverror, shootkP, shootkI, shootkD, shooterror, shootintegral, shootderivative, shootpreverror, mmforward, motorpower, shootpower, vexcode_brain_precision, vexcode_console_precision, shooting, flywheelspeed
    shootkP = 1
    shootkI = 0.13
    shootkD = 0
    shooterror = 0
    shootintegral = 0
    shootderivative = 0
    shootpreverror = 0
    flywheelspeed = 90
    while shooting == True:
        shooterror = flywheelspeed - ((Flywheel_motor_a.velocity(RPM) + Flywheel_motor_b.velocity(RPM)) / 2)
        shootintegral += shooterror
        shootderivative = shooterror - shootpreverror
        shootpreverror = shooterror
        shootpower = (shootkP * shooterror + shootkI * shootintegral) + shootkD * shootderivative
        if shootpower > 127:
            shootpower = 127
        elif shootpower < -127:
            shootpower = -127
        Flywheel_motor_a.set_velocity(shootpower, RPM)
        Flywheel_motor_b.set_velocity(shootpower, RPM)
        Flywheel.spin(FORWARD)
        wait(20, MSEC)
    Flywheel.stop()
    shooting = False
    flywheelspeed = 95

def when_started1():
    global ForwardkP, ForwardkI, ForwardkD, TurnkP, TurnkI, TurnkD, error, integral, derivative, preverror, shootkP, shootkI, shootkD, shooterror, shootintegral, shootderivative, shootpreverror, mmforward, motorpower, shootpower, vexcode_brain_precision, vexcode_console_precision, discshuffle, shooting, flywheelspeed
    ForwardkP = 0.78
    ForwardkI = 0.003
    ForwardkD = 0.04
    TurnkP = 0.78
    TurnkI = 0.06
    TurnkD = 0.05
    LeftMotor.set_stopping(HOLD)
    RightMotor.set_stopping(HOLD)
    Flywheel.set_stopping(BRAKE)
    Ramp.set_stopping(HOLD)
    BlueDispenser.set_stopping(HOLD)
    BlueDispenser.set_velocity(80, PERCENT)
    Ramp.set_velocity(95, PERCENT)
    LeftMotor.set_max_torque(90, PERCENT)
    RightMotor.set_max_torque(90, PERCENT)
    Flywheel.set_max_torque(100, PERCENT)
    BlueDispenser.set_max_torque(70, PERCENT)
    Ramp.set_max_torque(70, PERCENT)
    BlueDispenser.set_position(0, DEGREES)
    Ramp.set_position(50, DEGREES)
    Ramp.spin(FORWARD)
    wait(700, MSEC)
    Ramp.stop()
    LeftMotor.spin(REVERSE)
    RightMotor.spin(REVERSE)
    wait(400, MSEC)
    LeftMotor.stop()
    RightMotor.stop()
    BlueDispenser.set_position(0, DEGREES)
    brain_inertial.calibrate()
    while brain_inertial.is_calibrating():
       sleep(2000)
    touchled_11.set_color(Color.RED)
    while not touchled_11.pressing():
       wait(20, MSEC)
    touchled_11.set_color(Color.GREEN)
    '''
    Forward_distance(-14)
    Forward_distance(-5)
    Ramp.spin(FORWARD)
    wait(0.65, SECONDS)
    Ramp.stop()
    wait(100, MSEC)
    Ramp.spin_to_position(50, DEGREES)
    LeftMotor.spin(REVERSE)
    RightMotor.spin(REVERSE)
    wait(450, MSEC)
    LeftMotor.stop()
    RightMotor.stop()
    Ramp.spin_to_position(150, DEGREES, wait=False)
    Forward_distance(10)
    Turn_degrees(45)
    discshuffle = True
    rampshuffle.broadcast()
    Forward_distance(17)
    Turn_degrees(0)
    Forward_distance(8)
    discshuffle = False
    Flywheel.set_velocity(120, RPM)
    Flywheel.spin(FORWARD)
    Forward_distance(1)
    Ramp.spin_to_position(200, DEGREES, wait=False)
    Forward_distance(1)
    Flywheel.set_velocity(115, RPM)
    for x in range(0, 2):
        Ramp.spin(FORWARD)
        wait(0.8, SECONDS)
        Ramp.stop()
        wait(0.3, SECONDS)
        Ramp.spin(REVERSE)
        wait(0.5, SECONDS)
        Ramp.stop()
    Ramp.spin(FORWARD)
    wait(0.8, SECONDS)
    Ramp.stop
    wait(0.3, SECONDS)
    Ramp.stop()
    Flywheel.stop()
    Turn_degrees(-45)
    Forward_distance(-8.5)
    Turn_degrees(0)
    Forward_distance(-5.4)
    '''
    Curve_distance(26, 7, 50, 2.3)
    Turn_degrees(0)
    Forward_distance(-3)
    LeftMotor.set_velocity(50, RPM)
    RightMotor.set_velocity(50, RPM)
    LeftMotor.spin(REVERSE)
    RightMotor.spin(REVERSE)
    wait(0.65, SECONDS)
    LeftMotor.stop()
    RightMotor.stop()
    BlueDispenser.spin(FORWARD)
    wait(0.4, SECONDS)
    BlueDispenser.stop()
    wait(0.3, SECONDS)
    BlueDispenser.spin(REVERSE)
    wait(290, MSEC)
    BlueDispenser.stop()
    Forward_distance(5.3)
    Turn_degrees(91.1)
    Flywheel.set_velocity(127, RPM)
    Flywheel.spin(FORWARD)
    Forward_distance(3)
    Ramp.spin_to_position(100, DEGREES)
    Forward_distance(3)
    discshuffle = True
    rampshuffle.broadcast()
    wait(1.7, SECONDS)
    Flywheel.stop()
    discshuffle = False
    Forward_distance(-8)
    Turn_degrees(-15)
    Ramp.stop()
    Ramp.spin_to_position(170, DEGREES, wait=False)
    LeftMotor.set_stopping(COAST)
    RightMotor.set_stopping(COAST)
    Forward_distance(12)
    LeftMotor.set_velocity(50, RPM)
    RightMotor.set_velocity(50, RPM)
    LeftMotor.set_stopping(COAST)
    RightMotor.set_stopping(COAST)
    LeftMotor.spin(FORWARD)
    RightMotor.spin(FORWARD)
    wait(800, MSEC)
    LeftMotor.stop()
    RightMotor.stop()
    discshuffle = False
    Shoot_times(2)
    Flywheel.spin(FORWARD)
    shooting = True
    flywheelpid.broadcast()
    Ramp.spin_to_position(200, DEGREES, wait=False)
    BlueDispenser.spin(FORWARD)
    wait(550, MSEC)
    BlueDispenser.stop()
    wait(500, MSEC)
    BlueDispenser.spin_to_position(0, DEGREES)
    wait(200, MSEC)
    Ramp.spin(REVERSE)
    wait(0.5, SECONDS)
    Ramp.stop()
    Shoot_times(4)
    Ramp.spin_to_position(220, DEGREES, wait=False)
    shooting = False
    Flywheel.stop()
    touchled_11.set_color(Color.RED)
    LeftMotor.set_velocity(80, RPM)
    RightMotor.set_velocity(80, RPM)
    LeftMotor.spin(REVERSE)
    RightMotor.spin(REVERSE)
    wait(2, SECONDS)
    LeftMotor.stop()
    RightMotor.stop()
    touchled_11.set_color(Color.RED)
    while not touchled_11.pressing():
        wait(20, MSEC)
    touchled_11.set_color(Color.GREEN)
    LeftMotor.set_stopping(HOLD)
    RightMotor.set_stopping(HOLD)
    brain_inertial.set_rotation(0, DEGREES)
    Curve_distance(25, 7, -50, 2.3)
    Turn_degrees(0)
    Forward_distance(-3)
    LeftMotor.set_velocity(50, RPM)
    RightMotor.set_velocity(50, RPM)
    LeftMotor.set_stopping(COAST)
    RightMotor.set_stopping(COAST)
    LeftMotor.spin(REVERSE)
    RightMotor.spin(REVERSE)
    wait(0.65, SECONDS)
    LeftMotor.stop()
    RightMotor.stop()
    BlueDispenser.spin(FORWARD)
    wait(0.4, SECONDS)
    BlueDispenser.stop()
    wait(0.3, SECONDS)
    BlueDispenser.spin(REVERSE)
    wait(290, MSEC)
    BlueDispenser.stop()
    Forward_distance(8)
    Curve_distance(23, 0.5, 90, 2)
    Turn_degrees(0)
    LeftMotor.set_stopping(HOLD)
    RightMotor.set_stopping(HOLD)
    LeftMotor.set_velocity(50, RPM)
    RightMotor.set_velocity(50, RPM)
    LeftMotor.spin(REVERSE)
    RightMotor.spin(REVERSE)
    wait(0.6, SECONDS)
    LeftMotor.stop()
    RightMotor.stop()
    LeftMotor.spin(FORWARD)
    RightMotor.spin(FORWARD)
    wait(0.8, SECONDS)
    LeftMotor.stop()
    RightMotor.stop()
    Ramp.spin_to_position(250, DEGREES, wait=False)
    Flywheel.spin(FORWARD)
    shooting = True
    flywheelpid.broadcast()
    BlueDispenser.spin_to_position(300, DEGREES, wait=False)
    Shoot_times(4)
    shooting = False
    Ramp.spin(FORWARD)
    Turn_degrees(-85)
    Ramp.stop()
    Forward_distance(25)
    Turn_degrees(-5)
    LeftMotor.set_velocity(50, RPM)
    RightMotor.set_velocity(50, RPM)
    LeftMotor.spin(FORWARD)
    RightMotor.spin(FORWARD)
    wait(0.7, SECONDS)
    BlueDispenser.spin(REVERSE)
    wait(1.5, SECONDS)
    BlueDispenser.spin_to_position(0, DEGREES)
    Flywheel.stop()
    Ramp.stop()
    BlueDispenser.stop()
    LeftMotor.stop()
    RightMotor.stop()

rampshuffle(onevent_rampshuffle_0)
flywheelpid(onevent_flywheelpid_0)
wait(15, MSEC)
when_started1()
