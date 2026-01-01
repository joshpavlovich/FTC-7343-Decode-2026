package org.firstinspires.ftc.teamcode.subsystem

import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.NullCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.CRServoEx
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.powerable.SetPower

const val FLYWHEEL_MOTOR_POWER_BACK_LAUNCH_ZONE = 0.50
const val FLYWHEEL_MOTOR_POWER_BACK_LAUNCH_MID_AUTO_ZONE = 0.45
const val FLYWHEEL_MOTOR_POWER_FRONT_AUTO_LAUNCH_ZONE = 0.56
const val FLYWHEEL_MOTOR_POWER_FRONT_LAUNCH_ZONE = 0.625

const val FLYWHEEL_MOTOR_VELOCITY_BACK_LAUNCH_ZONE = 2600.0
const val FLYWHEEL_MOTOR_VELOCITY_FRONT_LAUNCH_ZONE = 3600.0

const val FLYWHEEL_MOTOR_VELOCITY_BACK_AUTO_LAUNCH_ZONE = 2200.0
const val FLYWHEEL_MOTOR_VELOCITY_FRONT_AUTO_LAUNCH_ZONE = 3200.0

private const val KICKER_SERVO_DOWN_POSITION = 0.0
private const val KICKER_SERVO_UP_POSITION = 0.35

private const val ENCODER_TICKS_PER_REV = 28.0

// Define your motor's physical limits
private const val MAX_VELOCITY = 4000.0 // The fastest your motor can safely spin
private const val MIN_VELOCITY = 1714.48071 // Usually 0, or your 'c' value (1714.0)

object FlywheelShooterSubsystem : Subsystem {

    // TODO: Tune these values???
    private val flywheelPID = PIDCoefficients(0.008, 0.0, 0.0)
    private val flywheelFF = BasicFeedforwardParameters(0.00055, 0.0, 0.0)

    private val flywheelController = controlSystem {
        velPid(flywheelPID)
        basicFF(flywheelFF)
    }

    private lateinit var motors: MotorGroup

    private lateinit var flywheelMotorLeft: MotorEx
    private lateinit var flywheelMotorRight: MotorEx

//    private lateinit var voltageSensor: VoltageSensor
    // A nominal voltage for voltage compensation.
    // TODO: You may need to tune this value for your robot's battery.
//    private const val NOMINAL_VOLTAGE = 12.0

    private val kickerServo by lazy { ServoEx("kicker_servo") }

    private val transferServoBottomLeft by lazy { CRServoEx("transfer_servo_bottom_left") }
    private val transferServoBottomRight by lazy { CRServoEx("transfer_servo_bottom_right") }
    private val transferServoTopLeft by lazy { CRServoEx("transfer_servo_top_left") }
    private val transferServoTopRight by lazy { CRServoEx("transfer_servo_top_right") }

    override fun initialize() {
        super.initialize()
        flywheelMotorLeft = MotorEx("flywheel_motor_left")
        flywheelMotorRight = MotorEx("flywheel_motor_right").reversed()
        motors = MotorGroup(flywheelMotorLeft, flywheelMotorRight)

        // Set the initial goal to 0 velocity.
        flywheelController.goal = KineticState(velocity = 0.0)

        transferServoTopLeft.power = 0.0
        transferServoBottomLeft.power = 0.0
        transferServoTopRight.power = 0.0
        transferServoBottomRight.power = 0.0

        kickerServo.position = KICKER_SERVO_DOWN_POSITION
//        voltageSensor = ActiveOpMode.hardwareMap.voltageSensor.first()
    }

    override fun periodic() {
        val motorPower = flywheelController.calculate(motors.state)

        ActiveOpMode.telemetry.addData("Calculated Power", motorPower)

// TODO: DO WE NEED VOLTAGE COMPENSATION???
//        val voltageMultiplier = NOMINAL_VOLTAGE / voltageSensor.voltage
//        ActiveOpMode.telemetry.addData("Voltage Multiplier", voltageMultiplier)
//        ActiveOpMode.telemetry.addData("Calculated Voltage Power", motorPower * voltageMultiplier)
// TODO: DO WE NEED VOLTAGE COMPENSATION???

        motors.power = motorPower //* voltageMultiplier

        ActiveOpMode.telemetry.addData("Flywheel Motor's Power", motors.power)
        ActiveOpMode.telemetry.addData("Flywheel Motor Left Power", flywheelMotorLeft.power)
        ActiveOpMode.telemetry.addData("Flywheel Motor Right Power", flywheelMotorRight.power)
        ActiveOpMode.telemetry.addData(
            "Target velocity:",
            flywheelController.goal.velocity / ENCODER_TICKS_PER_REV * 60.0
        )
        ActiveOpMode.telemetry.addData(
            "Flywheel Motor's Velocity",
            motors.velocity / ENCODER_TICKS_PER_REV * 60.0
        )
        ActiveOpMode.telemetry.addData(
            "Flywheel Motor Left Velocity",
            flywheelMotorLeft.velocity / ENCODER_TICKS_PER_REV * 60.0
        )
        ActiveOpMode.telemetry.addData(
            "Flywheel Motor Right Velocity",
            flywheelMotorRight.velocity / ENCODER_TICKS_PER_REV * 60.0
        )

        ActiveOpMode.telemetry.addData("Kicker Servo Position", kickerServo.position)
    }

    val kickArtifact
        get() = startTransfer.and(InstantCommand {
            if (kickerServo.servo.position == KICKER_SERVO_DOWN_POSITION) {
                kickerServo.position = KICKER_SERVO_UP_POSITION
            }
        }.requires(this))

    val resetKickerServo
        get() = InstantCommand {
            kickerServo.position = KICKER_SERVO_DOWN_POSITION
        }.requires(this)

    /**
     * Sets the target velocity of the flywheel.
     * @param velocity The target velocity in ticks per second.
     */
    fun startSpin(velocity: Double = 1200.0): Command = InstantCommand {
        flywheelController.goal = KineticState(0.0, (velocity / 60.0) * ENCODER_TICKS_PER_REV)
    }

    val stopSpin get() = startSpin(0.0).and(stopTransfer)

    // TODO: CHANGE MOTOR DIRECTION (-1.0 POWER TO REVERSE) ACCORDING TO LOCATION OF SERVO (RIGHT OR LEFT AND UP OR DOWN)
    val startTransfer
        get() = SetPower(transferServoBottomLeft, 1.0).requires(this).and(
            SetPower(transferServoBottomRight, -1.0).requires(this),
            SetPower(transferServoTopLeft, 1.0).requires(this),
            SetPower(transferServoTopRight, -1.0).requires(this)
        )

    val stopTransfer
        get() = SetPower(transferServoBottomLeft, 0.0).requires(this).and(
            SetPower(transferServoBottomRight, 0.0).requires(this),
            SetPower(transferServoTopLeft, 0.0).requires(this),
            SetPower(transferServoTopRight, 0.0).requires(this)
        )


    val autoStopTransfer: Command
        get() = if (transferServoBottomLeft.power > 0) {
            stopTransfer
        } else NullCommand()

    // https://github.com/AtomicRobotics3805/Decode/blob/leaguemeet2/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/AutoAdjustingCalc.kt
    // Adapted from AutoAdjustingCalc.calculatePower()
    fun calculateRpm(distance: Double): Double {
        // If the sensor doesn't see the target, don't spin the motor at all
        if (distance <= 0) return 0.0

        val rawVelocity = (0.0357839 * (distance * distance)) + (8.43768 * distance) + MIN_VELOCITY
        return rawVelocity.coerceIn(MIN_VELOCITY, MAX_VELOCITY)
    }
}