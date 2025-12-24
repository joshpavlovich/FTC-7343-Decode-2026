package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.CRServoEx
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.robotcore.external.Telemetry

const val FLYWHEEL_MOTOR_POWER_BACK_LAUNCH_ZONE = 0.50
const val FLYWHEEL_MOTOR_POWER_BACK_LAUNCH_MID_AUTO_ZONE = 0.45
const val FLYWHEEL_MOTOR_POWER_FRONT_AUTO_LAUNCH_ZONE = 0.56
const val FLYWHEEL_MOTOR_POWER_FRONT_LAUNCH_ZONE = 0.625

const val FLYWHEEL_MOTOR_VELOCITY_BACK_LAUNCH_ZONE = 2600.0
const val FLYWHEEL_MOTOR_VELOCITY_FRONT_LAUNCH_ZONE = 3600.0

const val FLYWHEEL_MOTOR_VELOCITY_BACK_AUTO_LAUNCH_ZONE = 2400.0
const val FLYWHEEL_MOTOR_VELOCITY_FRONT_AUTO_LAUNCH_ZONE = 3000.0

private const val KICKER_SERVO_DOWN_POSITION = 0.0
private const val KICKER_SERVO_UP_POSITION = 0.35

private const val ENCODER_TICKS_PER_REV = 28.0

object FlywheelShooterSubsystem : Subsystem {

    // TODO: Tune these values
    private val flywheelPID = PIDCoefficients(0.008, 0.0, 0.0)
    private val flywheelFF = BasicFeedforwardParameters(0.00055,0.0,0.0)

    private val flywheelController = controlSystem {
        velPid(flywheelPID)
        basicFF(flywheelFF)
    }

    private lateinit var motors: MotorGroup

    private lateinit var flywheelMotorLeft: MotorEx
    private lateinit var flywheelMotorRight: MotorEx

    private lateinit var voltageSensor: VoltageSensor

    // A nominal voltage for voltage compensation.
    // TODO: You may need to tune this value for your robot's battery.
    private const val NOMINAL_VOLTAGE = 12.0

    private val kickerServo by lazy { ServoEx("kicker_servo") }

    private val transferServoTopLeft by lazy { CRServoEx("transfer_servo_top_left") }
    private val transferServoBottomLeft by lazy { CRServoEx("transfer_servo_bottom_left") }
    private val transferServoTopRight by lazy { CRServoEx("transfer_servo_top_right") }
    private val transferServoBottomRight by lazy { CRServoEx("transfer_servo_bottom_right") }

    override fun initialize() {
        super.initialize()
        flywheelMotorLeft = MotorEx("flywheel_motor_left")
        flywheelMotorRight = MotorEx("flywheel_motor_right").reversed()
        motors = MotorGroup(flywheelMotorLeft, flywheelMotorRight)

        voltageSensor = ActiveOpMode.hardwareMap.voltageSensor.first()

        // Set the initial goal to 0 velocity.
        flywheelController.goal = KineticState(velocity = 0.0)

        kickerServo.position = KICKER_SERVO_DOWN_POSITION
    }

    override fun periodic() {
        val motorPower = flywheelController.calculate(motors.state)

        val voltageMultiplier = NOMINAL_VOLTAGE / voltageSensor.voltage

        ActiveOpMode.telemetry.addData("Calculated Power", motorPower)

        ActiveOpMode.telemetry.addData("Voltage Multiplier", voltageMultiplier)
        ActiveOpMode.telemetry.addData("Calculated Voltage Power", motorPower * voltageMultiplier)

        motors.power = motorPower //* voltageMultiplier
    }

    fun kickArtifact() = transfer().and(InstantCommand {
        if (kickerServo.servo.position == KICKER_SERVO_DOWN_POSITION) {
            kickerServo.position = KICKER_SERVO_UP_POSITION
        }
    }.requires(this))

    fun resetKickerServo() = InstantCommand {
        kickerServo.position = KICKER_SERVO_DOWN_POSITION
    }.requires(this)

    /**
     * Sets the target velocity of the flywheel.
     * @param velocity The target velocity in ticks per second.
     */
    fun spin(velocity: Double = 600.0): Command =
        RunToVelocity(flywheelController, (velocity / 60.0) * ENCODER_TICKS_PER_REV).requires(this)

    fun stopSpin() = spin(0.0).and(stopTransfer())

    // TODO: CHANGE MOTOR DIRECTION (-1.0 POWER TO REVERSE) ACCORDING TO LOCATION OF SERVO (RIGHT OR LEFT AND UP OR DOWN)
    fun transfer() = SetPower(transferServoBottomLeft, 1.0).requires(this).and(
        SetPower(transferServoBottomRight, -1.0).requires(this),
        SetPower(transferServoTopLeft, 1.0).requires(this),
        SetPower(transferServoTopRight, -1.0).requires(this)
    )

    fun stopTransfer() = SetPower(transferServoBottomLeft, 0.0).requires(this).and(
        SetPower(transferServoBottomRight, 0.0).requires(this),
        SetPower(transferServoTopLeft, 0.0).requires(this),
        SetPower(transferServoTopRight, 0.0).requires(this)
    )

    fun Telemetry.addShooterDetails() {
        addData("Flywheel Motors Power", motors.power)
        addData("Flywheel Motor Left Power", flywheelMotorLeft.power)
        addData("Flywheel Motor Right Power", flywheelMotorRight.power)
        addData("Flywheel Motor Left Velocity", flywheelMotorLeft.velocity)
        addData("Flywheel Motor Right Velocity", flywheelMotorRight.velocity)

        addData("Kicker Servo Position", kickerServo.position)
    }
}