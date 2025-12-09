package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.CRServoEx
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.robotcore.external.Telemetry

const val FLYWHEEL_MOTOR_POWER_BACK_LAUNCH_ZONE = 0.50
const val FLYWHEEL_MOTOR_POWER_FRONT_LAUNCH_ZONE = 0.60

private const val KICKER_SERVO_DOWN_POSITION = 0.0
private const val KICKER_SERVO_UP_POSITION = 0.35

object FlywheelShooterSubsystem : Subsystem {

    var targetVelocity: Double = 1700.0

    var kP: Double = 0.001
    var kI: Double = 0.00000000001
    var kD: Double = 0.001
    var kV: Double = 0.0004
    var kA: Double = 0.0
    var kS: Double = 0.0

    var kP2: Double = 0.001
    var kI2: Double = 0.00000000001
    var kD2: Double = 0.001
    var kV2: Double = 0.00042
    var kA2: Double = 0.0
    var kS2: Double = 0.0

    val pid: PIDCoefficients = PIDCoefficients(kP, kI, kD)
    val ff: BasicFeedforwardParameters = BasicFeedforwardParameters(kV, kA, kS)

    val pid2: PIDCoefficients = PIDCoefficients(kP2, kI2, kD2)
    val ff2: BasicFeedforwardParameters = BasicFeedforwardParameters(kV2, kA2, kS2)

    private val controllerRight: ControlSystem = ControlSystem.builder()
        .velPid(pid)
        .basicFF(ff)
        .build()

    private val controllerLeft: ControlSystem = ControlSystem.builder()
        .velPid(pid2)
        .basicFF(ff2)
        .build()

    private val flyWheelMotorLeft by lazy { MotorEx("flywheel_motor_left") }
    private val flyWheelMotorRight by lazy { MotorEx("flywheel_motor_right") }

    private val kickerServo by lazy { ServoEx("kicker_servo") }

    private val transferServoTopLeft by lazy { CRServoEx("transfer_servo_top_left") }
    private val transferServoBottomLeft by lazy { CRServoEx("transfer_servo_bottom_left") }
    private val transferServoTopRight by lazy { CRServoEx("transfer_servo_top_right") }
    private val transferServoBottomRight by lazy { CRServoEx("transfer_servo_bottom_right") }

    val superLongShot = InstantCommand { setTargetVelocity(2300.0) }
        .and(transfer().afterTime(1.5)).requires(this)
    val longShot = InstantCommand { setTargetVelocity(1560.0) }
        .and(transfer().afterTime(1.5)).requires(this)
    val shortShot = InstantCommand { setTargetVelocity(1430.0) }
        .and(transfer().afterTime(1.5)).requires(this)
    val stop = InstantCommand { setTargetVelocity(0.0) }.requires(this)

    override fun initialize() {
        flyWheelMotorLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        flyWheelMotorRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // TODO: INITIALIZE KICKER SERVO POSITION??? 0.0 or the kicker servo's down position???
        kickerServo.position = KICKER_SERVO_DOWN_POSITION
    }

    fun kickArtifact() = InstantCommand {
        if (kickerServo.servo.position == KICKER_SERVO_DOWN_POSITION) kickerServo.position =
            KICKER_SERVO_UP_POSITION
    }.requires(this)

    fun resetKickerServo() = InstantCommand {
        kickerServo.position = KICKER_SERVO_DOWN_POSITION
    }.requires(this)

    // TODO: DO THE SERVOS NEED TO BE DELAYED, SO THAT THE FLYWHEEL CAN SPIN TO THE DESIRED VELOCITY
    // FIRST IF THEY ARE IN A PARALLEL GROUP? SEE, https://nextftc.dev/nextftc/commands/delays
    fun spin(power: Double = 1.0) = SetPower(flyWheelMotorLeft, power).requires(this).and(
        SetPower(flyWheelMotorRight, power).requires(this),
        // TODO: CHANGE WHEN WE KNOW HOW LONG IT TAKES TO SPIN THE FLYWHEEL
        transfer().afterTime(1.5)
        // TODO: CHANGE WHEN WE KNOW HOW LONG IT TAKES TO SPIN THE FLYWHEEL
    )

    fun stopSpin() = SetPower(flyWheelMotorLeft, 0.0).requires(this).and(
        SetPower(flyWheelMotorRight, 0.0).requires(this),
        stopTransfer()
    )

    // TODO: CHANGE MOTOR DIRECTION (-1.0 POWER TO REVERSE) ACCORDING TO LOCATION OF SERVO (RIGHT OR LEFT AND UP OR DOWN)
    private fun transfer() = SetPower(transferServoBottomLeft, 1.0).requires(this).and(
        SetPower(transferServoBottomRight, -1.0).requires(this),
        SetPower(transferServoTopLeft, 1.0).requires(this),
        SetPower(transferServoTopRight, -1.0).requires(this)
    )

    private fun stopTransfer() = SetPower(transferServoBottomLeft, 0.0).requires(this).and(
        SetPower(transferServoBottomRight, 0.0).requires(this),
        SetPower(transferServoTopLeft, 0.0).requires(this),
        SetPower(transferServoTopRight, 0.0).requires(this)
    )

    fun Telemetry.addShooterDetails() {
        addData("Flywheel Motor Left Power", flyWheelMotorLeft.power)
        addData("Flywheel Motor Left Velocity", flyWheelMotorLeft.velocity)
        addData("Flywheel Motor Right Power", flyWheelMotorRight.power)
        addData("Flywheel Motor Right Velocity", flyWheelMotorRight.velocity)

        addData("Kicker Servo Position", kickerServo.position)
    }

    override fun periodic() {
        super.periodic()
        flyWheelMotorLeft.power = controllerLeft.calculate(flyWheelMotorLeft.state)
        flyWheelMotorRight.power = controllerRight.calculate(flyWheelMotorRight.state)
    }

    fun setTargetVelocity(velocity: Double) {
        targetVelocity = velocity
        // This is the correct way to set the target. The periodic() method
        // will handle the rest.
        controllerRight.goal = KineticState(0.0, targetVelocity, 0.0)
        controllerLeft.goal = KineticState(0.0, -targetVelocity, 0.0)
    }
}