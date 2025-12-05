package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.CRServoEx
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.robotcore.external.Telemetry

private const val KICKER_SERVO_DOWN_POSITION = 0.0
private const val KICKER_SERVO_UP_POSITION = 0.25

object FlywheelShooterSubsystem : Subsystem {

    private val flyWheelMotorLeft by lazy { MotorEx("flywheel_motor_left") }
    private val flyWheelMotorRight by lazy { MotorEx("flywheel_motor_right") }

    private val kickerServo by lazy { ServoEx("kicker_servo") }

    private val transferServoTopLeft by lazy { CRServoEx("transfer_servo_top_left") }
    private val transferServoBottomLeft by lazy { CRServoEx("transfer_servo_bottom_left") }
    private val transferServoTopRight by lazy { CRServoEx("transfer_servo_top_right") }
    private val transferServoBottomRight by lazy { CRServoEx("transfer_servo_bottom_right") }

    override fun initialize() {
        flyWheelMotorLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        flyWheelMotorRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // TODO: INITIALIZE KICKER SERVO DIRECTION AND POSITION???
//        kickerServo.servo.direction = Servo.Direction.REVERSE
        // TODO: INITIALIZE KICKER SERVO POSITION??? 0.0 or the kicker servo's down position???
//        kickerServo.position = KICKER_SERVO_DOWN_POSITION
    }

    fun kickArtifact() = InstantCommand {
        if (kickerServo.servo.position == KICKER_SERVO_DOWN_POSITION) kickerServo.position = KICKER_SERVO_UP_POSITION
    }.requires(this)

    fun resetKickerServo() = InstantCommand {
        kickerServo.position = KICKER_SERVO_DOWN_POSITION
    }.requires(this)

    // TODO: DO THE SERVOS NEED TO BE DELAYED, SO THAT THE FLYWHEEL CAN SPIN TO THE DESIRED VELOCITY
    // FIRST IF THEY ARE IN A PARALLEL GROUP? SEE, https://nextftc.dev/nextftc/commands/delays
    fun spin(power: Double = 1.0) = SetPower(flyWheelMotorLeft, power).requires(this).and(
        SetPower(flyWheelMotorRight, power).requires(this)
    )

    fun stopSpin() = SetPower(flyWheelMotorLeft, 0.0).requires(this).and(
        SetPower(flyWheelMotorRight, 0.0).requires(this)
    )

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
        addData("Flywheel Motor Left Power", flyWheelMotorLeft.power)
        addData("Flywheel Motor Left Velocity", flyWheelMotorLeft.velocity)
        addData("Flywheel Motor Right Power", flyWheelMotorRight.power)
        addData("Flywheel Motor Right Velocity", flyWheelMotorRight.velocity)

        addData("Kicker Servo Position", kickerServo.position)
    }
}