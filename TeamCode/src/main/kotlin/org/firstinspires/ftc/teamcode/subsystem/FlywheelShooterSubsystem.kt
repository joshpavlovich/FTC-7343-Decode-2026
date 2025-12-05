package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.CRServoEx
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.robotcore.external.Telemetry

object FlywheelShooterSubsystem : Subsystem {

    private val flyWheelMotor by lazy { MotorEx("flywheel_motor") }

    private val transferServoTopLeft by lazy { CRServoEx("transfer_servo_top_left") }
    private val transferServoBottomLeft by lazy { CRServoEx("transfer_servo_bottom_left") }
    private val transferServoTopRight by lazy { CRServoEx("transfer_servo_top_right") }
    private val transferServoBottomRight by lazy { CRServoEx("transfer_servo_bottom_right") }

    override fun initialize() {
        flyWheelMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    // TODO: DO THE SERVOS NEED TO BE DELAYED, SO THAT THE FLYWHEEL CAN SPIN TO THE DESIRED VELOCITY
    // FIRST IF THEY ARE IN A PARALLEL GROUP? SEE, https://nextftc.dev/nextftc/commands/delays
    fun spin(power: Double = 1.0) = SetPower(flyWheelMotor, power).requires(this)

    fun stopSpin() = SetPower(flyWheelMotor, 0.0).requires(this)

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
        addData("Flywheel Motor Power", flyWheelMotor.power)
        addData("Flywheel Motor Velocity", flyWheelMotor.velocity)
    }
}