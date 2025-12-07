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
private const val KICKER_SERVO_UP_POSITION = 0.35

object KickerSubsystem : Subsystem {

    private val kickerServo by lazy { ServoEx("kicker_servo") }

    override fun initialize() {
        // TODO: INITIALIZE KICKER SERVO POSITION??? 0.0 or the kicker servo's down position???
        kickerServo.position = KICKER_SERVO_DOWN_POSITION
    }

    fun kickArtifact() = InstantCommand {
        if (kickerServo.servo.position == KICKER_SERVO_DOWN_POSITION) kickerServo.position = KICKER_SERVO_UP_POSITION
    }.requires(this)

    fun resetKickerServo() = InstantCommand {
        kickerServo.position = KICKER_SERVO_DOWN_POSITION
    }.requires(this)

    fun Telemetry.addShooterDetails() {
        addData("Kicker Servo Position", kickerServo.position)
    }
}