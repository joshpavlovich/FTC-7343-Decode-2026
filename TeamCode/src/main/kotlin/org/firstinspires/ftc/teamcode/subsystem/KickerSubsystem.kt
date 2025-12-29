package org.firstinspires.ftc.teamcode.subsystem

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.ServoEx

private const val KICKER_SERVO_DOWN_POSITION = 0.0
private const val KICKER_SERVO_UP_POSITION = 0.35

object KickerSubsystem : Subsystem {

    private lateinit var kickerServo: ServoEx

    override fun initialize() {
        kickerServo = ServoEx("kicker_servo")

        // TODO: INITIALIZE KICKER SERVO POSITION??? 0.0 or the kicker servo's down position???
        kickerServo.position = KICKER_SERVO_DOWN_POSITION
    }

    override fun periodic() {
        ActiveOpMode.telemetry.addData("Kicker Servo Position", kickerServo.position)
    }

    fun kickArtifact() = InstantCommand {
        if (kickerServo.servo.position == KICKER_SERVO_DOWN_POSITION) kickerServo.position =
            KICKER_SERVO_UP_POSITION
    }.requires(this)

    fun resetKickerServo() = InstantCommand {
        kickerServo.position = KICKER_SERVO_DOWN_POSITION
    }.requires(this)
}