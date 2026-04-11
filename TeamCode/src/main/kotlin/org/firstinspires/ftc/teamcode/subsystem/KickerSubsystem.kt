package org.firstinspires.ftc.teamcode.subsystem

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.ServoEx

private const val KICKER_SERVO_DOWN_POSITION = 0.0
private const val KICKER_SERVO_UP_POSITION = 0.35

/**
 * KickerSubsystem manages the servo responsible for "kicking" artifacts from the intake/transfer
 * into the flywheel shooter.
 */
object KickerSubsystem : Subsystem {

    private lateinit var kickerServo: ServoEx

    /**
     * Initializes the kicker servo and sets it to the default down position.
     */
    override fun initialize() {
        kickerServo = ServoEx("kicker_servo")

        // Set initial position to down
        kickerServo.position = KICKER_SERVO_DOWN_POSITION
    }

    /**
     * Sends the current kicker servo position to telemetry for debugging.
     */
    override fun periodic() {
        ActiveOpMode.telemetry.addData("Kicker Servo Position", kickerServo.position)
    }

    /**
     * Creates an InstantCommand to move the kicker servo to the up position,
     * effectively launching an artifact if one is present.
     *
     * @return An InstantCommand that requires this subsystem.
     */
    fun kickArtifact() = InstantCommand {
        if (kickerServo.servo.position == KICKER_SERVO_DOWN_POSITION) {
            kickerServo.position = KICKER_SERVO_UP_POSITION
        }
    }.requires(this)

    /**
     * Creates an InstantCommand to move the kicker servo back to the down position.
     *
     * @return An InstantCommand that requires this subsystem.
     */
    fun resetKickerServo() = InstantCommand {
        kickerServo.position = KICKER_SERVO_DOWN_POSITION
    }.requires(this)
}
