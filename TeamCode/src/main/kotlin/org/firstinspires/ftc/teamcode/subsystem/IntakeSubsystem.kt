package org.firstinspires.ftc.teamcode.subsystem

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

/**
 * IntakeSubsystem manages the motor responsible for intaking artifacts into the robot.
 * It provides simple commands for forward, reverse, and stopping the motor.
 */
object IntakeSubsystem : Subsystem {

    private lateinit var intakeMotor: MotorEx

    /**
     * Initializes the intake motor with the specified hardware name, direction, and brake mode.
     */
    override fun initialize() {
        intakeMotor = MotorEx("intake_motor").reversed().brakeMode()
    }

    /**
     * Command to run the intake motor forward at full power.
     */
    val forward = InstantCommand { intakeMotor.power = 1.0 }.requires(this)

    /**
     * Command to run the intake motor in reverse at full power.
     */
    val reverse = InstantCommand { intakeMotor.power = -1.0 }.requires(this)

    /**
     * Command to stop the intake motor.
     */
    val stop = InstantCommand { intakeMotor.power = 0.0 }.requires(this)
}
