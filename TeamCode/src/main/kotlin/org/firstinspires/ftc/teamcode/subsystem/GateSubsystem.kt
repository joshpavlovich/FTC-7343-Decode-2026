package org.firstinspires.ftc.teamcode.subsystem

import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.abs

/**
 * KickerSubsystem manages the servo responsible for "kicking" artifacts from the intake/transfer
 * into the flywheel shooter.
 */
object GateSubsystem : Subsystem {

    private lateinit var motor: MotorEx

    // Define the states with position only
    val OPEN_STATE = KineticState(position = 119.0)
    val CLOSED_STATE = KineticState(position = 0.0)

    // Define tolerance as a separate constant
    const val POSITION_TOLERANCE = 5.0
    const val POWER_CAP = 0.4

    // NextFTC ControlSystem setup
    private val controlSystem = controlSystem {
        posPid(0.008, 0.0, 0.0001)
        // You can also add feedforward here if needed later
    }

    var targetState = CLOSED_STATE
        private set

    override fun initialize() {
        super.initialize()

        motor = MotorEx("gate_motor").brakeMode().zeroed()
    }

    override fun periodic() {
        val currentPos = motor.currentPosition

        // Calculate the error state as the difference between target and current
        val errorState = KineticState(
            position = targetState.position - currentPos
        )

        val power = controlSystem.calculate(errorState)
        motor.power = power.coerceIn(-POWER_CAP, POWER_CAP)
    }

    val open = InstantCommand { targetState = OPEN_STATE }.requires(this)
    val close = InstantCommand { targetState = CLOSED_STATE }.requires(this)

    // Logic for checking if we reached the target
    fun isAtTarget(): Boolean =
        abs(targetState.position - motor.currentPosition) < POSITION_TOLERANCE
}
