package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.abs

class FlywheelSubsystem() : Subsystem {

    private val flywheelMotorLeft = MotorEx("flywheel_motor_left").brakeMode()
    private val flywheelMotorRight = MotorEx("flywheel_motor_right").brakeMode().reversed()

    private val voltageSensor by lazy { ActiveOpMode.hardwareMap.voltageSensor.first() }

    // Output wrapper
    private val kineticState = KineticState()

    // ===== NEXTFTC CONTROL SYSTEM =====
    private val controlSystem = controlSystem {
        velPid(
            PIDCoefficients(
                kP = 0.0002,
                kI = 0.000001,
                kD = 0.00001
            )
        )
        basicFF(
            BasicFeedforwardParameters(
                kV = 1.0 / 2500.0
            )
        )
    }

    // Target velocity (ticks/sec)
    private var targetVelocity = 0.0

    // Nominal FTC battery voltage
    private val nominalVoltage = 12.0

    override fun initialize() {
        super.initialize()

        flywheelMotorLeft.brakeMode()
        flywheelMotorRight.brakeMode().reversed()
    }

    override fun periodic() {
        val measuredVelocity = flywheelMotorLeft.velocity

        // PID + feedforward handled by NEXTFTC
        val controlOutput = controlSystem.calculate(
            targetVelocity,
            measuredVelocity
        )

        // ===== VOLTAGE COMPENSATION =====
        val voltageCompensation =
            nominalVoltage / voltageSensor.voltage

        val compensatedOutput =
            controlOutput * voltageCompensation

        kineticState.set(compensatedOutput.coerceIn(0.0, 1.0))

        flywheelMotorLeft.power = kineticState.output
        flywheelMotorRight.power = kineticState.output
    }

    /** Set target flywheel velocity (ticks/sec) */
    fun setTargetVelocity(ticksPerSecond: Double) {
        targetVelocity = ticksPerSecond
    }

    /** Stop flywheel and reset controller */
    fun stop() {
        targetVelocity = 0.0
        kineticState = KineticState(0.0)
        controlSystem.reset()
    }

    /** Check if flywheel is at speed */
    fun atSpeed(tolerance: Double = 50.0): Boolean {
        return abs(targetVelocity - flywheelMotorLeft.velocity) < tolerance
    }
}
