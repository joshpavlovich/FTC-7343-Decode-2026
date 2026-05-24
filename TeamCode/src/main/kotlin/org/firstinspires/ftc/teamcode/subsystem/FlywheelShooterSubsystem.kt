package org.firstinspires.ftc.teamcode.subsystem

import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx

/** RPM preset for shooting from the back launch zone. */
const val FLYWHEEL_MOTOR_RPM_BACK_LAUNCH_ZONE = 2700.0
/** RPM preset for shooting from the front launch zone. */
const val FLYWHEEL_MOTOR_RPM_FRONT_LAUNCH_ZONE = 3600.0

private const val ENCODER_TICKS_PER_REV = 28.0

private const val MAX_MOTOR_RPM = 6000.0

/**
 * FlywheelShooterSubsystem manages the flywheel motors and gate servo.
 * It uses a PIDF control system to maintain precise flywheel velocity and provides
 * commands for automated shooting and artifact transfer.
 */
object FlywheelShooterSubsystem : Subsystem {

    // PID and Feedforward coefficients for flywheel velocity control
    private val flywheelPID = PIDCoefficients(0.008, 0.0, 0.0)
    private val flywheelFF = BasicFeedforwardParameters(0.00055, 0.0, 0.0)

    private val flywheelController = controlSystem {
        velPid(flywheelPID)
        basicFF(flywheelFF)
    }

    private lateinit var motors: MotorGroup

    /**
     * Initializes the flywheel motors, setting initial goals and positions.
     */
    override fun initialize() {
        super.initialize()
        motors = MotorGroup(
            MotorEx("flywheel_motor_left").reversed(),
            MotorEx("flywheel_motor_right")
        )

        // Set the initial goal to 0 velocity.
        flywheelController.goal = KineticState(velocity = 0.0)
    }

    /**
     * Calculates and applies the required power to the flywheel motors based on the PIDF controller.
     * Updates telemetry with current velocity and status.
     */
    override fun periodic() {
        val motorPower = flywheelController.calculate(motors.state)
        motors.power = motorPower

        ActiveOpMode.telemetry.addData("Calculated Power", motorPower)
        ActiveOpMode.telemetry.addData("Flywheel Motor's Power", motors.power)

        try {
            ActiveOpMode.telemetry.addData(
                "Target velocity",
                flywheelController.goal.velocity / ENCODER_TICKS_PER_REV * 60.0
            )
            ActiveOpMode.telemetry.addData(
                "Flywheel Motor's Velocity",
                motors.velocity / ENCODER_TICKS_PER_REV * 60.0
            )
        } catch (_: Exception) {
        }
    }

    /**
     * Sets the target velocity of the flywheel in RPM.
     * @param rpm The target velocity in Revolutions Per Minute.
     * @return A Command to update the flywheel controller goal.
     */
    fun startSpin(rpm: Double): Command = InstantCommand {
        flywheelController.goal = KineticState(0.0, (rpm / 60.0) * ENCODER_TICKS_PER_REV)
    }

    /**
     * Command to stop the flywheel.
     */
    val stopSpin get() = startSpin(0.0)

    /**
     * Calculates the target RPM for the flywheel shooter based on the distance to the target.
     *
     * This function uses a quadratic formula derived from real-world test data to determine the
     * optimal flywheel speed for a given distance. The formula is in the form of `y = ax^2 + bx + c`,
     * where:
     * - `y` is the target RPM.
     * - `x` is the distance in inches.
     * - `a` (`airAndGravityCurve`) accounts for the air resistance and gravity affecting the projectile.
     * - `b` (`linearGrowth`) provides linear scaling for the RPM as distance increases.
     * - `c` (`baselineRpm`) is the base RPM for shooting at point-blank range.
     *
     * The coefficients `a`, `b`, and `c` are tuned to the specific physical characteristics of the
     * robot's shooter mechanism.
     *
     * Adapted from AutoAdjustingCalc.calculatePower(): https://github.com/AtomicRobotics3805/Decode/blob/leaguemeet2/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/AutoAdjustingCalc.kt
     *
     * @param distanceInches The distance to the target in inches.
     * @return The calculated target RPM for the flywheel, coerced to be within the safe operating
     *         range of the motor (0 to `MAX_MOTOR_RPM`). Returns 0.0 if the distance is
     *         less than or equal to 0.
     */
    fun calculateRpm(distanceInches: Double): Double {
        // If the sensor doesn't see the target, don't spin the motor at all
        if (distanceInches <= 0) return 0.0

        // Shooter's quadratic formula (y = ax^2 + bx + c), each letter represents a specific physical
        // behavior of your robot's launch system. Because we used Regression to fit the formula to your
        // real-world test data, these values capture the "personality" of your specific flywheel,
        // motors, and ball compression.
        //
        // | Coefficient | Role            | If you increase this... |
        // |     c       | Vertical Offset | Every single shot (near and far) will go higher/further. |
        // |     b       | Linear Scaling  | Long-range shots will increase in power much faster than close-range shots. |
        // |     a       | Curve Shape     | "Adjusts the ""arc"" consistency for extreme distances (150""+)." |
        // How to use this for "Quick Fixes":
        // If all shots are low: Add +50 to your "c" value.
        // If close shots are good, but far shots are low: Increase your "b" value slightly (e.g., from 14.28 to 14.50).
        //

        val airAndGravityCurve = 0.0663 // The "a" Value: The "Air & Gravity Curve" (Curvature)
        val linearGrowth = 0.887 // The "b" Value: The "Linear Growth" (Slope)
        val baselineRpm = 2295.5 // The "c" Value: The "Baseline Power" (Y-Intercept)

        val targetRpm =
            (airAndGravityCurve * (distanceInches * distanceInches)) + (linearGrowth * distanceInches) + baselineRpm
        return targetRpm.coerceIn(baselineRpm, MAX_MOTOR_RPM)
    }
}
