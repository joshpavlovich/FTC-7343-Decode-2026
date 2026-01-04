package org.firstinspires.ftc.teamcode.subsystem

import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.NullCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.CRServoEx
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.powerable.SetPower

const val FLYWHEEL_MOTOR_RPM_BACK_LAUNCH_ZONE = 2600.0
const val FLYWHEEL_MOTOR_RPM_FRONT_LAUNCH_ZONE = 3600.0
const val FLYWHEEL_MOTOR_RPM_BACK_AUTO_LAUNCH_ZONE = 2400.0
const val FLYWHEEL_MOTOR_RPM_FRONT_AUTO_LAUNCH_ZONE = 3200.0

private const val KICKER_SERVO_DOWN_POSITION = 0.0
private const val KICKER_SERVO_UP_POSITION = 0.35

private const val ENCODER_TICKS_PER_REV = 28.0

private const val MAX_MOTOR_RPM = 4000.0

object FlywheelShooterSubsystem : Subsystem {

    // TODO: TUNE THESE VALUES???
    private val flywheelPID = PIDCoefficients(0.008, 0.0, 0.0)
    private val flywheelFF = BasicFeedforwardParameters(0.00055, 0.0, 0.0)

    private val flywheelController = controlSystem {
        velPid(flywheelPID)
        basicFF(flywheelFF)
    }

    private lateinit var motors: MotorGroup

    private val kickerServo by lazy { ServoEx("kicker_servo") }

    private val transferServoBottomLeft by lazy { CRServoEx("transfer_servo_bottom_left") }
    private val transferServoBottomRight by lazy { CRServoEx("transfer_servo_bottom_right") }
    private val transferServoTopLeft by lazy { CRServoEx("transfer_servo_top_left") }
    private val transferServoTopRight by lazy { CRServoEx("transfer_servo_top_right") }

    override fun initialize() {
        super.initialize()
        motors = MotorGroup(
            MotorEx("flywheel_motor_left"),
            MotorEx("flywheel_motor_right").reversed()
        )

        // Set the initial goal to 0 velocity.
        flywheelController.goal = KineticState(velocity = 0.0)

        transferServoTopLeft.power = 0.0
        transferServoBottomLeft.power = 0.0
        transferServoTopRight.power = 0.0
        transferServoBottomRight.power = 0.0

        kickerServo.position = KICKER_SERVO_DOWN_POSITION
    }

    override fun periodic() {
        val motorPower = flywheelController.calculate(motors.state)

        ActiveOpMode.telemetry.addData("Calculated Power", motorPower)

        motors.power = motorPower

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

        ActiveOpMode.telemetry.addData("Kicker Servo Position", kickerServo.position)
    }

    val kickArtifact
        get() = startTransfer.and(InstantCommand {
            if (kickerServo.servo.position == KICKER_SERVO_DOWN_POSITION) {
                kickerServo.position = KICKER_SERVO_UP_POSITION
            }
        }.requires(this))

    val resetKickerServo
        get() = InstantCommand {
            kickerServo.position = KICKER_SERVO_DOWN_POSITION
        }.requires(this)

    /**
     * Sets the target velocity of the flywheel.
     * @param rpm The target velocity in ticks per second.
     */
    fun startSpin(rpm: Double): Command = InstantCommand {
        flywheelController.goal = KineticState(0.0, (rpm / 60.0) * ENCODER_TICKS_PER_REV)
    }

    val stopSpin get() = startSpin(0.0).and(stopTransfer)

    // TODO: CHANGE MOTOR DIRECTION (-1.0 POWER TO REVERSE) ACCORDING TO LOCATION OF SERVO (RIGHT OR LEFT AND UP OR DOWN)
    val startTransfer
        get() = SetPower(transferServoBottomLeft, 1.0).requires(this).and(
            SetPower(transferServoBottomRight, -1.0).requires(this),
            SetPower(transferServoTopLeft, 1.0).requires(this),
            SetPower(transferServoTopRight, -1.0).requires(this)
        )

    val stopTransfer
        get() = SetPower(transferServoBottomLeft, 0.0).requires(this).and(
            SetPower(transferServoBottomRight, 0.0).requires(this),
            SetPower(transferServoTopLeft, 0.0).requires(this),
            SetPower(transferServoTopRight, 0.0).requires(this)
        )

    val autoStopTransfer: Command
        get() = if (transferServoBottomLeft.power > 0) {
            stopTransfer
        } else NullCommand()


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
        val airAndGravityCurve = -0.0148529 // The "a" Value: The "Air & Gravity Curve" (Curvature)
        val linearGrowth = 14.282 // The "b" Value: The "Linear Growth" (Slope)
        val baselineRpm = 1714.48071 // The "c" Value: The "Baseline Power" (Y-Intercept)

        val targetRpm =
            (airAndGravityCurve * (distanceInches * distanceInches)) + (linearGrowth * distanceInches) + baselineRpm
        return targetRpm.coerceIn(baselineRpm, MAX_MOTOR_RPM)
    }
}
