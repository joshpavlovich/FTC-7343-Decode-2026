package org.firstinspires.ftc.teamcode.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.subsystem.ColorSensorSubsystem
import org.firstinspires.ftc.teamcode.subsystem.FLYWHEEL_MOTOR_RPM_BACK_LAUNCH_ZONE
import org.firstinspires.ftc.teamcode.subsystem.FLYWHEEL_MOTOR_RPM_FRONT_LAUNCH_ZONE
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem

private const val RIGHT_TRIGGER_MINIMUM_VALUE = 0.5

/**
 * ManualTeleOp provides direct driver control over the robot's movement and subsystems.
 * It uses a Mecanum drive system and maps various gamepad buttons to intake, shooting, and transfer functions.
 */
@TeleOp(name = "Manual TeleOp")
class ManualTeleOp : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(FlywheelShooterSubsystem, ColorSensorSubsystem, IntakeSubsystem),
            BulkReadComponent,
            BindingsComponent
        )
    }

    // Drivetrain motors configuration
    private val frontLeftMotor = MotorEx("front_left_motor").brakeMode()
    private val frontRightMotor = MotorEx("front_right_motor").brakeMode().reversed()
    private val backLeftMotor = MotorEx("back_left_motor").brakeMode()
    private val backRightMotor = MotorEx("back_right_motor").brakeMode().reversed()

    /**
     * Sets up the driver-controlled movement and button bindings when the start button is pressed.
     */
    override fun onStartButtonPressed() {
        resetRuntime()

        // Initialize Mecanum drive control with gamepad1 sticks
        val driverControlled = MecanumDriverControlled(
            frontLeftMotor = frontLeftMotor,
            frontRightMotor = frontRightMotor,
            backLeftMotor = backLeftMotor,
            backRightMotor = backRightMotor,
            drivePower = Gamepads.gamepad1.leftStickY,
            strafePower = -Gamepads.gamepad1.leftStickX,
            turnPower = -Gamepads.gamepad1.rightStickX
        )
        driverControlled()

        // Intake controls: Right Bumper for reverse, Left Bumper for forward
        Gamepads.gamepad1.rightBumper whenBecomesTrue IntakeSubsystem.reverse whenBecomesFalse IntakeSubsystem.stop
        Gamepads.gamepad1.leftBumper whenBecomesTrue IntakeSubsystem.forward whenBecomesFalse IntakeSubsystem.stop

        // Kicker control: Right Trigger to kick, release to reset
        Gamepads.gamepad1.rightTrigger.atLeast(RIGHT_TRIGGER_MINIMUM_VALUE)
            .whenBecomesTrue(FlywheelShooterSubsystem.kickArtifact)
            .whenBecomesFalse(FlywheelShooterSubsystem.resetKickerServo)

        // Transfer control: Left Trigger to stop transfer
        Gamepads.gamepad1.leftTrigger.atLeast(RIGHT_TRIGGER_MINIMUM_VALUE)
            .whenBecomesTrue(FlywheelShooterSubsystem.stopTransfer)

        // Shooter RPM presets: Circle for front launch zone, Square for back launch zone
        Gamepads.gamepad1.circle.toggleOnBecomesTrue().whenBecomesTrue(
            FlywheelShooterSubsystem.startSpin(FLYWHEEL_MOTOR_RPM_FRONT_LAUNCH_ZONE)
        ) whenBecomesFalse(FlywheelShooterSubsystem.stopSpin)

        Gamepads.gamepad1.square.toggleOnBecomesTrue().whenBecomesTrue(
            FlywheelShooterSubsystem.startSpin(FLYWHEEL_MOTOR_RPM_BACK_LAUNCH_ZONE)
        ).whenBecomesFalse(FlywheelShooterSubsystem.stopSpin)
    }

    /**
     * Ensures the flywheel stops spinning when the OpMode is stopped.
     */
    override fun onStop() {
        FlywheelShooterSubsystem.stopSpin()
    }

    /**
     * Updates telemetry data on each loop iteration.
     */
    override fun onUpdate() {
        ActiveOpMode.telemetry.update()
    }
}
