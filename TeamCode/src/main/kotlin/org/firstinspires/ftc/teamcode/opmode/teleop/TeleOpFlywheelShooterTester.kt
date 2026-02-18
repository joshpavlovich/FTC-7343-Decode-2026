package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystem.FLYWHEEL_MOTOR_RPM_BACK_LAUNCH_ZONE
import org.firstinspires.ftc.teamcode.subsystem.FLYWHEEL_MOTOR_RPM_FRONT_LAUNCH_ZONE
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem

private const val RIGHT_TRIGGER_MINIMUM_VALUE = 0.3

/**
 * A testing OpMode for the FlywheelShooterSubsystem.
 * Provides manual control over the flywheel velocity and kicker servo for verification and tuning.
 */
@Disabled
@TeleOp(name = "TeleOp Shooter Tester")
class TeleOpFlywheelShooterTester : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(FlywheelShooterSubsystem),
            BulkReadComponent,
            BindingsComponent
        )

        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
    }

    /**
     * Binds gamepad buttons to shooter actions:
     * - Right Trigger: Activates the kicker mechanism.
     * - Circle: Toggles the flywheel to the front launch zone RPM preset.
     * - Square: Toggles the flywheel to the back launch zone RPM preset.
     */
    override fun onStartButtonPressed() {
        Gamepads.gamepad1.rightTrigger.atLeast(RIGHT_TRIGGER_MINIMUM_VALUE)
            .whenBecomesTrue(FlywheelShooterSubsystem.kickArtifact)
            .whenBecomesFalse(FlywheelShooterSubsystem.resetKickerServo)
        Gamepads.gamepad1.circle.toggleOnBecomesTrue().whenBecomesTrue(
            FlywheelShooterSubsystem.startSpin(FLYWHEEL_MOTOR_RPM_FRONT_LAUNCH_ZONE)
        ).whenBecomesFalse(FlywheelShooterSubsystem.stopSpin)
        Gamepads.gamepad1.square.toggleOnBecomesTrue().whenBecomesTrue(
            FlywheelShooterSubsystem.startSpin(FLYWHEEL_MOTOR_RPM_BACK_LAUNCH_ZONE)
        ).whenBecomesFalse(FlywheelShooterSubsystem.stopSpin)
    }

    /**
     * Updates telemetry data on each loop iteration to monitor the shooter's performance.
     */
    override fun onUpdate() {
        ActiveOpMode.telemetry.update()
    }
}
