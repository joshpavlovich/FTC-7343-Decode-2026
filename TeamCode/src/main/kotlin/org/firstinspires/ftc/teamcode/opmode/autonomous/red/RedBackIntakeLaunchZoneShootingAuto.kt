package org.firstinspires.ftc.teamcode.opmode.autonomous.red

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutonomousRoutines
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutonomousStateManager
import org.firstinspires.ftc.teamcode.opmode.autonomous.PathManager
import org.firstinspires.ftc.teamcode.opmode.autonomous.PathManager.goalPose
import org.firstinspires.ftc.teamcode.opmode.teleop.PEDRO_TELE_OP
import org.firstinspires.ftc.teamcode.panels.Drawing
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystem.ColorSensorSubsystem
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem.calculateRpm
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem

/**
 * Autonomous OpMode for the Red Alliance that starts in the back launch zone,
 * performs shooting, and utilizes the intake to collect and shoot additional artifacts.
 * Currently disabled.
 */
@Disabled
@Autonomous(
    name = "\uD83D\uDFE6 Red Back Intake Launch Zone Shoot Auto",
    group = "Red Alliance",
    preselectTeleOp = PEDRO_TELE_OP
)
class RedBackIntakeLaunchZoneShootingAuto : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(FlywheelShooterSubsystem, ColorSensorSubsystem, IntakeSubsystem),
            PedroComponent(Constants::createFollower),
            BulkReadComponent
        )

        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        AutonomousStateManager.isRedAlliance = true
    }

    /**
     * Initializes paths and sets the starting pose for the back launch zone.
     */
    override fun onInit() {
        PathManager.buildPaths(PedroComponent.follower)
        PedroComponent.follower.setStartingPose(PathManager.backLaunchZoneStartPose)

        Drawing.init()
    }

    /**
     * Executes the back launch intake and shooting routine on start.
     */
    override fun onStartButtonPressed() {
        AutonomousRoutines.backLaunchIntakeShootingAutoRoutine()
    }

    /**
     * Stops the flywheel and records the final pose for TeleOp.
     */
    override fun onStop() {
        FlywheelShooterSubsystem.stopSpin()
        AutonomousStateManager.startPoseAtEndOfAuto = PedroComponent.follower.pose
    }

    /**
     * Continuously calculates the required RPM based on distance to the goal
     * and updates telemetry/debug visuals.
     */
    override fun onUpdate() {
        val distanceFrom = PedroComponent.follower.pose.distanceFrom(goalPose)
        val calculatedRpm = calculateRpm(distanceFrom)
        FlywheelShooterSubsystem.startSpin(calculatedRpm).schedule()

        Drawing.drawDebug(PedroComponent.follower)
        ActiveOpMode.telemetry.addData("Pedro Follower isBusy", PedroComponent.follower.isBusy)
        ActiveOpMode.telemetry.addData("Current pose", PedroComponent.follower.pose)
        ActiveOpMode.telemetry.addData("Distance to Goal?", distanceFrom)
        ActiveOpMode.telemetry.addData("Calculated RPM", calculatedRpm)

        ActiveOpMode.telemetry.update()
    }
}
