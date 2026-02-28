package org.firstinspires.ftc.teamcode.opmode.autonomous.blue

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
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

/**
 * Autonomous OpMode for the Blue Alliance that starts in the front launch zone,
 * strafes to the back wall shooting position, and launches artifacts.
 */
@Autonomous(
    name = "\uD83D\uDFE6 Blue Front Launch Zone Wall Shoot Auto",
    group = "Blue Alliance",
    preselectTeleOp = PEDRO_TELE_OP
)
class BlueFrontLaunchZoneToBackWallShootingAuto : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(FlywheelShooterSubsystem, ColorSensorSubsystem),
            PedroComponent(Constants::createFollower),
            BulkReadComponent
        )

        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        AutonomousStateManager.isRedAlliance = false
    }

    /**
     * Initializes paths and sets the starting pose for the front launch zone strafe start.
     */
    override fun onInit() {
        PathManager.buildPaths(PedroComponent.follower)
        PedroComponent.follower.setStartingPose(PathManager.frontLaunchZoneStrafeStartPose)

        ColorSensorSubsystem.isOpModeTeleOp = false

        Drawing.init()
    }

    /**
     * Executes the front launch strafe to back wall shooting routine on start.
     */
    override fun onStartButtonPressed() {
        AutonomousRoutines.frontLaunchZoneStrafeStartWallShootingAutoRoutine()
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
