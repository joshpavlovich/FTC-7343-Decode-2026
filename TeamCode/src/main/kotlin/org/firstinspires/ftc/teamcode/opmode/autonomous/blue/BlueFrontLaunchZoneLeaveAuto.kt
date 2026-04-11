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
import org.firstinspires.ftc.teamcode.opmode.teleop.PEDRO_TELE_OP
import org.firstinspires.ftc.teamcode.panels.Drawing
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystem.ColorSensorSubsystem
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem

/**
 * Autonomous OpMode for the Blue Alliance that starts in the front launch zone,
 * leaves the zone to score points, and parks.
 */
@Autonomous(
    name = "\uD83D\uDFE6 Blue Leave Auto",
    group = "Blue Alliance",
    preselectTeleOp = PEDRO_TELE_OP
)
class BlueFrontLaunchZoneLeaveAuto : NextFTCOpMode() {

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
     * Initializes the paths and sets the starting pose for the robot.
     */
    override fun onInit() {
        PathManager.buildPaths(PedroComponent.follower)
        PedroComponent.follower.setStartingPose(PathManager.frontLaunchZoneStartPose)

        Drawing.init()
    }

    /**
     * Starts the leave and park autonomous routine when the start button is pressed.
     */
    override fun onStartButtonPressed() {
        AutonomousRoutines.frontLaunchZoneLeaveParkAutoRoutine()
    }

    /**
     * Cleans up subsystems and saves the final robot pose for TeleOp when the OpMode stops.
     */
    override fun onStop() {
        FlywheelShooterSubsystem.stopSpin()
        AutonomousStateManager.startPoseAtEndOfAuto = PedroComponent.follower.pose
    }

    /**
     * Updates telemetry and debug drawings during the OpMode execution.
     */
    override fun onUpdate() {
        Drawing.drawDebug(PedroComponent.follower)
        ActiveOpMode.telemetry.addData("Current pose", PedroComponent.follower.pose)
        ActiveOpMode.telemetry.addData("isBusy", PedroComponent.follower.isBusy)

        ActiveOpMode.telemetry.update()
    }
}
