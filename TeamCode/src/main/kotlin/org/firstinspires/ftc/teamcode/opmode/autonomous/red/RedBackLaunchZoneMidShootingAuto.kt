package org.firstinspires.ftc.teamcode.opmode.autonomous.red

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
import org.firstinspires.ftc.teamcode.panels.Drawing
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem

@Autonomous(
    name = "Red Back LaunchZone Shooting Auto",
    group = "Red Alliance",
preselectTeleOp = "Manual TeleOp"
)
class RedBackLaunchZoneMidShootingAuto : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(FlywheelShooterSubsystem),
            PedroComponent(Constants::createFollower),
            BulkReadComponent
        )


        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        AutonomousStateManager.isRedAlliance = true
    }

    override fun onInit() {
        PathManager.buildPaths(PedroComponent.follower)
        PedroComponent.follower.setStartingPose(PathManager.backLaunchZoneStartPose.mirror())

        Drawing.init()
    }

    override fun onStartButtonPressed() {
        AutonomousRoutines.backLaunchMidShootingAutoRoutine()
    }

    override fun onStop() {
        super.onStop()
        AutonomousStateManager.startPoseAtEndOfAuto = PedroComponent.follower.pose
    }


    override fun onUpdate() {
        Drawing.drawDebug(PedroComponent.follower)
        ActiveOpMode.telemetry.addData("Current pose", PedroComponent.follower.pose)
        ActiveOpMode.telemetry.addData("isBusy", PedroComponent.follower.isBusy)

        ActiveOpMode.telemetry.update()
    }
}