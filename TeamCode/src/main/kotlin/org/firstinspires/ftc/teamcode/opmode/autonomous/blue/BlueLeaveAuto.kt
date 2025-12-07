package org.firstinspires.ftc.teamcode.opmode.autonomous.blue

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.RobotLog
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
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem

@Autonomous(name = "Blue Leave Auto", group = "Blue Alliance", preselectTeleOp = "Pedro TeleOp")
class BlueLeaveAuto : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(IntakeSubsystem),
            PedroComponent(Constants::createFollower),
            BulkReadComponent
        )


        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        AutonomousStateManager.isRedAlliance = false
    }

    override fun onInit() {
        PathManager.buildPaths(PedroComponent.follower)
        PedroComponent.follower.setStartingPose(PathManager.farLaunchZoneStartPose)

        Drawing.init()
    }

    override fun onStartButtonPressed() {
        AutonomousRoutines.farParkAutoRoutine()
    }

    override fun onStop() {
        super.onStop()
        AutonomousStateManager.startPoseAtEndOfAuto = PedroComponent.follower.pose
    }


    override fun onUpdate() {
        Drawing.drawDebug(PedroComponent.follower)
        ActiveOpMode.telemetry.addData("Current pose", PedroComponent.follower.pose)
        ActiveOpMode.telemetry.addData("isBusy", PedroComponent.follower.isBusy)

        RobotLog.d("Motor Voltage: Drivetrain: " + PedroComponent.follower.drivetrain.voltage.toString())

        ActiveOpMode.telemetry.update()
    }
}