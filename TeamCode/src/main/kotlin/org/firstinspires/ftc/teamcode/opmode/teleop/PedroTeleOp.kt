package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.bindings.BindingManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutonomousStateManager
import org.firstinspires.ftc.teamcode.opmode.autonomous.PathManager.blueBackShootingPose
import org.firstinspires.ftc.teamcode.opmode.autonomous.PathManager.blueFrontShootingPose
import org.firstinspires.ftc.teamcode.opmode.autonomous.PathManager.blueGoalGatePose
import org.firstinspires.ftc.teamcode.opmode.autonomous.PathManager.endGameBaseZoneParkPose
import org.firstinspires.ftc.teamcode.opmode.autonomous.PathManager.goalPose
import org.firstinspires.ftc.teamcode.panels.Drawing
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystem.ColorSensorSubsystem
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem.calculateRpm
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem

private const val LAYER_ENDGAME = "endgame"
private const val RIGHT_TRIGGER_MINIMUM_VALUE = 0.5

@Configurable
@TeleOp(name = "Pedro TeleOp")
class PedroTeleOp : NextFTCOpMode() {

    companion object {
        @JvmField
        var configurableRpm: Double = 2000.0

        @JvmField
        var useConfigurableRpm = false
    }

    init {
        addComponents(
            SubsystemComponent(FlywheelShooterSubsystem, ColorSensorSubsystem, IntakeSubsystem),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent
        )

        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
    }

    override fun onInit() {
        super.onInit()

        PedroComponent.follower.setStartingPose(AutonomousStateManager.startPoseAtEndOfAuto)

        Drawing.init()
    }

    override fun onStartButtonPressed() {
        resetRuntime()

        val driverControlled = PedroDriverControlled(
            drivePower = -Gamepads.gamepad1.leftStickY,
            strafePower = -Gamepads.gamepad1.leftStickX,
            turnPower = -Gamepads.gamepad1.rightStickX
        )
        driverControlled()

        Gamepads.gamepad1.rightTrigger.atLeast(RIGHT_TRIGGER_MINIMUM_VALUE)
            .whenBecomesTrue(FlywheelShooterSubsystem.kickArtifact)
            .whenBecomesFalse(FlywheelShooterSubsystem.resetKickerServo)

        Gamepads.gamepad1.leftTrigger.atLeast(RIGHT_TRIGGER_MINIMUM_VALUE)
            .whenBecomesTrue { driverControlled.scalar = 0.1 }
            .whenBecomesFalse { driverControlled.scalar = 1.0 }
//            .whenBecomesTrue(FlywheelShooterSubsystem.stopTransfer)
//            .inLayer(LAYER_ENDGAME) {
//            }

        Gamepads.gamepad1.rightBumper whenBecomesTrue IntakeSubsystem.reverse  whenBecomesFalse IntakeSubsystem.stop
        Gamepads.gamepad1.leftBumper whenBecomesTrue IntakeSubsystem.forward whenBecomesFalse IntakeSubsystem.stop

        // TODO: DO WE NEED AN END GAME LAYER???
        Gamepads.gamepad1.ps.whenTrue {
            followDynamicPath(endGameBaseZoneParkPose)
        }

        Gamepads.gamepad1.cross.whenTrue {
            followDynamicPath(blueFrontShootingPose)
        }

        Gamepads.gamepad1.triangle.whenTrue {
            followDynamicPath(blueBackShootingPose)
        }

        Gamepads.gamepad1.square.whenTrue {
            followDynamicPath(blueGoalGatePose)
        }
    }

    override fun onStop() {
        FlywheelShooterSubsystem.stopSpin()
    }

    override fun onUpdate() {
//        if (ActiveOpMode.opModeIsActive && ActiveOpMode.runtime > END_GAME_START_TIME_SECONDS) {
//            BindingManager.layer = LAYER_ENDGAME
//        }

        val distanceFrom = PedroComponent.follower.pose.distanceFrom(goalPose)
        val calculatedRpm = calculateRpm(distanceFrom)
        val targetRpm = if (useConfigurableRpm) configurableRpm else calculatedRpm
        FlywheelShooterSubsystem.startSpin(targetRpm).schedule()

        // TODO: TEST BREAK OUT OF PEDRO PATH FOLLOWING
        if (!PedroComponent.follower.teleopDrive && !PedroComponent.follower.isBusy) {
            PedroComponent.follower.startTeleOpDrive()
        }

        Drawing.drawDebug(PedroComponent.follower)
        ActiveOpMode.telemetry.addData("Pedro Follower isBusy", PedroComponent.follower.isBusy)
        ActiveOpMode.telemetry.addData("Current pose", PedroComponent.follower.pose)
        ActiveOpMode.telemetry.addData("Distance to Goal?", distanceFrom)
        ActiveOpMode.telemetry.addData("Calculated RPM", calculatedRpm)
        ActiveOpMode.telemetry.addData("Target RPM", targetRpm)
        ActiveOpMode.telemetry.update()
    }

    private fun followDynamicPath(pose: Pose) {
        if (!PedroComponent.follower.isBusy) {
            val currentPose = PedroComponent.follower.pose
            val path = if (AutonomousStateManager.isRedAlliance) {
                PedroComponent.follower.pathBuilder()
                    .addPath(BezierLine(currentPose, pose.mirror()))
                    .setLinearHeadingInterpolation(
                        currentPose.heading,
                        pose.mirror().heading
                    )
                    .build()
            } else {
                PedroComponent.follower.pathBuilder()
                    .addPath(BezierLine(currentPose, pose))
                    .setLinearHeadingInterpolation(
                        currentPose.heading,
                        pose.heading
                    )
                    .build()
            }

            PedroComponent.follower.followPath(path, true)
        }
    }
}