package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutonomousStateManager
import org.firstinspires.ftc.teamcode.opmode.autonomous.PathManager.blueFrontShootingPose
import org.firstinspires.ftc.teamcode.opmode.autonomous.PathManager.blueGoalPose
import org.firstinspires.ftc.teamcode.opmode.autonomous.PathManager.endGameBaseZoneParkPose
import org.firstinspires.ftc.teamcode.opmode.autonomous.PathManager.blueBackShootingPose
import org.firstinspires.ftc.teamcode.panels.Drawing
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystem.ColorSensorSubsystem
import org.firstinspires.ftc.teamcode.subsystem.FLYWHEEL_MOTOR_POWER_BACK_LAUNCH_ZONE
import org.firstinspires.ftc.teamcode.subsystem.FLYWHEEL_MOTOR_POWER_FRONT_LAUNCH_ZONE
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem.addShooterDetails

private const val RIGHT_TRIGGER_MINIMUM_VALUE = 0.5

@TeleOp(name = "Pedro TeleOp")
class PedroTeleOp : NextFTCOpMode() {

    private val goalPose: Pose
        get() = if (AutonomousStateManager.isRedAlliance) {
            blueGoalPose.mirror()
        } else {
            blueGoalPose
        }

    init {
        addComponents(
            SubsystemComponent(FlywheelShooterSubsystem, ColorSensorSubsystem),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent
        )

        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
    }

    override fun onInit() {
        super.onInit()

        PedroComponent.follower.setStartingPose(AutonomousStateManager.startPoseAtEndOfAuto)
        PedroComponent.follower.update()

        Drawing.init()
    }

    override fun onStartButtonPressed() {
        val driverControlled = PedroDriverControlled(
            drivePower = -Gamepads.gamepad1.leftStickY,
            strafePower = -Gamepads.gamepad1.leftStickX,
            turnPower = -Gamepads.gamepad1.rightStickX,
            robotCentric = true // TODO: WE NEED TO TEST FIELD CENTRIC VS ROBOT CENTRIC, SEE https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric
        )
        driverControlled()

        Gamepads.gamepad1.rightTrigger.atLeast(RIGHT_TRIGGER_MINIMUM_VALUE)
            .whenBecomesTrue(FlywheelShooterSubsystem.kickArtifact())
            .whenBecomesFalse(FlywheelShooterSubsystem.resetKickerServo())

        Gamepads.gamepad1.leftTrigger.atLeast(RIGHT_TRIGGER_MINIMUM_VALUE)
            .whenBecomesTrue(FlywheelShooterSubsystem.stopTransfer())

        Gamepads.gamepad1.circle.toggleOnBecomesTrue().whenBecomesTrue(
            FlywheelShooterSubsystem.spin(FLYWHEEL_MOTOR_POWER_FRONT_LAUNCH_ZONE)
        ) whenBecomesFalse FlywheelShooterSubsystem.stopSpin()

        Gamepads.gamepad1.square.toggleOnBecomesTrue().whenBecomesTrue(
            FlywheelShooterSubsystem.spin(FLYWHEEL_MOTOR_POWER_BACK_LAUNCH_ZONE)
        ).whenBecomesFalse(FlywheelShooterSubsystem.stopSpin())

        // TODO: ONLY EXECUTE THIS IF IN END GAME?
        Gamepads.gamepad1.ps.whenTrue {
            followDynamicPath(endGameBaseZoneParkPose)
        }

        Gamepads.gamepad1.cross.whenTrue {
            followDynamicPath(blueFrontShootingPose)

        }
            Gamepads.gamepad1.triangle.whenTrue {
                followDynamicPath(blueBackShootingPose)

            }

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

    override fun onUpdate() {
        PedroComponent.follower.update()

        // TODO: TEST BREAK OUT OF PEDRO PATH FOLLOWING
        if (!PedroComponent.follower.teleopDrive && !PedroComponent.follower.isBusy) {
            PedroComponent.follower.startTeleOpDrive()
        }

        Drawing.drawDebug(PedroComponent.follower)
        ActiveOpMode.telemetry.addData("Current pose", PedroComponent.follower.pose)
        ActiveOpMode.telemetry.addData("isBusy", PedroComponent.follower.isBusy)

        ActiveOpMode.telemetry.addData("Goal pose", goalPose)
        ActiveOpMode.telemetry.addData(
            "Distance to Goal?", PedroComponent.follower.pose.distanceFrom(
                goalPose
            )
        )

        ActiveOpMode.telemetry.addShooterDetails()
        ActiveOpMode.telemetry.update()
    }
}