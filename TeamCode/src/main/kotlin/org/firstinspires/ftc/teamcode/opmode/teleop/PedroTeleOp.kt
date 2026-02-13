package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.extensions.pedro.TurnBy
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutonomousStateManager
import org.firstinspires.ftc.teamcode.opmode.autonomous.PathManager.blueBackShootingPose
import org.firstinspires.ftc.teamcode.opmode.autonomous.PathManager.blueBackWallShootingPose
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
import kotlin.math.atan2

private const val LAYER_ENDGAME = "endgame"
private const val RIGHT_TRIGGER_MINIMUM_VALUE = 0.5

const val PEDRO_TELE_OP = "\uD83D\uDC25 Pedro TeleOp"

@Configurable
@TeleOp(name = PEDRO_TELE_OP)
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

        Gamepads.gamepad1.rightBumper whenBecomesTrue IntakeSubsystem.reverse whenBecomesFalse IntakeSubsystem.stop
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

        Gamepads.gamepad1.dpadDown.whenTrue {
            followDynamicPath(blueBackWallShootingPose)
        }

        Gamepads.gamepad1.square.whenTrue {
            followDynamicPath(blueGoalGatePose)
        }

// TODO: TEST BREAK OUT OF PEDRO PATH FOLLOWING TURN TO GOAL WITH 1 SECOND DELAY
//        Gamepads.gamepad1.dpadDown.whenTrue {
//            turnToGoal()
//        }
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

            FollowPath(path, true).schedule()
        }
    }

    /**
     * Calculates the relative bearing to any target goal.
     * @param robotPose The current pose of the robot (from follower.pose)
     * @param goalPose The static pose of the target goal
     * @return The relative turn needed in DEGREES [-180, 180]
     */
    fun getRelativeBearing(robotPose: Pose, goalPose: Pose): Double {
        // 1. Calculate the absolute field angle from robot to goal
        val deltaX = goalPose.x - robotPose.x
        val deltaY = goalPose.y - robotPose.y
        val fieldTargetAngleRad = atan2(deltaY, deltaX)

        // 2. Convert angles to degrees for easier logic/normalization
        val robotHeadingDeg = Math.toDegrees(robotPose.heading)
        val fieldTargetAngleDeg = Math.toDegrees(fieldTargetAngleRad)

        // 3. Calculate relative turn
        var relativeTurn = fieldTargetAngleDeg - robotHeadingDeg

        // 4. Normalize to the shortest path
        while (relativeTurn > 180) relativeTurn -= 360.0
        while (relativeTurn <= -180) relativeTurn += 360.0

        return relativeTurn
    }

    /**
     * Rotates the robot to face the target goal.
     *
     * This method calculates the required relative bearing from the current robot pose
     * to the [goalPose] and initiates a turn using [TurnBy] if the robot is not already
     * in the middle of a turn.
     */
    private fun turnToGoal() {
        ActiveOpMode.telemetry.addData("turnToGoal", PedroComponent.follower.isBusy)
        if (!PedroComponent.follower.isBusy) {
            val relativeBearing = getRelativeBearing(
                robotPose = PedroComponent.follower.pose,
                goalPose = goalPose
            )

            ActiveOpMode.telemetry.addData("relativeBearing", relativeBearing)

            SequentialGroup(
                TurnBy(Angle.fromDeg(relativeBearing)),
                InstantCommand({ PedroComponent.follower.startTeleOpDrive() }).afterTime(1.0)
            ).schedule()
        }
    }
}
