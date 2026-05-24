package org.firstinspires.ftc.teamcode.opmode.teleop

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.extensions.pedro.TurnBy
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opmode.autonomous.PathManager
import org.firstinspires.ftc.teamcode.opmode.autonomous.PathManager.goalPose
import org.firstinspires.ftc.teamcode.panels.Drawing
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystem.LimelightSubsystem
import kotlin.math.absoluteValue
import kotlin.math.atan2

/**
 * A testing OpMode for the IntakeSubsystem.
 * Allows for manual verification of intake forward and reverse functionality.
 */
@TeleOp(name = "TeleOp Limelight Subsytem Tester")
class TeleOpLimelightTester : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(LimelightSubsystem),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent
        )
    }

    /**
     * Initializes the robot's pose based on the state saved at the end of the autonomous period.
     */
    override fun onInit() {
        super.onInit()

        PedroComponent.follower.setStartingPose(PathManager.frontLaunchZoneStartPose)

        Drawing.init()
    }

    /**
     * Sets up the driver-controlled movement and button bindings when the start button is pressed.
     */
    override fun onStartButtonPressed() {
        resetRuntime()

        // Primary driver control for movement
        val driverControlled = PedroDriverControlled(
            drivePower = -Gamepads.gamepad1.leftStickY,
            strafePower = -Gamepads.gamepad1.leftStickX,
            turnPower = -Gamepads.gamepad1.rightStickX
        )
        driverControlled()

        Gamepads.gamepad1.circle.whenBecomesTrue { turnToGoal() }
    }

    /**
     * Updates telemetry data on each loop iteration.
     */
    override fun onUpdate() {
        val relativeBearing = getRelativeBearing(
            robotPose = PedroComponent.follower.pose,
            goalPose = goalPose
        )
        ActiveOpMode.telemetry.addData("Pedro relativeBearing offset", relativeBearing)

        ActiveOpMode.telemetry.update()
    }

    /**
     * Calculates the relative bearing (in degrees) from the robot's current pose to a target goal pose.
     *
     * @param robotPose The current pose of the robot.
     * @param goalPose The static pose of the target goal.
     * @return The relative turn needed in degrees within the range [-180, 180].
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
     * to the [goalPose] and initiates a turn using [TurnBy].
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
                InstantCommand { PedroComponent.follower.startTeleOpDrive() }
            ).schedule()
        }
    }

    private fun autoAimCommand(): Command = InstantCommand({
        val follower = PedroComponent.follower

        val relativeBearing = getRelativeBearing(
            robotPose = PedroComponent.follower.pose,
            goalPose = goalPose
        )
        ActiveOpMode.telemetry.addData("relativeBearing offset", relativeBearing)
        ActiveOpMode.telemetry.addData(
            "currentHeading + relativeBearing offset",
            Math.toDegrees(follower.pose.heading + Math.toRadians(relativeBearing))
        )

        val robotPose = follower.pose
        val targetAngle = atan2(goalPose.y - robotPose.y, goalPose.x - robotPose.x)
        ActiveOpMode.telemetry.addData("targetAngle", Math.toDegrees(targetAngle))

        if (LimelightSubsystem.hasTarget) {
            // Visual Lock
            val currentHeading = follower.pose.heading
            ActiveOpMode.telemetry.addData("currentHeading", currentHeading)
            ActiveOpMode.telemetry.addData(
                "currentHeading + horizontalOffset",
                Math.toDegrees(currentHeading + Math.toRadians(LimelightSubsystem.horizontalOffset.absoluteValue))
            )
            follower.heading =
                currentHeading + Math.toRadians(LimelightSubsystem.horizontalOffset.absoluteValue)
        } else {
            // Odometry Fallback
            val robotPose = follower.pose
            val targetAngle = atan2(goalPose.y - robotPose.y, goalPose.x - robotPose.x)
            ActiveOpMode.telemetry.addData("targetAngle", targetAngle)
            follower.heading = targetAngle
        }
    }).addRequirements(LimelightSubsystem)
}
