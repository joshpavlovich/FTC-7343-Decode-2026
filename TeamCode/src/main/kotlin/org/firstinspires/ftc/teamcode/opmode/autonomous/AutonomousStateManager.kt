package org.firstinspires.ftc.teamcode.opmode.autonomous

import com.pedropathing.geometry.Pose

/**
 * AutonomousStateManager holds global state information for the autonomous period.
 * This includes the current alliance color and the robot's pose at the end of autonomous,
 * which can be used to initialize the robot's position for TeleOp.
 */
object AutonomousStateManager {

    /**
     * Whether the robot is currently on the Red alliance.
     * Set to true for Red alliance, false for Blue alliance.
     */
    var isRedAlliance: Boolean = false

    /**
     * The robot's expected pose at the end of the autonomous routine.
     * Defaults to the front launch zone start pose.
     */
    var startPoseAtEndOfAuto: Pose = PathManager.frontLaunchZoneStartPose
}
