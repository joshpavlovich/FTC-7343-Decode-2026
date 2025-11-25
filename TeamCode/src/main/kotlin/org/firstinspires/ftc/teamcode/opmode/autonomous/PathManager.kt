package org.firstinspires.ftc.teamcode.opmode.autonomous

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.units.deg

object PathManager {

    // ALL POSES ARE ON BLUE SCORING/GOAL SIDE OF FIELD!!!

    val farLaunchZoneStartPose = Pose(56.0, 8.0, 90.deg.inRad)

    val farLeaveEndPose = Pose(36.0, 8.0, 90.deg.inRad)

    lateinit var farLaunchZoneStartToPark: PathChain

    fun buildPaths(follower: Follower) {
        if (AutonomousStateManager.isRedAlliance) {
            farLaunchZoneStartToPark = follower.pathBuilder()
                .addPath(BezierLine(farLaunchZoneStartPose.mirror(), farLeaveEndPose.mirror()))
                .setLinearHeadingInterpolation(farLaunchZoneStartPose.mirror().heading, farLeaveEndPose.mirror().heading)
                .build()
        } else {
            farLaunchZoneStartToPark = follower.pathBuilder()
                .addPath(BezierLine(farLaunchZoneStartPose, farLeaveEndPose))
                .setLinearHeadingInterpolation(farLaunchZoneStartPose.heading, farLeaveEndPose.heading)
                .build()
        }
    }
}