package org.firstinspires.ftc.teamcode.opmode.autonomous

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.units.deg

object PathManager {

    // ALL POSES ARE ON BLUE ALLIANCE SIDE OF FIELD
    val frontLaunchZoneStartPose = Pose(56.0, 8.0, 90.deg.inRad)
    val frontLaunchZoneShootingPose = Pose(56.0, 21.258, 112.deg.inRad)
    val frontLaunchZoneLeaveParkPose = Pose(36.0, 8.0, 90.deg.inRad)
    val backLaunchZoneStartPose = Pose(33.5, 134.5, 90.deg.inRad)
    val backLaunchZoneShootingPose = Pose(56.0, 86.0, 135.deg.inRad)
    val backLaunchZoneShootingControlPointPose = Pose(65.0, 122.0)
    val backLaunchZoneParkPose = Pose(42.0, 72.0, 90.deg.inRad)

    // TELEOP POSES
    val endGameBaseZoneParkPose = Pose(105.25, 33.25, 90.deg.inRad)
    val blueFrontShootingPose = Pose(92.0, 13.0, 125.deg.inRad)

    // FIELD LOCATION POSES
    val blueGoalPose = Pose(16.3, 131.8)

    lateinit var frontLaunchZoneStartToPark: PathChain
    lateinit var frontLaunchZoneStartToFrontLaunchZoneShooting: PathChain
    lateinit var frontLaunchZoneShootingToFrontLaunchZoneLeavePark: PathChain
    lateinit var backLaunchZoneStartToBackLaunchZoneShooting: PathChain
    lateinit var backLaunchZoneShootingToBackLaunchZoneLeavePark: PathChain

    fun buildPaths(follower: Follower) {
        if (AutonomousStateManager.isRedAlliance) {
            frontLaunchZoneStartToPark = follower.pathBuilder()
                .addPath(
                    BezierLine(
                        frontLaunchZoneStartPose.mirror(),
                        frontLaunchZoneLeaveParkPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    frontLaunchZoneStartPose.mirror().heading,
                    frontLaunchZoneLeaveParkPose.mirror().heading
                )
                .build()

            frontLaunchZoneStartToFrontLaunchZoneShooting = follower.pathBuilder()
                .addPath(
                    BezierLine(
                        frontLaunchZoneStartPose.mirror(),
                        frontLaunchZoneShootingPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    frontLaunchZoneStartPose.mirror().heading,
                    frontLaunchZoneShootingPose.mirror().heading
                )
                .build()

            frontLaunchZoneShootingToFrontLaunchZoneLeavePark = follower.pathBuilder()
                .addPath(
                    BezierLine(
                        frontLaunchZoneShootingPose.mirror(),
                        frontLaunchZoneLeaveParkPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    frontLaunchZoneShootingPose.mirror().heading,
                    frontLaunchZoneLeaveParkPose.mirror().heading
                )
                .build()

            backLaunchZoneStartToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        backLaunchZoneStartPose.mirror(),
                        backLaunchZoneShootingControlPointPose.mirror(),
                        backLaunchZoneShootingPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    backLaunchZoneStartPose.mirror().heading,
                    backLaunchZoneShootingPose.mirror().heading
                )
                .build()

            backLaunchZoneShootingToBackLaunchZoneLeavePark = follower.pathBuilder()
                .addPath(
                    BezierLine(
                        backLaunchZoneShootingPose.mirror(),
                        backLaunchZoneParkPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    backLaunchZoneShootingPose.mirror().heading,
                    backLaunchZoneParkPose.mirror().heading
                )
                .build()
        } else {
            frontLaunchZoneStartToPark = follower.pathBuilder()
                .addPath(BezierLine(frontLaunchZoneStartPose, frontLaunchZoneLeaveParkPose))
                .setLinearHeadingInterpolation(
                    frontLaunchZoneStartPose.heading,
                    frontLaunchZoneLeaveParkPose.heading
                )
                .build()

            frontLaunchZoneStartToFrontLaunchZoneShooting = follower.pathBuilder()
                .addPath(BezierLine(frontLaunchZoneStartPose, frontLaunchZoneShootingPose))
                .setLinearHeadingInterpolation(
                    frontLaunchZoneStartPose.heading,
                    frontLaunchZoneShootingPose.heading
                )
                .build()

            frontLaunchZoneShootingToFrontLaunchZoneLeavePark = follower.pathBuilder()
                .addPath(BezierLine(frontLaunchZoneShootingPose, frontLaunchZoneLeaveParkPose))
                .setLinearHeadingInterpolation(
                    frontLaunchZoneShootingPose.heading,
                    frontLaunchZoneLeaveParkPose.heading
                )
                .build()

            backLaunchZoneStartToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        backLaunchZoneStartPose,
                        backLaunchZoneShootingControlPointPose,
                        backLaunchZoneShootingPose
                    )
                )
                .setLinearHeadingInterpolation(
                    backLaunchZoneStartPose.heading,
                    backLaunchZoneShootingPose.heading
                )
                .build()

            backLaunchZoneShootingToBackLaunchZoneLeavePark = follower.pathBuilder()
                .addPath(BezierLine(backLaunchZoneShootingPose, backLaunchZoneParkPose))
                .setLinearHeadingInterpolation(
                    backLaunchZoneShootingPose.heading,
                    backLaunchZoneParkPose.heading
                )
                .build()
        }
    }
}