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
    val frontLaunchZoneShootingPose = Pose(58.47, 12.855, 114.0.deg.inRad)
    val frontLaunchZoneLeaveParkPose = Pose(36.0, 8.0, 90.deg.inRad)
    val backLaunchZoneStartPose = Pose(33.5, 134.5, 90.deg.inRad)
    val backIntakeLaunchZoneShootingPose = Pose(36.5, 106.8, 135.deg.inRad)
    val backIntakeLaunchZonePreGppSpikeMarkPose = Pose(42.0, 84.21, 180.deg.inRad)
    val backIntakeLaunchZonePreGppSpikeMarkControlPose = Pose(48.3, 95.79)
    val backIntakeLaunchZoneGppSpikeMarkPose = Pose(17.37, 83.90, 180.deg.inRad)
    val backIntakeLaunchZonePrePgpSpikeMarkPose = Pose(42.0, 59.63, 180.deg.inRad)
    val backIntakeLaunchZonePgpSpikeMarkPose = Pose(17.37, 83.90, 180.deg.inRad)
    val backIntakeLaunchZonePrePgpSpikeMarkControlPose = Pose(48.3, 64.6)
    val frontLaunchZonePrePpgSpikeMarkPose = Pose(42.0, 35.5, 180.deg.inRad)
    val frontLaunchZonePpgSpikeMarkPose = Pose(18.0, 35.5, 180.deg.inRad)
    val frontLaunchZonePrePpgSpikeMarkControlPose = Pose(55.4, 28.8)
    val backLaunchZoneShootingPose = Pose(56.0, 86.0, 135.deg.inRad)
    val backLaunchZoneShootingControlPointPose = Pose(65.0, 122.0)
    val backLaunchZoneParkPose = Pose(42.0, 72.0, 90.deg.inRad)

    // TELEOP POSES
    //Goes to Parking Square
    val endGameBaseZoneParkPose = Pose(105.25, 33.25, 90.deg.inRad)
    //Goes to closer shooting zone
    val blueBackShootingPose = Pose(62.0, 82.0, 130.deg.inRad)
    //Goes to far shooing zone
    val blueFrontShootingPose = Pose(81.0, 21.0, 123.deg.inRad)
    //Goes to gate
    val blueGoalGatePose = Pose(30.0, 67.0, 90.deg.inRad)

    // FIELD LOCATION POSES
    val blueGoalPose = Pose(9.0, 134.7, 110.0.deg.inRad)

    val goalPose: Pose
        get() = if (AutonomousStateManager.isRedAlliance) {
            blueGoalPose.mirror()
        } else {
            blueGoalPose
        }

    lateinit var frontLaunchZoneStartToPark: PathChain
    lateinit var frontLaunchZoneStartToFrontLaunchZoneShooting: PathChain
    lateinit var frontLaunchZoneShootingToFrontLaunchZoneLeavePark: PathChain
    lateinit var backLaunchZoneStartToBackLaunchZoneShooting: PathChain
    lateinit var backLaunchZoneStartToBackIntakeLaunchZoneShooting: PathChain
    lateinit var backIntakeLaunchZoneShootingToGppPreSpikeMark: PathChain
    lateinit var backIntakeLaunchZoneGppPreSpikeMarkToGppSpikeMark: PathChain
    lateinit var backIntakeLaunchZoneGppSpikeMarkToBackLaunchZoneShooting: PathChain
    lateinit var backIntakeLaunchZoneShootingToPgpPreSpikeMark: PathChain
    lateinit var backIntakeLaunchZonePgpPreSpikeMarkToPgpSpikeMark: PathChain
    lateinit var backIntakeLaunchZonePgpSpikeMarkToBackLaunchZoneShooting: PathChain
    lateinit var backIntakeLaunchZoneShootingToBackLaunchZoneLeavePark: PathChain
    lateinit var backLaunchZoneShootingToBackLaunchZoneLeavePark: PathChain

    lateinit var frontLaunchZoneShootingToPpgPreSpikeMark: PathChain
    lateinit var frontLaunchZonePpgPreSpikeMarkToPpgSpikeMark: PathChain
    lateinit var frontLaunchZonePpgSpikeMarkToFrontLaunchZoneShooting: PathChain
    lateinit var frontLaunchZoneShootingToPgpPreSpikeMark: PathChain

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

            backLaunchZoneStartToBackIntakeLaunchZoneShooting = follower.pathBuilder()
                .addPath(BezierLine(backLaunchZoneStartPose.mirror(), backIntakeLaunchZoneShootingPose.mirror()))
                .setLinearHeadingInterpolation(
                    backLaunchZoneStartPose.mirror().heading,
                    backIntakeLaunchZoneShootingPose.mirror().heading
                )
                .build()

            backIntakeLaunchZoneShootingToGppPreSpikeMark = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        backIntakeLaunchZoneShootingPose.mirror(),
                        backIntakeLaunchZonePreGppSpikeMarkControlPose.mirror(),
                        backIntakeLaunchZonePreGppSpikeMarkPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZoneShootingPose.mirror().heading,
                    backIntakeLaunchZonePreGppSpikeMarkPose.mirror().heading
                )
                .build()

            backIntakeLaunchZoneGppPreSpikeMarkToGppSpikeMark = follower.pathBuilder()
                .addPath(BezierLine(backIntakeLaunchZonePreGppSpikeMarkPose.mirror(), backIntakeLaunchZoneGppSpikeMarkPose.mirror()))
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZonePreGppSpikeMarkPose.mirror().heading,
                    backIntakeLaunchZoneGppSpikeMarkPose.mirror().heading
                )
                .build()

            backIntakeLaunchZoneGppSpikeMarkToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(BezierLine(backIntakeLaunchZoneGppSpikeMarkPose.mirror(), backIntakeLaunchZoneShootingPose.mirror()))
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZoneGppSpikeMarkPose.mirror().heading,
                    backIntakeLaunchZoneShootingPose.mirror().heading
                )
                .build()

            backIntakeLaunchZoneShootingToPgpPreSpikeMark = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        backIntakeLaunchZoneShootingPose.mirror(),
                        backIntakeLaunchZonePrePgpSpikeMarkControlPose.mirror(),
                        backIntakeLaunchZonePrePgpSpikeMarkPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZoneShootingPose.mirror().heading,
                    backIntakeLaunchZonePrePgpSpikeMarkPose.mirror().heading
                )
                .build()

            backIntakeLaunchZonePgpPreSpikeMarkToPgpSpikeMark = follower.pathBuilder()
                .addPath(BezierLine(backIntakeLaunchZonePrePgpSpikeMarkPose.mirror(), backIntakeLaunchZonePgpSpikeMarkPose.mirror()))
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZonePrePgpSpikeMarkPose.mirror().heading,
                    backIntakeLaunchZonePgpSpikeMarkPose.mirror().heading
                )
                .build()

            backIntakeLaunchZonePgpSpikeMarkToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(BezierLine(backIntakeLaunchZonePgpSpikeMarkPose.mirror(), backIntakeLaunchZoneShootingPose.mirror()))
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZonePgpSpikeMarkPose.mirror().heading,
                    backIntakeLaunchZoneShootingPose.mirror().heading
                )
                .build()

            backIntakeLaunchZoneShootingToBackLaunchZoneLeavePark = follower.pathBuilder()
                .addPath(BezierLine(backIntakeLaunchZoneShootingPose.mirror(), backLaunchZoneParkPose.mirror()))
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZoneShootingPose.mirror().heading,
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

            backLaunchZoneStartToBackIntakeLaunchZoneShooting = follower.pathBuilder()
                .addPath(BezierLine(backLaunchZoneStartPose, backIntakeLaunchZoneShootingPose))
                .setLinearHeadingInterpolation(
                    backLaunchZoneStartPose.heading,
                    backIntakeLaunchZoneShootingPose.heading
                )
                .build()

            backIntakeLaunchZoneShootingToGppPreSpikeMark = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        backIntakeLaunchZoneShootingPose,
                        backIntakeLaunchZonePreGppSpikeMarkControlPose,
                        backIntakeLaunchZonePreGppSpikeMarkPose
                    )
                )
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZoneShootingPose.heading,
                    backIntakeLaunchZonePreGppSpikeMarkPose.heading
                )
                .build()

            backIntakeLaunchZoneGppPreSpikeMarkToGppSpikeMark = follower.pathBuilder()
                .addPath(BezierLine(backIntakeLaunchZonePreGppSpikeMarkPose, backIntakeLaunchZoneGppSpikeMarkPose))
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZonePreGppSpikeMarkPose.heading,
                    backIntakeLaunchZoneGppSpikeMarkPose.heading
                )
                .build()

            backIntakeLaunchZoneGppSpikeMarkToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(BezierLine(backIntakeLaunchZoneGppSpikeMarkPose, backIntakeLaunchZoneShootingPose))
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZoneGppSpikeMarkPose.heading,
                    backIntakeLaunchZoneShootingPose.heading
                )
                .build()

            backIntakeLaunchZoneShootingToPgpPreSpikeMark = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        backIntakeLaunchZoneShootingPose,
                        backIntakeLaunchZonePrePgpSpikeMarkControlPose,
                        backIntakeLaunchZonePrePgpSpikeMarkPose
                    )
                )
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZoneShootingPose.heading,
                    backIntakeLaunchZonePrePgpSpikeMarkPose.heading
                )
                .build()

            backIntakeLaunchZonePgpPreSpikeMarkToPgpSpikeMark = follower.pathBuilder()
                .addPath(BezierLine(backIntakeLaunchZonePrePgpSpikeMarkPose, backIntakeLaunchZonePgpSpikeMarkPose))
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZonePrePgpSpikeMarkPose.heading,
                    backIntakeLaunchZonePgpSpikeMarkPose.heading
                )
                .build()

            backIntakeLaunchZonePgpSpikeMarkToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(BezierLine(backIntakeLaunchZonePgpSpikeMarkPose, backIntakeLaunchZoneShootingPose))
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZonePgpSpikeMarkPose.heading,
                    backIntakeLaunchZoneShootingPose.heading
                )
                .build()

            backIntakeLaunchZoneShootingToBackLaunchZoneLeavePark = follower.pathBuilder()
                .addPath(BezierLine(backIntakeLaunchZoneShootingPose, backLaunchZoneParkPose))
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZoneShootingPose.heading,
                    backLaunchZoneParkPose.heading
                )
                .build()

            frontLaunchZoneShootingToPpgPreSpikeMark = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        frontLaunchZoneShootingPose,
                        frontLaunchZonePrePpgSpikeMarkPose,
                        frontLaunchZonePrePpgSpikeMarkControlPose
                    )
                )
                .setLinearHeadingInterpolation(
                    frontLaunchZoneShootingPose.heading,
                    frontLaunchZonePrePpgSpikeMarkPose.heading
                )
                .build()

            frontLaunchZonePpgPreSpikeMarkToPpgSpikeMark = follower.pathBuilder()
                .addPath(BezierLine(frontLaunchZonePrePpgSpikeMarkPose, frontLaunchZonePpgSpikeMarkPose))
                .setLinearHeadingInterpolation(
                    frontLaunchZonePrePpgSpikeMarkPose.heading,
                    backIntakeLaunchZoneShootingPose.heading
                )
                .build()

            frontLaunchZonePpgSpikeMarkToFrontLaunchZoneShooting = follower.pathBuilder()
                .addPath(BezierLine(frontLaunchZonePpgSpikeMarkPose, frontLaunchZoneShootingPose))
                .setLinearHeadingInterpolation(
                    frontLaunchZonePpgSpikeMarkPose.heading,
                    frontLaunchZoneShootingPose.heading
                )
                .build()

            frontLaunchZoneShootingToPgpPreSpikeMark = follower.pathBuilder()
                .addPath(BezierLine(frontLaunchZonePpgSpikeMarkPose, frontLaunchZoneShootingPose))
                .setLinearHeadingInterpolation(
                    frontLaunchZonePpgSpikeMarkPose.heading,
                    frontLaunchZoneShootingPose.heading
                )
                .build()
        }
    }
}