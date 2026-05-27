package org.firstinspires.ftc.teamcode.opmode.autonomous

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.units.deg

/**
 * PathManager is responsible for defining all the robot's poses and paths used during autonomous.
 * It manages both Blue and Red alliance paths by mirroring the base Blue alliance coordinates.
 */
object PathManager {

    // ALL POSES ARE ON BLUE ALLIANCE SIDE OF FIELD
    val backLaunchZoneStartPose = Pose(33.5, 134.5, 90.deg.inRad)
    val backLaunchZoneShootingPose = Pose(56.0, 86.0, 132.5.deg.inRad)
    val backLaunchZoneShootingControlPointPose = Pose(65.0, 122.0)
    val backLaunchZoneParkPose = Pose(42.0, 72.0, 270.deg.inRad)

    val backIntakeLaunchZoneShootingPose = Pose(56.0, 86.0, 132.5.deg.inRad)
    val backIntakeLaunchZonePreGppSpikeMarkPose = Pose(42.0, 84.21, 180.deg.inRad)
    val backIntakeLaunchZonePreGppSpikeMarkControlPose = Pose(48.3, 95.79)
    val backIntakeLaunchZoneGppSpikeMarkPose = Pose(19.37, 83.90, 180.deg.inRad)
    val backIntakeLaunchZonePrePgpSpikeMarkPose = Pose(42.0, 59.63, 180.deg.inRad)
    val backIntakeLaunchZonePgpSpikeMarkPose = Pose(17.37, 60.0, 180.deg.inRad)
    val backIntakeLaunchZonePrePgpSpikeMarkControlPose = Pose(48.3, 64.6)

    val backLaunchZoneWallShootingPose = Pose(60.0, 132.0, 178.deg.inRad)
    val backLaunchZoneWallParkPose = Pose(60.0, 42.0, 340.deg.inRad)
    val frontLaunchZoneStrafeStartPose = Pose(57.0, 9.0, 180.deg.inRad)

    val frontLaunchZoneStartPose = Pose(57.0, 9.0, 90.deg.inRad)
    val frontLaunchZoneShootingPose = Pose(59.5, 24.0, 120.0.deg.inRad)

    val frontLaunchZoneLeaveParkPose = Pose(36.0, 9.0, 90.deg.inRad)

    val frontIntakeLaunchZoneBackShootingPose = Pose(58.0, 75.5, 128.0.deg.inRad)
    val frontIntakeLaunchZonePrePpgSpikeMarkPose = Pose(42.0, 35.5, 180.deg.inRad)
    val frontIntakeLaunchZonePrePpgSpikeMarkControlPose = Pose(58.5, 34.5)
    val frontIntakeLaunchZonePpgSpikeMarkPose = Pose(18.0, 35.5, 180.deg.inRad)
    val frontIntakeLaunchZonePrePpgSpikeMarkControlToShootingPose = Pose(58.75, 37.0)
    val frontIntakeLaunchZonePreLoadingZoneFirstPose = Pose(14.85, 11.75, 180.deg.inRad)
    val frontIntakeLaunchZonePreLoadingZoneFirstControlPose = Pose(55.0, 7.5)
    val frontIntakeLaunchZoneLoadingZoneFirstPose = Pose(9.0, 11.5, 180.deg.inRad)
    val frontIntakeLaunchZonePreLoadingZoneSecondPose = Pose(20.0, 9.0, 180.deg.inRad)
    val frontIntakeLaunchZoneLoadingZoneSecondPose = Pose(9.0, 9.0, 180.deg.inRad)
    val frontIntakeLaunchZoneLoadingZoneControlToShootingPose = Pose(58.25, 12.75)
    val frontIntakeLaunchZoneLeaveParkPose = Pose(58.0, 60.5, 180.0.deg.inRad)

    // TELEOP POSES
    //Goes to Parking Square
    val endGameBaseZoneParkPose = Pose(105.25, 33.25, 90.deg.inRad)

    //Goes to back shooting zone close to the wall
    val blueBackWallShootingPose = Pose(60.0, 128.5, 178.deg.inRad)

    //Goes to closer shooting zone
    val blueBackShootingPose = Pose(62.0, 82.0, 130.deg.inRad)

    //Goes to far shooing zone
    val blueFrontShootingPose = Pose(81.0, 21.0, 123.deg.inRad)

    //Goes to gate
    val blueGoalGatePose = Pose(30.0, 67.0, 270.deg.inRad)

    // FIELD LOCATION POSES
    val blueGoalPose = Pose(16.3, 131.8, 110.0.deg.inRad)

    /**
     * Gets the goal pose based on the current alliance.
     */
    val goalPose: Pose
        get() = if (AutonomousStateManager.isRedAlliance) {
            blueGoalPose.mirror()
        } else {
            blueGoalPose
        }

    lateinit var backLaunchZoneWallShootingToBackLaunchZoneWallPark: PathChain
    lateinit var backLaunchZoneStartToBackLaunchZoneShooting: PathChain
    lateinit var backLaunchZoneStartToBackIntakeLaunchZoneShooting: PathChain
    lateinit var backLaunchZoneShootingToBackLaunchZoneLeavePark: PathChain

    lateinit var backIntakeLaunchZoneShootingToGppPreSpikeMark: PathChain
    lateinit var backIntakeLaunchZoneGppPreSpikeMarkToGppSpikeMark: PathChain
    lateinit var backIntakeLaunchZoneGppSpikeMarkToBackLaunchZoneShooting: PathChain
    lateinit var backIntakeLaunchZoneShootingToPgpPreSpikeMark: PathChain
    lateinit var backIntakeLaunchZonePgpPreSpikeMarkToPgpSpikeMark: PathChain
    lateinit var backIntakeLaunchZonePgpSpikeMarkToBackLaunchZoneShooting: PathChain
    lateinit var backIntakeLaunchZoneShootingToBackLaunchZoneLeavePark: PathChain

    lateinit var frontLaunchZoneStartToPark: PathChain
    lateinit var frontLaunchZoneStartToFrontLaunchZoneShooting: PathChain
    lateinit var frontLaunchZoneShootingToFrontLaunchZoneLeavePark: PathChain
    lateinit var frontLaunchZoneStrafeStartToBackLaunchZoneWallShooting: PathChain

    lateinit var frontLaunchZoneStartToToBackLaunchZoneShooting: PathChain
    lateinit var frontLaunchZoneShootingToPpgPreSpikeMark: PathChain
    lateinit var frontLaunchZonePpgPreSpikeMarkToPpgSpikeMark: PathChain
    lateinit var frontLaunchZonePpgSpikeMarkToToBackLaunchZoneShooting: PathChain
    lateinit var frontLaunchZoneShootingToPreLoadingZoneFirstPose: PathChain
    lateinit var frontLaunchZonePreLoadingZoneFirstPoseToLoadingZoneFirstPose: PathChain
    lateinit var frontLaunchZoneLoadingZoneFirstPoseToLoadingZonePreSecondPose: PathChain
    lateinit var frontLaunchZoneLoadingZonePreSecondPoseToLoadingZoneSecondPose: PathChain
    lateinit var frontLaunchZoneLoadingZoneToBackLaunchZoneShooting: PathChain
    lateinit var frontLaunchZoneShootingToBackLaunchZoneLeavePark: PathChain

    /**
     * Builds all path chains based on the current alliance state.
     * This should be called during the initialization phase of an OpMode.
     *
     * @param follower The Pedro Pathing follower instance used to build the paths.
     */
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
                .addPath(
                    BezierLine(
                        backLaunchZoneStartPose.mirror(),
                        backIntakeLaunchZoneShootingPose.mirror()
                    )
                )
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
                .addPath(
                    BezierLine(
                        backIntakeLaunchZonePreGppSpikeMarkPose.mirror(),
                        backIntakeLaunchZoneGppSpikeMarkPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZonePreGppSpikeMarkPose.mirror().heading,
                    backIntakeLaunchZoneGppSpikeMarkPose.mirror().heading
                )
                .build()

            backIntakeLaunchZoneGppSpikeMarkToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(
                    BezierLine(
                        backIntakeLaunchZoneGppSpikeMarkPose.mirror(),
                        backIntakeLaunchZoneShootingPose.mirror()
                    )
                )
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
                .addPath(
                    BezierLine(
                        backIntakeLaunchZonePrePgpSpikeMarkPose.mirror(),
                        backIntakeLaunchZonePgpSpikeMarkPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZonePrePgpSpikeMarkPose.mirror().heading,
                    backIntakeLaunchZonePgpSpikeMarkPose.mirror().heading
                )
                .build()

            backIntakeLaunchZonePgpSpikeMarkToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(
                    BezierLine(
                        backIntakeLaunchZonePgpSpikeMarkPose.mirror(),
                        backIntakeLaunchZoneShootingPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZonePgpSpikeMarkPose.mirror().heading,
                    backIntakeLaunchZoneShootingPose.mirror().heading
                )
                .build()

            backIntakeLaunchZoneShootingToBackLaunchZoneLeavePark = follower.pathBuilder()
                .addPath(
                    BezierLine(
                        backIntakeLaunchZoneShootingPose.mirror(),
                        backLaunchZoneParkPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZoneShootingPose.mirror().heading,
                    backLaunchZoneParkPose.mirror().heading
                )
                .build()

            frontLaunchZoneStartToToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(BezierLine(frontLaunchZoneStartPose.mirror(), frontIntakeLaunchZoneBackShootingPose.mirror()))
                .setLinearHeadingInterpolation(
                    frontLaunchZoneStartPose.mirror().heading,
                    frontIntakeLaunchZoneBackShootingPose.mirror().heading
                )
                .build()

            frontLaunchZoneShootingToPpgPreSpikeMark = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        frontIntakeLaunchZoneBackShootingPose.mirror(),
                        frontIntakeLaunchZonePrePpgSpikeMarkControlPose.mirror(),
                        frontIntakeLaunchZonePrePpgSpikeMarkPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    frontLaunchZoneShootingPose.mirror().heading,
                    frontIntakeLaunchZonePrePpgSpikeMarkPose.mirror().heading
                )
                .build()

            frontLaunchZonePpgPreSpikeMarkToPpgSpikeMark = follower.pathBuilder()
                .addPath(
                    BezierLine(
                        frontIntakeLaunchZonePrePpgSpikeMarkPose.mirror(),
                        frontIntakeLaunchZonePpgSpikeMarkPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    frontIntakeLaunchZonePrePpgSpikeMarkPose.mirror().heading,
                    backIntakeLaunchZoneShootingPose.mirror().heading
                )
                .build()

            frontLaunchZonePpgSpikeMarkToToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        frontIntakeLaunchZonePpgSpikeMarkPose.mirror(),
                        frontIntakeLaunchZonePrePpgSpikeMarkControlToShootingPose.mirror(),
                        frontIntakeLaunchZoneBackShootingPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    frontIntakeLaunchZonePpgSpikeMarkPose.mirror().heading,
                    frontIntakeLaunchZoneBackShootingPose.mirror().heading
                )
                .build()

            frontLaunchZoneShootingToPreLoadingZoneFirstPose = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        frontIntakeLaunchZoneBackShootingPose.mirror(),
                        frontIntakeLaunchZonePreLoadingZoneFirstControlPose.mirror(),
                        frontIntakeLaunchZonePreLoadingZoneFirstPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    frontLaunchZoneShootingPose.mirror().heading,
                    frontIntakeLaunchZonePreLoadingZoneFirstPose.mirror().heading
                )
                .build()

            frontLaunchZonePreLoadingZoneFirstPoseToLoadingZoneFirstPose = follower.pathBuilder()
                .addPath(BezierLine(frontIntakeLaunchZonePreLoadingZoneFirstPose.mirror(), frontIntakeLaunchZoneLoadingZoneFirstPose.mirror()))
                .setLinearHeadingInterpolation(
                    frontIntakeLaunchZonePreLoadingZoneFirstPose.mirror().heading,
                    frontIntakeLaunchZoneLoadingZoneFirstPose.mirror().heading
                )
                .build()

            frontLaunchZoneLoadingZoneFirstPoseToLoadingZonePreSecondPose = follower.pathBuilder()
                .addPath(BezierLine(frontIntakeLaunchZoneLoadingZoneFirstPose.mirror(), frontIntakeLaunchZonePreLoadingZoneSecondPose.mirror()))
                .setLinearHeadingInterpolation(
                    frontIntakeLaunchZoneLoadingZoneFirstPose.mirror().heading,
                    frontIntakeLaunchZonePreLoadingZoneSecondPose.mirror().heading
                )
                .build()

            frontLaunchZoneLoadingZonePreSecondPoseToLoadingZoneSecondPose = follower.pathBuilder()
                .addPath(BezierLine(frontIntakeLaunchZonePreLoadingZoneSecondPose.mirror(), frontIntakeLaunchZoneLoadingZoneSecondPose.mirror()))
                .setLinearHeadingInterpolation(
                    frontIntakeLaunchZonePreLoadingZoneSecondPose.mirror().heading,
                    frontIntakeLaunchZoneLoadingZoneSecondPose.mirror().heading
                )
                .build()

            frontLaunchZoneLoadingZoneToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        frontIntakeLaunchZoneLoadingZoneSecondPose.mirror(),
                        frontIntakeLaunchZoneLoadingZoneControlToShootingPose.mirror(),
                        frontIntakeLaunchZoneBackShootingPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    frontIntakeLaunchZoneLoadingZoneSecondPose.mirror().heading,
                    frontIntakeLaunchZoneBackShootingPose.mirror().heading
                )
                .build()

            frontLaunchZoneShootingToBackLaunchZoneLeavePark= follower.pathBuilder()
                .addPath(BezierLine(frontIntakeLaunchZoneBackShootingPose.mirror(), frontIntakeLaunchZoneLeaveParkPose.mirror()))
                .setLinearHeadingInterpolation(
                    frontIntakeLaunchZoneBackShootingPose.mirror().heading,
                    frontIntakeLaunchZoneLeaveParkPose.mirror().heading
                )
                .build()

            frontLaunchZoneStrafeStartToBackLaunchZoneWallShooting = follower.pathBuilder()
                .addPath(
                    BezierLine(
                        frontLaunchZoneStrafeStartPose.mirror(),
                        backLaunchZoneWallShootingPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    frontLaunchZoneStrafeStartPose.mirror().heading,
                    backLaunchZoneWallShootingPose.mirror().heading
                )
                .build()

            backLaunchZoneWallShootingToBackLaunchZoneWallPark = follower.pathBuilder()
                .addPath(
                    BezierLine(
                        backLaunchZoneWallShootingPose.mirror(),
                        backLaunchZoneWallParkPose.mirror()
                    )
                )
                .setLinearHeadingInterpolation(
                    backLaunchZoneWallShootingPose.mirror().heading,
                    backLaunchZoneWallParkPose.mirror().heading
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
                .addPath(
                    BezierLine(
                        backIntakeLaunchZonePreGppSpikeMarkPose,
                        backIntakeLaunchZoneGppSpikeMarkPose
                    )
                )
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZonePreGppSpikeMarkPose.heading,
                    backIntakeLaunchZoneGppSpikeMarkPose.heading
                )
                .build()

            backIntakeLaunchZoneGppSpikeMarkToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(
                    BezierLine(
                        backIntakeLaunchZoneGppSpikeMarkPose,
                        backIntakeLaunchZoneShootingPose
                    )
                )
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
                .addPath(
                    BezierLine(
                        backIntakeLaunchZonePrePgpSpikeMarkPose,
                        backIntakeLaunchZonePgpSpikeMarkPose
                    )
                )
                .setLinearHeadingInterpolation(
                    backIntakeLaunchZonePrePgpSpikeMarkPose.heading,
                    backIntakeLaunchZonePgpSpikeMarkPose.heading
                )
                .build()

            backIntakeLaunchZonePgpSpikeMarkToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(
                    BezierLine(
                        backIntakeLaunchZonePgpSpikeMarkPose,
                        backIntakeLaunchZoneShootingPose
                    )
                )
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

            frontLaunchZoneStartToToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(BezierLine(frontLaunchZoneStartPose, frontIntakeLaunchZoneBackShootingPose))
                .setLinearHeadingInterpolation(
                    frontLaunchZoneStartPose.heading,
                    frontIntakeLaunchZoneBackShootingPose.heading
                )
                .build()

            frontLaunchZoneShootingToPpgPreSpikeMark = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        frontIntakeLaunchZoneBackShootingPose,
                        frontIntakeLaunchZonePrePpgSpikeMarkControlPose,
                        frontIntakeLaunchZonePrePpgSpikeMarkPose
                    )
                )
                .setLinearHeadingInterpolation(
                    frontLaunchZoneShootingPose.heading,
                    frontIntakeLaunchZonePrePpgSpikeMarkPose.heading
                )
                .build()

            frontLaunchZonePpgPreSpikeMarkToPpgSpikeMark = follower.pathBuilder()
                .addPath(
                    BezierLine(
                        frontIntakeLaunchZonePrePpgSpikeMarkPose,
                        frontIntakeLaunchZonePpgSpikeMarkPose
                    )
                )
                .setLinearHeadingInterpolation(
                    frontIntakeLaunchZonePrePpgSpikeMarkPose.heading,
                    backIntakeLaunchZoneShootingPose.heading
                )
                .build()

            frontLaunchZonePpgSpikeMarkToToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        frontIntakeLaunchZonePpgSpikeMarkPose,
                        frontIntakeLaunchZonePrePpgSpikeMarkControlToShootingPose,
                        frontIntakeLaunchZoneBackShootingPose
                    )
                )
                .setLinearHeadingInterpolation(
                    frontIntakeLaunchZonePpgSpikeMarkPose.heading,
                    frontIntakeLaunchZoneBackShootingPose.heading
                )
                .build()

            frontLaunchZoneShootingToPreLoadingZoneFirstPose = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        frontIntakeLaunchZoneBackShootingPose,
                        frontIntakeLaunchZonePreLoadingZoneFirstControlPose,
                        frontIntakeLaunchZonePreLoadingZoneFirstPose
                    )
                )
                .setLinearHeadingInterpolation(
                    frontLaunchZoneShootingPose.heading,
                    frontIntakeLaunchZonePreLoadingZoneFirstPose.heading
                )
                .build()

            frontLaunchZonePreLoadingZoneFirstPoseToLoadingZoneFirstPose = follower.pathBuilder()
                .addPath(BezierLine(frontIntakeLaunchZonePreLoadingZoneFirstPose, frontIntakeLaunchZoneLoadingZoneFirstPose))
                .setLinearHeadingInterpolation(
                    frontIntakeLaunchZonePreLoadingZoneFirstPose.heading,
                    frontIntakeLaunchZoneLoadingZoneFirstPose.heading
                )
                .build()

            frontLaunchZoneLoadingZoneFirstPoseToLoadingZonePreSecondPose = follower.pathBuilder()
                .addPath(BezierLine(frontIntakeLaunchZoneLoadingZoneFirstPose, frontIntakeLaunchZonePreLoadingZoneSecondPose))
                .setLinearHeadingInterpolation(
                    frontIntakeLaunchZoneLoadingZoneFirstPose.heading,
                    frontIntakeLaunchZonePreLoadingZoneSecondPose.heading
                )
                .build()

            frontLaunchZoneLoadingZonePreSecondPoseToLoadingZoneSecondPose = follower.pathBuilder()
                .addPath(BezierLine(frontIntakeLaunchZonePreLoadingZoneSecondPose, frontIntakeLaunchZoneLoadingZoneSecondPose))
                .setLinearHeadingInterpolation(
                    frontIntakeLaunchZonePreLoadingZoneSecondPose.heading,
                    frontIntakeLaunchZoneLoadingZoneSecondPose.heading
                )
                .build()

            frontLaunchZoneLoadingZoneToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        frontIntakeLaunchZoneLoadingZoneSecondPose,
                        frontIntakeLaunchZoneLoadingZoneControlToShootingPose,
                        frontIntakeLaunchZoneBackShootingPose
                    )
                )
                .setLinearHeadingInterpolation(
                    frontIntakeLaunchZoneLoadingZoneSecondPose.heading,
                    frontIntakeLaunchZoneBackShootingPose.heading
                )
                .build()

            frontLaunchZoneShootingToBackLaunchZoneLeavePark= follower.pathBuilder()
                .addPath(BezierLine(frontIntakeLaunchZoneBackShootingPose, frontIntakeLaunchZoneLeaveParkPose))
                .setLinearHeadingInterpolation(
                    frontIntakeLaunchZoneBackShootingPose.heading,
                    frontIntakeLaunchZoneLeaveParkPose.heading
                )
                .build()

            frontLaunchZoneStrafeStartToBackLaunchZoneWallShooting = follower.pathBuilder()
                .addPath(BezierLine(frontLaunchZoneStrafeStartPose, backLaunchZoneWallShootingPose))
                .setLinearHeadingInterpolation(
                    frontLaunchZoneStrafeStartPose.heading,
                    backLaunchZoneWallShootingPose.heading
                )
                .build()

            backLaunchZoneWallShootingToBackLaunchZoneWallPark = follower.pathBuilder()
                .addPath(BezierLine(backLaunchZoneWallShootingPose, backLaunchZoneWallParkPose))
                .setLinearHeadingInterpolation(
                    backLaunchZoneWallShootingPose.heading,
                    backLaunchZoneWallParkPose.heading
                )
                .build()
        }
    }
}
