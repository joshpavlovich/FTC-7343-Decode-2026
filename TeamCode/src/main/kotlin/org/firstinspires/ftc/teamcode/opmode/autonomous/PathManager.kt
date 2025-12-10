package org.firstinspires.ftc.teamcode.opmode.autonomous

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.units.deg

object PathManager {

    // ALL POSES ARE ON BLUE SCORING/GOAL SIDE OF FIELD!!!

    val frontLaunchZoneStartPose = Pose(56.0, 8.0, 90.deg.inRad)

    val frontLaunchZoneShootingPose = Pose(56.0, 21.258, 105.deg.inRad)

    val ppgSpikeMarkFromFrontLaunchZonePose = Pose(26.45, 46.767, 180.deg.inRad)

    val ppgSpikeMarkFromFrontLaunchZoneControlPointPose = Pose(61.555, 47.507)

    val loadingZoneParkPose = Pose(18.0, 11.0, 180.deg.inRad)

    val frontLaunchZoneLeaveParkPose = Pose(36.0, 8.0, 90.deg.inRad)

    val backLaunchZoneStartPose = Pose(33.5, 134.5, 90.deg.inRad)

    val backLaunchZoneShootingPose = Pose(56.0, 86.0, 135.deg.inRad)

    val backLaunchZoneShootingControlPointPose = Pose(65.0, 122.0)

    val ppgSpikeMarkFromBackLaunchZonePose = Pose(29.0, 40.0, 145.deg.inRad)

    val ppgSpikeMarkFromBackLaunchZoneControlPointPose = Pose(62.0, 59.0)

    val loadingZoneToBackLaunchZoneControlPointPose = Pose(65.0, 35.0)

    val backLaunchZoneParkPose = Pose(42.0, 72.0, 90.deg.inRad)

    // TELEOP POSES
    val endGameBasZoneParkPose = Pose(38.75, 33.25, 90.deg.inRad)

    lateinit var frontLaunchZoneStartToPark: PathChain

    lateinit var frontLaunchZoneStartToFrontLaunchZoneShooting: PathChain

    lateinit var frontLaunchZoneShootingToPpgSpikeMark: PathChain

    lateinit var ppgSpikeMarkFromFrontToLoadingZonePark: PathChain

    lateinit var loadingZoneToFrontLaunchZoneShooting: PathChain

    lateinit var frontLaunchZoneShootingToLoadingZone: PathChain

    lateinit var frontLaunchZoneShootingToFrontLaunchZoneLeavePark: PathChain

    lateinit var backLaunchZoneStartToBackLaunchZoneShooting: PathChain

    lateinit var backLaunchZoneShootingToPpgSpikeMark: PathChain

    lateinit var backLaunchZoneShootingToLoadingZone: PathChain

    lateinit var ppgSpikeMarkFromBackLaunchToLoadingZonePark: PathChain

    lateinit var loadingZoneToBackLaunchZoneShooting: PathChain

    lateinit var backLaunchZoneShootingToBackLaunchZoneLeavePark: PathChain

    fun buildPaths(follower: Follower) {
        if (AutonomousStateManager.isRedAlliance) {
            frontLaunchZoneStartToPark = follower.pathBuilder()
                .addPath(BezierLine(frontLaunchZoneStartPose.mirror(), frontLaunchZoneLeaveParkPose.mirror()))
                .setLinearHeadingInterpolation(frontLaunchZoneStartPose.mirror().heading, frontLaunchZoneLeaveParkPose.mirror().heading)
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

            frontLaunchZoneShootingToPpgSpikeMark = follower.pathBuilder()
                .addPath(BezierCurve(frontLaunchZoneShootingPose.mirror(), ppgSpikeMarkFromFrontLaunchZoneControlPointPose.mirror(), ppgSpikeMarkFromFrontLaunchZonePose.mirror()))
                .setLinearHeadingInterpolation(frontLaunchZoneStartPose.mirror().heading, ppgSpikeMarkFromFrontLaunchZonePose.mirror().heading)
                .build()

            ppgSpikeMarkFromFrontToLoadingZonePark = follower.pathBuilder()
                .addPath(BezierLine(ppgSpikeMarkFromFrontLaunchZoneControlPointPose.mirror(), loadingZoneParkPose.mirror()))
                .setLinearHeadingInterpolation(ppgSpikeMarkFromFrontLaunchZoneControlPointPose.mirror().heading, loadingZoneParkPose.mirror().heading)
                .build()

            loadingZoneToFrontLaunchZoneShooting = follower.pathBuilder()
                .addPath(BezierLine(loadingZoneParkPose.mirror(), frontLaunchZoneShootingPose.mirror()))
                .setLinearHeadingInterpolation(loadingZoneParkPose.mirror().heading, frontLaunchZoneShootingPose.mirror().heading)
                .build()

            frontLaunchZoneShootingToLoadingZone = follower.pathBuilder()
                .addPath(BezierLine(frontLaunchZoneShootingPose.mirror(), loadingZoneParkPose.mirror()))
                .setLinearHeadingInterpolation(frontLaunchZoneShootingPose.mirror().heading, loadingZoneParkPose.mirror().heading)
                .build()
            
            frontLaunchZoneShootingToFrontLaunchZoneLeavePark = follower.pathBuilder()
                .addPath(BezierLine(frontLaunchZoneShootingPose.mirror(), frontLaunchZoneLeaveParkPose.mirror()))
                .setLinearHeadingInterpolation(frontLaunchZoneShootingPose.mirror().heading, frontLaunchZoneLeaveParkPose.mirror().heading)
                .build()

            backLaunchZoneStartToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(BezierCurve(backLaunchZoneStartPose.mirror(), backLaunchZoneShootingControlPointPose.mirror(), backLaunchZoneShootingPose.mirror()))
                .setLinearHeadingInterpolation(backLaunchZoneStartPose.mirror().heading, backLaunchZoneShootingPose.mirror().heading)
                .build()

            backLaunchZoneShootingToPpgSpikeMark = follower.pathBuilder()
                .addPath(BezierCurve(backLaunchZoneShootingPose.mirror(), ppgSpikeMarkFromBackLaunchZoneControlPointPose.mirror(), ppgSpikeMarkFromBackLaunchZonePose.mirror()))
                .setLinearHeadingInterpolation(backLaunchZoneShootingPose.mirror().heading, ppgSpikeMarkFromBackLaunchZonePose.mirror().heading)
                .build()

            ppgSpikeMarkFromBackLaunchToLoadingZonePark = follower.pathBuilder()
                .addPath(BezierLine(ppgSpikeMarkFromBackLaunchZonePose.mirror(), loadingZoneParkPose.mirror()))
                .setLinearHeadingInterpolation(ppgSpikeMarkFromBackLaunchZonePose.mirror().heading, loadingZoneParkPose.mirror().heading)
                .build()

            loadingZoneToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(BezierCurve(loadingZoneParkPose.mirror(), loadingZoneToBackLaunchZoneControlPointPose.mirror(), backLaunchZoneShootingPose.mirror()))
                .setLinearHeadingInterpolation(loadingZoneParkPose.mirror().heading, backLaunchZoneShootingPose.mirror().heading)
                .build()

            backLaunchZoneShootingToLoadingZone = follower.pathBuilder()
                .addPath(BezierCurve(backLaunchZoneShootingPose.mirror(), loadingZoneToBackLaunchZoneControlPointPose.mirror(), loadingZoneParkPose.mirror()))
                .setLinearHeadingInterpolation(backLaunchZoneShootingPose.mirror().heading, loadingZoneParkPose.mirror().heading)
                .build()

            backLaunchZoneShootingToBackLaunchZoneLeavePark = follower.pathBuilder()
                .addPath(BezierLine(backLaunchZoneShootingPose.mirror(), backLaunchZoneParkPose.mirror()))
                .setLinearHeadingInterpolation(backLaunchZoneShootingPose.mirror().heading, backLaunchZoneParkPose.mirror().heading)
                .build()
        } else {
            frontLaunchZoneStartToPark = follower.pathBuilder()
                .addPath(BezierLine(frontLaunchZoneStartPose, frontLaunchZoneLeaveParkPose))
                .setLinearHeadingInterpolation(frontLaunchZoneStartPose.heading, frontLaunchZoneLeaveParkPose.heading)
                .build()

            frontLaunchZoneStartToFrontLaunchZoneShooting = follower.pathBuilder()
                .addPath(BezierLine(frontLaunchZoneStartPose, frontLaunchZoneShootingPose))
                .setLinearHeadingInterpolation(frontLaunchZoneStartPose.heading, frontLaunchZoneShootingPose.heading)
                .build()

            frontLaunchZoneShootingToPpgSpikeMark = follower.pathBuilder()
                .addPath(BezierCurve(frontLaunchZoneShootingPose, ppgSpikeMarkFromFrontLaunchZoneControlPointPose, ppgSpikeMarkFromFrontLaunchZonePose))
                .setLinearHeadingInterpolation(frontLaunchZoneStartPose.heading, ppgSpikeMarkFromFrontLaunchZonePose.heading)
                .build()

            ppgSpikeMarkFromFrontToLoadingZonePark = follower.pathBuilder()
                .addPath(BezierLine(ppgSpikeMarkFromFrontLaunchZoneControlPointPose, loadingZoneParkPose))
                .setLinearHeadingInterpolation(ppgSpikeMarkFromFrontLaunchZoneControlPointPose.heading, loadingZoneParkPose.heading)
                .build()

            loadingZoneToFrontLaunchZoneShooting = follower.pathBuilder()
                .addPath(BezierLine(loadingZoneParkPose, frontLaunchZoneShootingPose))
                .setLinearHeadingInterpolation(loadingZoneParkPose.heading, frontLaunchZoneShootingPose.heading)
                .build()

            frontLaunchZoneShootingToLoadingZone = follower.pathBuilder()
                .addPath(BezierLine(frontLaunchZoneShootingPose, loadingZoneParkPose))
                .setLinearHeadingInterpolation(frontLaunchZoneShootingPose.heading, loadingZoneParkPose.heading)
                .build()

            frontLaunchZoneShootingToFrontLaunchZoneLeavePark = follower.pathBuilder()
                .addPath(BezierLine(frontLaunchZoneShootingPose, frontLaunchZoneLeaveParkPose.mirror()))
                .setLinearHeadingInterpolation(frontLaunchZoneShootingPose.mirror().heading, frontLaunchZoneLeaveParkPose.mirror().heading)
                .build()

            backLaunchZoneStartToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(BezierCurve(backLaunchZoneStartPose, backLaunchZoneShootingControlPointPose, backLaunchZoneShootingPose))
                .setLinearHeadingInterpolation(backLaunchZoneStartPose.heading, backLaunchZoneShootingPose.heading)
                .build()

            backLaunchZoneShootingToPpgSpikeMark = follower.pathBuilder()
                .addPath(BezierCurve(backLaunchZoneShootingPose, ppgSpikeMarkFromBackLaunchZoneControlPointPose, ppgSpikeMarkFromBackLaunchZonePose))
                .setLinearHeadingInterpolation(backLaunchZoneShootingPose.heading, ppgSpikeMarkFromBackLaunchZonePose.heading)
                .build()

            ppgSpikeMarkFromBackLaunchToLoadingZonePark = follower.pathBuilder()
                .addPath(BezierLine(ppgSpikeMarkFromBackLaunchZonePose, loadingZoneParkPose))
                .setLinearHeadingInterpolation(ppgSpikeMarkFromBackLaunchZonePose.heading, loadingZoneParkPose.heading)
                .build()

            loadingZoneToBackLaunchZoneShooting = follower.pathBuilder()
                .addPath(BezierCurve(loadingZoneParkPose, loadingZoneToBackLaunchZoneControlPointPose, backLaunchZoneShootingPose))
                .setLinearHeadingInterpolation(loadingZoneParkPose.heading, backLaunchZoneShootingPose.heading)
                .build()

            backLaunchZoneShootingToLoadingZone = follower.pathBuilder()
                .addPath(BezierCurve(backLaunchZoneShootingPose, loadingZoneToBackLaunchZoneControlPointPose, loadingZoneParkPose))
                .setLinearHeadingInterpolation(backLaunchZoneShootingPose.heading, loadingZoneParkPose.heading)
                .build()

            backLaunchZoneShootingToBackLaunchZoneLeavePark = follower.pathBuilder()
                .addPath(BezierLine(backLaunchZoneShootingPose, backLaunchZoneParkPose))
                .setLinearHeadingInterpolation(backLaunchZoneShootingPose.heading, backLaunchZoneParkPose.heading)
                .build()
        }
    }
}