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

    val ppgSpikeMarkPose = Pose(26.45, 46.767, 180.deg.inRad)

    val ppgSpikeMarkControlPointPose = Pose(61.555, 47.507)

    val loadingZoneParkPose = Pose(20.888, 10.982, 180.deg.inRad)

    val frontLaunchZoneLeaveParkPose = Pose(36.0, 8.0, 90.deg.inRad)

    val endGameBasZoneParkPose = Pose(38.75, 33.25, 90.deg.inRad)

    lateinit var frontLaunchZoneStartToPark: PathChain

    lateinit var frontLaunchZoneStartToFrontLaunchZoneShooting: PathChain

    lateinit var frontLaunchZoneShootingToPpgSpikeMark: PathChain

    lateinit var ppgSpikeMarkToLoadingZonePark: PathChain

    lateinit var loadingZoneToFrontLaunchZoneShooting: PathChain

    lateinit var frontLaunchZoneShootingToLoadingZone: PathChain

    lateinit var frontLaunchZoneShootingToFrontLaunchZoneLeavePark: PathChain

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
                .addPath(BezierCurve(frontLaunchZoneShootingPose.mirror(), ppgSpikeMarkControlPointPose.mirror(), ppgSpikeMarkPose.mirror()))
                .setLinearHeadingInterpolation(frontLaunchZoneStartPose.mirror().heading, ppgSpikeMarkPose.mirror().heading)
                .build()

            ppgSpikeMarkToLoadingZonePark = follower.pathBuilder()
                .addPath(BezierLine(ppgSpikeMarkControlPointPose.mirror(), loadingZoneParkPose.mirror()))
                .setLinearHeadingInterpolation(ppgSpikeMarkControlPointPose.mirror().heading, loadingZoneParkPose.mirror().heading)
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
                .addPath(BezierCurve(frontLaunchZoneShootingPose, ppgSpikeMarkControlPointPose, ppgSpikeMarkPose))
                .setLinearHeadingInterpolation(frontLaunchZoneStartPose.heading, ppgSpikeMarkPose.heading)
                .build()

            ppgSpikeMarkToLoadingZonePark = follower.pathBuilder()
                .addPath(BezierLine(ppgSpikeMarkControlPointPose, loadingZoneParkPose))
                .setLinearHeadingInterpolation(ppgSpikeMarkControlPointPose.heading, loadingZoneParkPose.heading)
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
        }
    }
}