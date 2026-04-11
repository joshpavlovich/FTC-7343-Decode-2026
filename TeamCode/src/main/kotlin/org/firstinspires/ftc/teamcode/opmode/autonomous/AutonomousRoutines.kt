package org.firstinspires.ftc.teamcode.opmode.autonomous

import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem

/**
 * AutonomousRoutines contains a collection of predefined autonomous command groups.
 * These routines define the sequence of actions the robot performs during the autonomous period,
 * such as following paths, shooting artifacts, and intaking from spike marks.
 */
object AutonomousRoutines {

    /**
     * A simple routine that leaves the front launch zone and parks.
     */
    val frontLaunchZoneLeaveParkAutoRoutine
        get() = SequentialGroup(FollowPath(PathManager.frontLaunchZoneStartToPark, true))

    /**
     * A routine that starts at the front launch zone, moves to a shooting position,
     * launches multiple artifacts, and then moves to a parking position.
     */
    val frontLaunchShootingAutoRoutine
        get() = SequentialGroup(
            // Starting at the front launch zone going to the front launch zone shooting and
            // starting flywheel motor leading into shooting
            FollowPath(PathManager.frontLaunchZoneStartToFrontLaunchZoneShooting, true, 0.6),
            FlywheelShooterSubsystem.kickArtifact.afterTime(5.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            FlywheelShooterSubsystem.kickArtifact.afterTime(3.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            FlywheelShooterSubsystem.kickArtifact.afterTime(3.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            // Add a backup kick in case the first kick doesn't work
            FlywheelShooterSubsystem.kickArtifact.afterTime(1.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            // Go from the front launch zone to outside the launch zone tape in order to get leave points
            ParallelGroup(
                FollowPath(PathManager.frontLaunchZoneShootingToFrontLaunchZoneLeavePark, true),
                FlywheelShooterSubsystem.stopSpin
            ).afterTime(3.0)
        )

    /**
     * A routine that starts at the front launch zone, strafes to a back wall shooting position,
     * launches artifacts, and then parks near the wall.
     */
    val frontLaunchZoneStrafeStartWallShootingAutoRoutine
        get() = SequentialGroup(
            // Starting at the front launch zone going to the back launch zone wall shooting and
            // starting flywheel motor leading into shooting
            FollowPath(PathManager.frontLaunchZoneStrafeStartToBackLaunchZoneWallShooting, true, 0.7),
            FlywheelShooterSubsystem.kickArtifact.afterTime(3.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            FlywheelShooterSubsystem.kickArtifact.afterTime(3.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            FlywheelShooterSubsystem.kickArtifact.afterTime(3.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            // Add a backup kick in case the first kick doesn't work
            FlywheelShooterSubsystem.kickArtifact.afterTime(1.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            // Go from the back launch zone wall shooting to outside the launch zone tape in order
            // to get leave points turning to the loading zone
            ParallelGroup(
                FollowPath(PathManager.backLaunchZoneWallShootingToBackLaunchZoneWallPark, true),
                FlywheelShooterSubsystem.stopSpin
            ).afterTime(3.0)
        )

    /**
     * A routine that starts at the back launch zone, moves to a mid-field shooting position,
     * launches artifacts, and then parks.
     */
    val backLaunchMidShootingAutoRoutine
        get() = SequentialGroup(
            // Starting at the back launch zone going to the back launch zone mid shooting and
            // starting flywheel motor leading into shooting
            FollowPath(PathManager.backLaunchZoneStartToBackLaunchZoneShooting, true),
            FlywheelShooterSubsystem.kickArtifact.afterTime(5.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            FlywheelShooterSubsystem.kickArtifact.afterTime(3.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            FlywheelShooterSubsystem.kickArtifact.afterTime(3.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            // Add a backup kick in case the first kick doesn't work
            FlywheelShooterSubsystem.kickArtifact.afterTime(1.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            // Go from the back launch zone to outside the launch zone tape in order to get
            // leave points and line up robot to open the gate at start of TeleOp
            ParallelGroup(
                FollowPath(PathManager.backLaunchZoneShootingToBackLaunchZoneLeavePark, true),
                FlywheelShooterSubsystem.stopSpin
            ).afterTime(3.0)
        )

    /**
     * A complex routine that starts at the back launch zone, shoots, then proceeds to intake
     * artifacts from multiple spike marks and shoot them, before finally parking.
     */
    val backLaunchIntakeShootingAutoRoutine
        get() = SequentialGroup(
            // Starting at the back launch zone going to the back launch zone mid shooting and
            // starting flywheel motor leading into shooting
            FollowPath(PathManager.backLaunchZoneStartToBackIntakeLaunchZoneShooting, true),
            WaitUntil { !PedroComponent.follower.isBusy },
//            FlywheelShooterSubsystem.kickArtifact.afterTime(2.0),
            FlywheelShooterSubsystem.kickArtifact,
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            FlywheelShooterSubsystem.kickArtifact.afterTime(1.5),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            FlywheelShooterSubsystem.kickArtifact.afterTime(1.5),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            // Add a backup kick in case the first kick doesn't work
            FlywheelShooterSubsystem.kickArtifact.afterTime(1.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),

            FollowPath(PathManager.backIntakeLaunchZoneShootingToGppPreSpikeMark, true),
            WaitUntil { !PedroComponent.follower.isBusy },
            IntakeSubsystem.forward,
            FollowPath(PathManager.backIntakeLaunchZoneGppPreSpikeMarkToGppSpikeMark, true, 0.5),
            ParallelGroup(
                IntakeSubsystem.stop,
                FollowPath(
                    PathManager.backIntakeLaunchZoneGppSpikeMarkToBackLaunchZoneShooting,
                    true
                ),
            ),

            WaitUntil { !PedroComponent.follower.isBusy },
            FlywheelShooterSubsystem.kickArtifact,
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            FlywheelShooterSubsystem.kickArtifact.afterTime(1.5),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            FlywheelShooterSubsystem.kickArtifact.afterTime(1.5),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            // Add a backup kick in case the first kick doesn't work
            FlywheelShooterSubsystem.kickArtifact.afterTime(1.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),

            FollowPath(PathManager.backIntakeLaunchZoneShootingToPgpPreSpikeMark, true),
            WaitUntil { !PedroComponent.follower.isBusy },
            IntakeSubsystem.forward,
            FollowPath(PathManager.backIntakeLaunchZonePgpPreSpikeMarkToPgpSpikeMark, true),
            ParallelGroup(
                IntakeSubsystem.stop,
                FollowPath(
                    PathManager.backIntakeLaunchZonePgpSpikeMarkToBackLaunchZoneShooting,
                    true
                ),
            ),

            WaitUntil { !PedroComponent.follower.isBusy },
            FlywheelShooterSubsystem.kickArtifact,
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            FlywheelShooterSubsystem.kickArtifact.afterTime(1.5),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            FlywheelShooterSubsystem.kickArtifact.afterTime(1.5),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            // Add a backup kick in case the first kick doesn't work
            FlywheelShooterSubsystem.kickArtifact.afterTime(1.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),

            // Go from the back launch zone to outside the launch zone tape in order to get
            // leave points and line up robot to open the gate at start of TeleOp
            ParallelGroup(
                FollowPath(PathManager.backIntakeLaunchZoneShootingToBackLaunchZoneLeavePark, true),
                FlywheelShooterSubsystem.stopSpin
            )
        )
}
