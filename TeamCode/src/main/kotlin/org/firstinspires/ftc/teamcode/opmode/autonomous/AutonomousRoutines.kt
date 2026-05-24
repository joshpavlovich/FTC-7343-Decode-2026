package org.firstinspires.ftc.teamcode.opmode.autonomous

import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystem.GateSubsystem
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem

/**
 * AutonomousRoutines contains a collection of predefined autonomous command groups.
 * These routines define the sequence of actions the robot performs during the autonomous period,
 * such as following paths, shooting artifacts, and intake from spike marks.
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
            // start the intake
            IntakeSubsystem.forward,
            FollowPath(PathManager.frontLaunchZoneStartToFrontLaunchZoneShooting, true, 0.6),
            WaitUntil { !PedroComponent.follower.isBusy },
            GateSubsystem.open,
            // Go from the front launch zone to outside the launch zone tape in order to get leave points
            ParallelGroup(
                IntakeSubsystem.stop,
                GateSubsystem.close,
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
            IntakeSubsystem.forward,
            FollowPath(
                PathManager.frontLaunchZoneStrafeStartToBackLaunchZoneWallShooting,
                true,
                0.7
            ),
            WaitUntil { !PedroComponent.follower.isBusy },
            GateSubsystem.open,
            // Go from the back launch zone wall shooting to outside the launch zone tape in order
            // to get leave points turning to the loading zone
            ParallelGroup(
                IntakeSubsystem.stop,
                GateSubsystem.close,
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
            IntakeSubsystem.forward,
            FollowPath(PathManager.backLaunchZoneStartToBackLaunchZoneShooting, true),
            WaitUntil { !PedroComponent.follower.isBusy },
            GateSubsystem.open,
            // Go from the back launch zone to outside the launch zone tape in order to get
            // leave points and line up robot to open the gate at start of TeleOp
            ParallelGroup(
                IntakeSubsystem.stop,
                GateSubsystem.close,
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
            IntakeSubsystem.forward,
            FollowPath(PathManager.backLaunchZoneStartToBackIntakeLaunchZoneShooting, true),
            WaitUntil { !PedroComponent.follower.isBusy },
            GateSubsystem.open,
            GateSubsystem.close.afterTime(2.0),
            FollowPath(PathManager.backIntakeLaunchZoneShootingToGppPreSpikeMark, true),
            WaitUntil { !PedroComponent.follower.isBusy },
            FollowPath(PathManager.backIntakeLaunchZoneGppPreSpikeMarkToGppSpikeMark, true, 0.5),
            FollowPath(
                PathManager.backIntakeLaunchZoneGppSpikeMarkToBackLaunchZoneShooting,
                true
            ),
            WaitUntil { !PedroComponent.follower.isBusy },
            GateSubsystem.open,
            GateSubsystem.close.afterTime(2.0),
            FollowPath(PathManager.backIntakeLaunchZoneShootingToPgpPreSpikeMark, true),
            WaitUntil { !PedroComponent.follower.isBusy },
            FollowPath(PathManager.backIntakeLaunchZonePgpPreSpikeMarkToPgpSpikeMark, true),
            FollowPath(
                PathManager.backIntakeLaunchZonePgpSpikeMarkToBackLaunchZoneShooting,
                true
            ),
            WaitUntil { !PedroComponent.follower.isBusy },
            GateSubsystem.open,
            GateSubsystem.close.afterTime(2.0),
            // Go from the back launch zone to outside the launch zone tape in order to get
            // leave points and line up robot to open the gate at start of TeleOp
            ParallelGroup(
                IntakeSubsystem.stop,
                GateSubsystem.close,
                FollowPath(PathManager.backIntakeLaunchZoneShootingToBackLaunchZoneLeavePark, true),
                FlywheelShooterSubsystem.stopSpin
            )
        )
}
