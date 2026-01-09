package org.firstinspires.ftc.teamcode.opmode.autonomous

import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem

object AutonomousRoutines {

    val frontLaunchZoneLeaveParkAutoRoutine
        get() = SequentialGroup(FollowPath(PathManager.frontLaunchZoneStartToPark, true))

    val frontLaunchShootingAutoRoutine
        get() = SequentialGroup(
            // Starting at the front launch zone going to the front launch zone shooting and
            // starting flywheel motor leading into shooting
            FollowPath(PathManager.frontLaunchZoneStartToFrontLaunchZoneShooting, true),
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
            ParallelGroup(
                IntakeSubsystem.forward,
                FollowPath(PathManager.backIntakeLaunchZoneGppPreSpikeMarkToGppSpikeMark, true),
            ),
            ParallelGroup(
                IntakeSubsystem.stop,
                FollowPath(PathManager.backIntakeLaunchZoneGppSpikeMarkToBackLaunchZoneShooting, true),
            ),

            FlywheelShooterSubsystem.kickArtifact.afterTime(2.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            FlywheelShooterSubsystem.kickArtifact.afterTime(2.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            FlywheelShooterSubsystem.kickArtifact.afterTime(2.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            // Add a backup kick in case the first kick doesn't work
            FlywheelShooterSubsystem.kickArtifact.afterTime(1.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),

            FollowPath(PathManager.backIntakeLaunchZoneShootingToPgpPreSpikeMark, true),
            WaitUntil { !PedroComponent.follower.isBusy },
            ParallelGroup(
                IntakeSubsystem.forward,
                FollowPath(PathManager.backIntakeLaunchZonePgpPreSpikeMarkToPgpSpikeMark, true),
            ),
            ParallelGroup(
                IntakeSubsystem.stop,
                FollowPath(PathManager.backIntakeLaunchZonePgpSpikeMarkToBackLaunchZoneShooting, true),
            ),

            FlywheelShooterSubsystem.kickArtifact.afterTime(2.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            FlywheelShooterSubsystem.kickArtifact.afterTime(2.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            FlywheelShooterSubsystem.kickArtifact.afterTime(2.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),
            // Add a backup kick in case the first kick doesn't work
            FlywheelShooterSubsystem.kickArtifact.afterTime(1.0),
            FlywheelShooterSubsystem.resetKickerServo.afterTime(1.0),

            // Go from the back launch zone to outside the launch zone tape in order to get
            // leave points and line up robot to open the gate at start of TeleOp
            ParallelGroup(
                FollowPath(PathManager.backIntakeLaunchZoneShootingToBackLaunchZoneLeavePark, true),
                FlywheelShooterSubsystem.stopSpin
            ).afterTime(3.0)
        )
}