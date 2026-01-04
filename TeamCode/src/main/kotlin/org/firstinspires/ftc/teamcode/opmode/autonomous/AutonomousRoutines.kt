package org.firstinspires.ftc.teamcode.opmode.autonomous

import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.extensions.pedro.FollowPath
import org.firstinspires.ftc.teamcode.subsystem.FLYWHEEL_MOTOR_VELOCITY_BACK_AUTO_LAUNCH_ZONE
import org.firstinspires.ftc.teamcode.subsystem.FLYWHEEL_MOTOR_VELOCITY_FRONT_AUTO_LAUNCH_ZONE
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem

object AutonomousRoutines {

    val frontLaunchZoneLeaveParkAutoRoutine
        get() = SequentialGroup(FollowPath(PathManager.frontLaunchZoneStartToPark, true))

    val frontLaunchShootingAutoRoutine
        get() = SequentialGroup(
            // Starting at the front launch zone going to the front launch zone shooting and
            // starting flywheel motor leading into shooting
            ParallelGroup(
                FlywheelShooterSubsystem.startSpin(FLYWHEEL_MOTOR_VELOCITY_FRONT_AUTO_LAUNCH_ZONE),
                FollowPath(PathManager.frontLaunchZoneStartToFrontLaunchZoneShooting, true)
            ),
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
            ParallelGroup(
                FlywheelShooterSubsystem.startSpin(FLYWHEEL_MOTOR_VELOCITY_BACK_AUTO_LAUNCH_ZONE),
                FollowPath(PathManager.backLaunchZoneStartToBackLaunchZoneShooting, true),
            ),
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
}