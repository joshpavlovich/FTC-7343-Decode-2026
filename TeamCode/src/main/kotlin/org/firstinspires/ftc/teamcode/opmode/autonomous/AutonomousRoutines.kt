package org.firstinspires.ftc.teamcode.opmode.autonomous

import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.extensions.pedro.FollowPath
import org.firstinspires.ftc.teamcode.subsystem.FLYWHEEL_MOTOR_POWER_BACK_LAUNCH_MID_AUTO_ZONE
import org.firstinspires.ftc.teamcode.subsystem.FLYWHEEL_MOTOR_POWER_FRONT_AUTO_LAUNCH_ZONE
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem

object AutonomousRoutines {

    val frontLaunchZoneLeaveParkAutoRoutine
        get() = SequentialGroup(FollowPath(PathManager.frontLaunchZoneStartToPark, true))

    val frontLaunchShootingAutoRoutine
        get() = SequentialGroup(
            // Starting at the front launch zone going to the front launch zone shooting and
            // starting flywheel motor leading into shooting
            FlywheelShooterSubsystem.spin(FLYWHEEL_MOTOR_POWER_FRONT_AUTO_LAUNCH_ZONE),
            FollowPath(PathManager.frontLaunchZoneStartToFrontLaunchZoneShooting, true),
            Delay(1.0),
            FlywheelShooterSubsystem.transfer(),
            Delay(4.0),
            FlywheelShooterSubsystem.kickArtifact(),
            Delay(1.0),
            FlywheelShooterSubsystem.resetKickerServo(),
            Delay(3.0),
            FlywheelShooterSubsystem.kickArtifact(),
            Delay(1.0),
            FlywheelShooterSubsystem.resetKickerServo(),
            Delay(3.0),
            FlywheelShooterSubsystem.kickArtifact(),
            Delay(1.0),
            FlywheelShooterSubsystem.kickArtifact(),
            Delay(1.0),
            FlywheelShooterSubsystem.resetKickerServo(),
            Delay(2.0),
            FlywheelShooterSubsystem.stopSpin(),

            // Go from the front launch zone to outside the launch zone tape in order to get
            // leave points
            FollowPath(PathManager.frontLaunchZoneShootingToFrontLaunchZoneLeavePark, true)
        )

    val backLaunchMidShootingAutoRoutine
        get() = SequentialGroup(
            // Starting at the back launch zone going to the back launch zone mid shooting and
            // starting flywheel motor leading into shooting
            FlywheelShooterSubsystem.spin(FLYWHEEL_MOTOR_POWER_BACK_LAUNCH_MID_AUTO_ZONE),
            FollowPath(PathManager.backLaunchZoneStartToBackLaunchZoneShooting, true),
            Delay(1.0),
            FlywheelShooterSubsystem.transfer(),
            Delay(4.0),
            FlywheelShooterSubsystem.kickArtifact(),
            Delay(1.0),
            FlywheelShooterSubsystem.resetKickerServo(),
            Delay(3.0),
            FlywheelShooterSubsystem.kickArtifact(),
            Delay(1.0),
            FlywheelShooterSubsystem.resetKickerServo(),
            Delay(3.0),
            FlywheelShooterSubsystem.kickArtifact(),
            Delay(1.0),
            FlywheelShooterSubsystem.kickArtifact(),
            Delay(1.0),
            FlywheelShooterSubsystem.resetKickerServo(),
            Delay(2.0),
            FlywheelShooterSubsystem.stopSpin(),

            // Go from the back launch zone to outside the launch zone tape in order to get
            // leave points and line up robot to open the gate at start of TeleOp
            Delay(3.0),
            FollowPath(PathManager.backLaunchZoneShootingToBackLaunchZoneLeavePark, true)
        )
}