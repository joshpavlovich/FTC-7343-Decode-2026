package org.firstinspires.ftc.teamcode.opmode.autonomous

import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.extensions.pedro.FollowPath
import org.firstinspires.ftc.teamcode.subsystem.FLYWHEEL_MOTOR_POWER_FRONT_LAUNCH_ZONE
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem

object AutonomousRoutines {

    val frontLaunchZoneLeaveParkAutoRoutine
        get() = SequentialGroup(FollowPath(PathManager.frontLaunchZoneStartToPark, true))

    val frontLaunchShootingAutoRoutine
        get() = SequentialGroup(
            // Starting at the front launch zone going to the front launch zone shooting and
            // starting flywheel motor leading into shooting
            FlywheelShooterSubsystem.spin(FLYWHEEL_MOTOR_POWER_FRONT_LAUNCH_ZONE),
            FollowPath(PathManager.frontLaunchZoneStartToFrontLaunchZoneShooting, true),
            Delay(1.5),
            FlywheelShooterSubsystem.kickArtifact(),
            Delay(1.0),
            FlywheelShooterSubsystem.kickArtifact(),
            Delay(1.0),
            FlywheelShooterSubsystem.kickArtifact(),
            FlywheelShooterSubsystem.stopSpin(),
            // Moving to the ppg spike mark to tap artifacts into loading zone
            FollowPath(PathManager.frontLaunchZoneShootingToPpgSpikeMark),
            // Moving to the loading zone with the artifacts previously tapped and get artifacts
            // loaded into the robot than start flywheel motor leading into shooting
            FollowPath(PathManager.ppgSpikeMarkToLoadingZonePark, true),
            Delay(3.0),
            FlywheelShooterSubsystem.spin(FLYWHEEL_MOTOR_POWER_FRONT_LAUNCH_ZONE),
            // After done loading, move to the front launch zone and shoot
            FollowPath(PathManager.loadingZoneToFrontLaunchZoneShooting, true),
            Delay(1.5),
            FlywheelShooterSubsystem.kickArtifact(),
            Delay(1.0),
            FlywheelShooterSubsystem.kickArtifact(),
            Delay(1.0),
            FlywheelShooterSubsystem.kickArtifact(),
            FlywheelShooterSubsystem.stopSpin(),

            // TODO: DO WE HAVE ENOUGH TIME TO GO BACK TO THE LOADING ZONE, SHOOT AGAIN, AND LEAVE??
            //  IF NOT, THEN COMMENT OUT THE LINES BELOW
            FollowPath(PathManager.frontLaunchZoneShootingToLoadingZone, true),
            Delay(3.0),
            FlywheelShooterSubsystem.spin(FLYWHEEL_MOTOR_POWER_FRONT_LAUNCH_ZONE),
            // After done loading, move to the front launch zone and shoot
            FollowPath(PathManager.loadingZoneToFrontLaunchZoneShooting, true),
            Delay(1.5),
            FlywheelShooterSubsystem.kickArtifact(),
            Delay(1.0),
            FlywheelShooterSubsystem.kickArtifact(),
            Delay(1.0),
            FlywheelShooterSubsystem.kickArtifact(),
            FlywheelShooterSubsystem.stopSpin(),
            // TODO: DO WE HAVE ENOUGH TIME TO GO BACK TO THE LOADING ZONE, SHOOT AGAIN, AND LEAVE??
            //  IF NOT, THEN COMMENT OUT THE LINES ABOVE

            // Go from the front launch zone to outside the launch zone tape in order to get
            // leave points
            FollowPath(PathManager.frontLaunchZoneShootingToFrontLaunchZoneLeavePark, true)
        )
}