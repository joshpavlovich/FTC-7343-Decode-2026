package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierLine
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutonomousStateManager
import org.firstinspires.ftc.teamcode.opmode.autonomous.PathManager.endGameBasZoneParkPose
import org.firstinspires.ftc.teamcode.panels.Drawing
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystem.FLYWHEEL_MOTOR_POWER_BACK_LAUNCH_ZONE
import org.firstinspires.ftc.teamcode.subsystem.FLYWHEEL_MOTOR_POWER_FRONT_LAUNCH_ZONE
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem.addShooterDetails

private const val RIGHT_TRIGGER_MINIMUM_VALUE = 0.3

@Disabled
@TeleOp(name = "Pedro TeleOp")
class PedroTeleOp : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(FlywheelShooterSubsystem),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent
        )

        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
    }

    override fun onInit() {
        super.onInit()

        PedroComponent.follower.setStartingPose(AutonomousStateManager.startPoseAtEndOfAuto)
        PedroComponent.follower.update()

        Drawing.init()
    }

    override fun onStartButtonPressed() {
        val driverControlled = PedroDriverControlled(
            drivePower = Gamepads.gamepad1.leftStickY.negate(),
            strafePower = Gamepads.gamepad1.leftStickX.negate(),
            turnPower = Gamepads.gamepad1.rightStickX.negate(),
            robotCentric = true // TODO: WE NEED TO TEST FIELD CENTRIC VS ROBOT CENTRIC, SEE https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric
        )
        driverControlled()

        Gamepads.gamepad1.rightTrigger.atLeast(RIGHT_TRIGGER_MINIMUM_VALUE)
            .whenBecomesTrue(FlywheelShooterSubsystem.kickArtifact())
            .whenBecomesFalse(FlywheelShooterSubsystem.resetKickerServo())
        Gamepads.gamepad1.circle.toggleOnBecomesTrue().whenBecomesTrue(
            FlywheelShooterSubsystem.spin(FLYWHEEL_MOTOR_POWER_FRONT_LAUNCH_ZONE)
        ) whenBecomesFalse FlywheelShooterSubsystem.stopSpin()
        Gamepads.gamepad1.square.toggleOnBecomesTrue().whenBecomesTrue(
            FlywheelShooterSubsystem.spin(FLYWHEEL_MOTOR_POWER_BACK_LAUNCH_ZONE)
        ).whenBecomesFalse(FlywheelShooterSubsystem.stopSpin())

        // TODO: ADD PARK POSE AND COMMAND FOR END GAME
        Gamepads.gamepad1.ps.whenBecomesTrue {
            parkRobotToBaseZone()
        }
    }

    private fun parkRobotToBaseZone() {
        // TODO: ONLY EXECUTE THIS IF IN END GAME?
        // TODO: ONLY EXECUTE IF FOLLOWER IS NOT BUSY?
        // TODO: ONLY EXECUTE ONCE
        val currentPose = PedroComponent.follower.pose
        val endGameBasZoneParkPath = if (AutonomousStateManager.isRedAlliance) {
            PedroComponent.follower.pathBuilder()
                .addPath(BezierLine(currentPose.mirror(), endGameBasZoneParkPose.mirror()))
                .setLinearHeadingInterpolation(
                    currentPose.mirror().heading,
                    endGameBasZoneParkPose.mirror().heading
                )
                .build()
        } else {
            PedroComponent.follower.pathBuilder()
                .addPath(BezierLine(currentPose, endGameBasZoneParkPose))
                .setLinearHeadingInterpolation(
                    currentPose.mirror().heading,
                    endGameBasZoneParkPose.heading
                )
                .build()
        }

        PedroComponent.follower.followPath(endGameBasZoneParkPath, true)
    }

    override fun onUpdate() {
        // TODO: TEST BREAK OUT OF PEDRO PATH
//        if (!PedroComponent.follower.isBusy) {
//            PedroComponent.follower.breakFollowing()
//        }

        Drawing.drawDebug(PedroComponent.follower)
        ActiveOpMode.telemetry.addData("Current pose", PedroComponent.follower.pose)
        ActiveOpMode.telemetry.addData("isBusy", PedroComponent.follower.isBusy)


        ActiveOpMode.telemetry.addShooterDetails()
        ActiveOpMode.telemetry.update()
    }
}