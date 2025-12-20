package org.firstinspires.ftc.teamcode.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.subsystem.ColorSensorSubsystem
import org.firstinspires.ftc.teamcode.subsystem.FLYWHEEL_MOTOR_POWER_BACK_LAUNCH_ZONE
import org.firstinspires.ftc.teamcode.subsystem.FLYWHEEL_MOTOR_POWER_FRONT_LAUNCH_ZONE
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem.addShooterDetails

private const val RIGHT_TRIGGER_MINIMUM_VALUE = 0.5

@TeleOp(name = "Manual TeleOp")
class ManualTeleOp : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(FlywheelShooterSubsystem, ColorSensorSubsystem),
            BulkReadComponent,
            BindingsComponent
        )
    }

    // change the names and directions to suit your robot
    private val frontLeftMotor = MotorEx("front_left_motor").brakeMode()
    private val frontRightMotor = MotorEx("front_right_motor").brakeMode().reversed()
    private val backLeftMotor = MotorEx("back_left_motor").brakeMode()
    private val backRightMotor = MotorEx("back_right_motor").brakeMode().reversed()

    override fun onStartButtonPressed() {
        val driverControlled = MecanumDriverControlled(
            frontLeftMotor = frontLeftMotor,
            frontRightMotor = frontRightMotor,
            backLeftMotor = backLeftMotor,
            backRightMotor = backRightMotor,
            drivePower = Gamepads.gamepad1.leftStickY,
            strafePower = -Gamepads.gamepad1.leftStickX,
            turnPower = -Gamepads.gamepad1.rightStickX
        )
        driverControlled()

//        Gamepads.gamepad1.rightBumper whenBecomesTrue IntakeSubsystem.reverse whenBecomesFalse IntakeSubsystem.stop
//        Gamepads.gamepad1.leftBumper whenBecomesTrue IntakeSubsystem.forward whenBecomesFalse IntakeSubsystem.stop

        Gamepads.gamepad1.rightTrigger.atLeast(RIGHT_TRIGGER_MINIMUM_VALUE)
            .whenBecomesTrue(FlywheelShooterSubsystem.kickArtifact())
            .whenBecomesFalse(FlywheelShooterSubsystem.resetKickerServo())

        Gamepads.gamepad1.leftTrigger.atLeast(RIGHT_TRIGGER_MINIMUM_VALUE)
            .whenBecomesTrue(FlywheelShooterSubsystem.stopTransfer())

        Gamepads.gamepad1.circle.toggleOnBecomesTrue().whenBecomesTrue(
            FlywheelShooterSubsystem.spin(FLYWHEEL_MOTOR_POWER_FRONT_LAUNCH_ZONE)
        ) whenBecomesFalse FlywheelShooterSubsystem.stopSpin()

        Gamepads.gamepad1.square.toggleOnBecomesTrue().whenBecomesTrue(
            FlywheelShooterSubsystem.spin(FLYWHEEL_MOTOR_POWER_BACK_LAUNCH_ZONE)
        ).whenBecomesFalse(FlywheelShooterSubsystem.stopSpin())
    }

    override fun onUpdate() {
        ActiveOpMode.telemetry.addShooterDetails()
        ActiveOpMode.telemetry.update()
    }
}