package org.firstinspires.ftc.teamcode.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem

@TeleOp(name = "Team Sapphire: TeleOp")
class TeleOp : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(IntakeSubsystem),
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

        Gamepads.gamepad1.rightBumper whenBecomesTrue IntakeSubsystem.reverse whenBecomesFalse IntakeSubsystem.stop
        Gamepads.gamepad1.leftBumper whenBecomesTrue IntakeSubsystem.forward whenBecomesFalse IntakeSubsystem.stop

//        Gamepads.gamepad2.dpadUp whenBecomesTrue Lift.toHigh whenBecomesFalse Claw.open
//
//        (Gamepads.gamepad2.rightTrigger greaterThan 0.2)
//            .whenBecomesTrue(
//                Claw.close.then(Lift.toHigh)
//            )
//
//        Gamepads.gamepad2.leftBumper whenBecomesTrue Claw.open.and(Lift.toLow)
    }
}