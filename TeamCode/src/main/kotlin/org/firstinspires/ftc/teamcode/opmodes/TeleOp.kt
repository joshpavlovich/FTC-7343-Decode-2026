package org.firstinspires.ftc.teamcode.guide.kotlin.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx

@TeleOp(name = "Team Sapphire: TeleOp")
class TeleOp : NextFTCOpMode() {
    init {
        addComponents(
//            SubsystemComponent(Lift, Claw),
            BulkReadComponent,
            BindingsComponent
        )
    }

    // change the names and directions to suit your robot
    private val frontLeftMotor = MotorEx("front_left_motor").reversed()
    private val frontRightMotor = MotorEx("front_right_motor")
    private val backLeftMotor = MotorEx("back_left_motor").reversed()
    private val backRightMotor = MotorEx("back_right_motor")

    override fun onStartButtonPressed() {
        val driverControlled = MecanumDriverControlled(
            frontLeftMotor = frontLeftMotor,
            frontRightMotor = frontRightMotor,
            backLeftMotor = backLeftMotor,
            backRightMotor = backRightMotor,
            drivePower = Gamepads.gamepad1.leftStickY,
            strafePower = Gamepads.gamepad1.leftStickX,
            turnPower = Gamepads.gamepad1.rightStickX
        )
        driverControlled()

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