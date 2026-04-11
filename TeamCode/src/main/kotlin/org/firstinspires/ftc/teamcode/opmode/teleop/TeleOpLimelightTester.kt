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
import org.firstinspires.ftc.teamcode.subsystem.LimelightSubsystem

/**
 * A testing OpMode for the IntakeSubsystem.
 * Allows for manual verification of intake forward and reverse functionality.
 */
@TeleOp(name = "TeleOp Limelight Subsytem Tester")
class TeleOpLimelightTester : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(LimelightSubsystem),
            BulkReadComponent,
            BindingsComponent
        )
    }

    // Drivetrain motors configuration
    private val frontLeftMotor = MotorEx("front_left_motor").brakeMode()
    private val frontRightMotor = MotorEx("front_right_motor").brakeMode().reversed()
    private val backLeftMotor = MotorEx("back_left_motor").brakeMode()
    private val backRightMotor = MotorEx("back_right_motor").brakeMode().reversed()

    /**
     * Sets up the driver-controlled movement and button bindings when the start button is pressed.
     */
    override fun onStartButtonPressed() {
        resetRuntime()

        // Initialize Mecanum drive control with gamepad1 sticks
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
    }

    /**
     * Updates telemetry data on each loop iteration.
     */
    override fun onUpdate() {
        ActiveOpMode.telemetry.update()
    }
}
