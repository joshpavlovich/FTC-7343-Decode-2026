package org.firstinspires.ftc.teamcode.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem

/**
 * A testing OpMode for the IntakeSubsystem.
 * Allows for manual verification of intake forward and reverse functionality.
 */
@Disabled
@TeleOp(name = "TeleOp Intake Tester")
class TeleOpIntakeTester : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(IntakeSubsystem),
            BulkReadComponent,
            BindingsComponent
        )
    }

    /**
     * Binds gamepad buttons to intake actions: Left Bumper for forward, Right Bumper for reverse.
     */
    override fun onStartButtonPressed() {
        Gamepads.gamepad1.rightBumper whenBecomesTrue IntakeSubsystem.reverse  whenBecomesFalse IntakeSubsystem.stop
        Gamepads.gamepad1.leftBumper whenBecomesTrue IntakeSubsystem.forward whenBecomesFalse IntakeSubsystem.stop
    }
}
