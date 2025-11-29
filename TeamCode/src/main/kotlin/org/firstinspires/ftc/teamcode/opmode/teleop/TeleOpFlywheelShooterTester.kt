package org.firstinspires.ftc.teamcode.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem

@TeleOp(name = "TeleOp Flywheel Shooter Tester")
class TeleOpFlywheelShooterTester : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(FlywheelShooterSubsystem),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onStartButtonPressed() {
        Gamepads.gamepad1.triangle.toggleOnBecomesTrue() whenBecomesTrue FlywheelShooterSubsystem.transfer() whenBecomesFalse FlywheelShooterSubsystem.stopTransfer()
        Gamepads.gamepad1.circle.toggleOnBecomesTrue() whenBecomesTrue FlywheelShooterSubsystem.spin(.80) whenBecomesFalse FlywheelShooterSubsystem.stopSpin()
        Gamepads.gamepad1.square.toggleOnBecomesTrue() whenBecomesTrue FlywheelShooterSubsystem.spin(.65) whenBecomesFalse FlywheelShooterSubsystem.stopSpin()
    }
}