package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem

@TeleOp(name = "Team Sapphire: TeleOp Flywheel Shooter")
class TeleOpFlywheelShooter : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(FlywheelShooterSubsystem),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onStartButtonPressed() {
        Gamepads.gamepad1.circle.toggleOnBecomesTrue() whenBecomesTrue FlywheelShooterSubsystem.spin() whenBecomesFalse FlywheelShooterSubsystem.stopSpin
        Gamepads.gamepad1.triangle.toggleOnBecomesTrue() whenBecomesTrue FlywheelShooterSubsystem.spin(.75) whenBecomesFalse FlywheelShooterSubsystem.stopSpin
        Gamepads.gamepad1.square.toggleOnBecomesTrue() whenBecomesTrue FlywheelShooterSubsystem.spin(.50) whenBecomesFalse FlywheelShooterSubsystem.stopSpin
        Gamepads.gamepad1.x.toggleOnBecomesTrue() whenBecomesTrue FlywheelShooterSubsystem.spin(.25) whenBecomesFalse FlywheelShooterSubsystem.stopSpin
    }
}