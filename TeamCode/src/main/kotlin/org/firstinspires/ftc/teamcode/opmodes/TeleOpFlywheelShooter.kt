package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent

@TeleOp(name = "Team Sapphire: TeleOp Flywheel Shooter")
class TeleOpFlywheelShooter : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(FlywheelShooter),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onStartButtonPressed() {
        Gamepads.gamepad1.circle.toggleOnBecomesTrue() whenBecomesTrue FlywheelShooter.spin whenBecomesFalse FlywheelShooter.stopSpin
    }
}