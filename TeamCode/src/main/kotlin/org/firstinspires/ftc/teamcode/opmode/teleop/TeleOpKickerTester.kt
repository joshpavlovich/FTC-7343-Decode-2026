package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystem.KickerSubsystem.addShooterDetails
import org.firstinspires.ftc.teamcode.subsystem.KickerSubsystem

@TeleOp(name = "TeleOp Kicker Tester")
class TeleOpKickerTester : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(KickerSubsystem),
            BulkReadComponent,
            BindingsComponent
        )

        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
    }

    override fun onStartButtonPressed() {
        Gamepads.gamepad1.rightTrigger.atLeast(0.3) whenBecomesTrue KickerSubsystem.kickArtifact() whenBecomesFalse KickerSubsystem.resetKickerServo()
    }

    override fun onUpdate() {
        ActiveOpMode.telemetry.addShooterDetails()

        ActiveOpMode.telemetry.update()
    }
}