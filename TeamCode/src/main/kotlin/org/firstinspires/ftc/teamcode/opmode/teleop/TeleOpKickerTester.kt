package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystem.KickerSubsystem

/**
 * A testing OpMode for the [KickerSubsystem].
 * Provides manual control over the kicker servo to verify its range of motion and responsiveness.
 */
@Disabled
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

    /**
     * Binds the gamepad 1 right trigger to the kicker actions.
     * When the trigger is pressed past 0.3, the artifact is kicked.
     * When released, the kicker servo resets.
     */
    override fun onStartButtonPressed() {
        Gamepads.gamepad1.rightTrigger.atLeast(0.3) whenBecomesTrue KickerSubsystem.kickArtifact() whenBecomesFalse KickerSubsystem.resetKickerServo()
    }

    /**
     * Updates telemetry data on each loop iteration.
     */
    override fun onUpdate() {
        ActiveOpMode.telemetry.update()
    }
}
