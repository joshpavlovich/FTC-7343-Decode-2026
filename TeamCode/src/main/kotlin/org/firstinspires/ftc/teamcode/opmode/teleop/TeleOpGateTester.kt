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
import org.firstinspires.ftc.teamcode.subsystem.GateSubsystem

private const val RIGHT_TRIGGER_MINIMUM_VALUE = 0.5

/**
 * A testing OpMode for the [GateSubsystem].
 * Provides manual control over the gate motor to verify its range of motion and responsiveness.
 */
@Disabled
@TeleOp(name = "TeleOp Gate Tester")
class TeleOpGateTester : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(GateSubsystem),
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
        Gamepads.gamepad1.rightTrigger.atLeast(RIGHT_TRIGGER_MINIMUM_VALUE)
            .whenBecomesTrue(GateSubsystem.open)
            .whenBecomesFalse(GateSubsystem.close)
    }

    /**
     * Updates telemetry data on each loop iteration.
     */
    override fun onUpdate() {
        ActiveOpMode.telemetry.update()
    }
}
