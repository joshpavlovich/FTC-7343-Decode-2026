package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystem.ColorSensorSubsystem

@TeleOp(name = "Team Sapphire: TeleOp Color Sensor Tester")
class TeleOpColorSensorTester : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(ColorSensorSubsystem),
            BulkReadComponent,
            BindingsComponent
        )
    }
}