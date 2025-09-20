package org.firstinspires.ftc.teamcode.guide.kotlin.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent

@TeleOp(name = "Team Sapphire: Autonomous")
class Autonomous : NextFTCOpMode() {
    init {
        addComponents(
//            SubsystemComponent(Lift, Claw),
            BulkReadComponent
        )
    }

    private val autonomousRoutine: Command
        get() = SequentialGroup(
//            Lift.toHigh,
//            ParallelGroup(
//                Lift.toMiddle,
//                Claw.close
//            ),
//            Delay(0.5.seconds),
//            ParallelGroup(
//                Claw.open,
//                Lift.toLow
//            )
        )

    override fun onStartButtonPressed() {
        autonomousRoutine()
    }
}