package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

object IntakeSubsystem : Subsystem {

    // TODO: NOT SURE OF THE INITIAL MOTOR DIRECTION BASED ON HOW IT IS MOUNTED
    private val intakeMotor = MotorEx("intake_motor").reversed()

    override fun initialize() {
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    val forward = InstantCommand { intakeMotor.power = 1.0 }.requires(this)

    // TODO: SHOULD REVERSE BE AT -1.0 POWER (FULL) OR -0.5 POWER (HALF)?
    val reverse = InstantCommand { intakeMotor.power = -1.0 }.requires(this)
    val stop = InstantCommand { intakeMotor.power = 0.0 }.requires(this)
}