package org.firstinspires.ftc.teamcode.opmodes

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower

object FlywheelShooterSubsystem : Subsystem {

    private val flyWheelMotor = MotorEx("flywheel_motor")

    val spin = SetPower(flyWheelMotor, 1.0)
        .requires(this)

    val stopSpin = SetPower(flyWheelMotor, 0.0)
        .requires(this)
}