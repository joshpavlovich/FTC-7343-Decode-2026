package org.firstinspires.ftc.teamcode.subsystem

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower

object FlywheelShooterSubsystem : Subsystem {

    private val flyWheelMotor = MotorEx("flywheel_motor")

    fun spin(power: Double = 1.0) = SetPower(flyWheelMotor, power)
        .requires(this)

    val stopSpin = SetPower(flyWheelMotor, 0.0)
        .requires(this)
}