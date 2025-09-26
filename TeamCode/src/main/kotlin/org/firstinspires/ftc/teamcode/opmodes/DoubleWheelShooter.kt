package org.firstinspires.ftc.teamcode.opmodes

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower

object DoubleWheelShooter : Subsystem {

    private val leftShooterMotor = MotorEx("left_shooter_motor").reversed()
    private val rightShooterMotor = MotorEx("right_shooter_motor")

    val spin = SetPower(leftShooterMotor, 1.0)
        .and(SetPower(rightShooterMotor, 1.0))
        .requires(this)

    val stopSpin = SetPower(leftShooterMotor, 0.0)
        .and(SetPower(rightShooterMotor, 0.0))
        .requires(this)
}