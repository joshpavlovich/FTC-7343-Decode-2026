package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.Direction
import dev.nextftc.hardware.impl.IMUEx

object LimelightSubsystem : Subsystem {
    private lateinit var limelight: Limelight3A
    private lateinit var imu: IMUEx

    override fun initialize() {
        limelight = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.pipelineSwitch(0)

        imu = IMUEx("imu", Direction.RIGHT, Direction.UP).zeroed()

        limelight.start()
    }

    val tx: Double
        get() = limelight.latestResult?.tx ?: 0.0

    val hasTarget: Boolean
        get() = limelight.latestResult?.isValid ?: false

    override fun periodic() {
        // Telemetry can be added here
        val orientation = imu.get().inDeg
        limelight.updateRobotOrientation(orientation)

        val llResult = limelight.latestResult
        if (llResult != null && llResult.isValid()) {
            val botPose = llResult.botpose
            ActiveOpMode.telemetry.addData("Tx", llResult.tx)
            ActiveOpMode.telemetry.addData("Ty", llResult.ty)
            ActiveOpMode.telemetry.addData("Ta", llResult.ta)
        }
    }

}