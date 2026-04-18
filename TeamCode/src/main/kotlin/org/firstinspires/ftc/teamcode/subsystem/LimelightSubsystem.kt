package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.ActiveOpMode

object LimelightSubsystem : Subsystem {

    private lateinit var limelight: Limelight3A

    /**
     * Returns the latest result from the Limelight.
     */
    val latestResult: LLResult?
        get() = limelight.latestResult

    /**
     * Returns the horizontal offset from the crosshair (tx) in degrees.
     */
    val horizontalOffset get() = latestResult?.tx ?: 0.0

    /**
     * Checks if the Limelight currently sees a valid target.
     */
    val hasTarget: Boolean
        get() = limelight.latestResult?.isValid ?: false

    override fun initialize() {
        limelight = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.pipelineSwitch(0)
        limelight.start()
    }

    override fun periodic() {
        val orientation = Math.toDegrees(PedroComponent.follower.heading)
        limelight.updateRobotOrientation(orientation)

        ActiveOpMode.telemetry.addData(
            "Pinpoint Orientation",
            Math.toDegrees(PedroComponent.follower.heading)
        )

        val llResult = limelight.latestResult
        if (llResult != null && llResult.isValid()) {
            ActiveOpMode.telemetry.addData("Tx", llResult.tx)
            ActiveOpMode.telemetry.addData("Ty", llResult.ty)
            ActiveOpMode.telemetry.addData("Ta", llResult.ta)
        }
    }
}