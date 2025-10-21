package org.firstinspires.ftc.teamcode.subsystem

import android.graphics.Color
import com.qualcomm.hardware.rev.RevColorSensorV3
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

object ColorSensorSubsystem : Subsystem {

    private lateinit var colorSensor: RevColorSensorV3

    private var hsv = FloatArray(3)

    private val greenRange = 155.0..160.0

    private val purpleRange = 180.0..240.0

    override fun initialize() {
        colorSensor = ActiveOpMode.hardwareMap.get(RevColorSensorV3::class.java, "color_sensor")
        colorSensor.enableLed(true)
    }

    override fun periodic() {
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv)

        val hue = hsv[0]
        val currentColorStatus = when (hue) {
            in greenRange -> ColorStatus.GREEN
            in purpleRange -> ColorStatus.PURPLE
            else -> ColorStatus.UNKNOWN
        }

        ActiveOpMode.telemetry.addData(
            "Raw distance sensor",
            colorSensor.getDistance(DistanceUnit.CM)
        )
        ActiveOpMode.telemetry.addData("Raw hsv hue", hue)
        ActiveOpMode.telemetry.addData("Current color status", currentColorStatus.name)
        ActiveOpMode.telemetry.update()
    }

    enum class ColorStatus {
        GREEN,
        PURPLE,
        UNKNOWN
    }
}