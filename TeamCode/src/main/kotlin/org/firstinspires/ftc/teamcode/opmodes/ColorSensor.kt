package org.firstinspires.ftc.teamcode.opmodes

import android.graphics.Color
import com.qualcomm.hardware.rev.RevColorSensorV3
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

object ColorSensorSubsystem : Subsystem {

    private lateinit var colorSensor: RevColorSensorV3

    var distanceThreshold = 3 // CM

    private var hsv = FloatArray(3)

    override fun initialize() {
        colorSensor = ActiveOpMode.hardwareMap.get(RevColorSensorV3::class.java, "color_sensor")
        colorSensor.enableLed(true)
    }

    override fun periodic() {
        val currentColorStatus = if (colorSensor.getDistance(DistanceUnit.CM) <= distanceThreshold) {
            Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv)

            if (hsv[0] <= 160) {
                ColorStatus.GREEN
            } else {
                ColorStatus.PURPLE
            }
        } else {
            ColorStatus.EMPTY
        }

        ActiveOpMode.telemetry.addData("Current color status", currentColorStatus.name)
        ActiveOpMode.telemetry.addData(
            "Raw distance sensor",
            colorSensor.getDistance(DistanceUnit.CM)
        )
        ActiveOpMode.telemetry.addData("Raw hsv hue", hsv[0])
    }

    enum class ColorStatus {
        EMPTY,
        GREEN,
        PURPLE
    }
}