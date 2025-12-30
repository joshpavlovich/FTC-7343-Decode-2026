package org.firstinspires.ftc.teamcode.subsystem

import android.graphics.Color
import com.pedropathing.util.Timer
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern
import com.qualcomm.hardware.rev.RevColorSensorV3
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem.autoTransfer

private const val DISTANCE_THRESHOLD_CENTIMETER = 5.0

object ColorSensorSubsystem : Subsystem {

    private val colorDetectionTimer by lazy { Timer() }

    private lateinit var blinkinLedDriver: RevBlinkinLedDriver
    private lateinit var colorSensor: RevColorSensorV3

    private var hsv = FloatArray(3)

    private val greenRange = 155.0..160.0
    private val purpleRange = 180.0..240.0

    override fun initialize() {
        colorSensor = ActiveOpMode.hardwareMap.get(RevColorSensorV3::class.java, "color_sensor")
        blinkinLedDriver = ActiveOpMode.hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkin")
        colorSensor.enableLed(true)
    }

    override fun periodic() {
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv)

        val pattern =
            if (colorSensor.getDistance(DistanceUnit.CM) <= DISTANCE_THRESHOLD_CENTIMETER) {
                val hue = hsv[0]
                val currentColorStatus: ColorStatus = when (hue) {
                    in greenRange -> ColorStatus.GREEN
                    in purpleRange -> ColorStatus.PURPLE
                    else -> ColorStatus.UNKNOWN
                }

                ActiveOpMode.telemetry.addData("Raw hsv hue", hue)
                ActiveOpMode.telemetry.addData("Current color status", currentColorStatus.name)

                if (currentColorStatus != ColorStatus.UNKNOWN) {
                    colorDetectionTimer.resetTimer()
                }

                when (currentColorStatus) {
                    ColorStatus.GREEN -> BlinkinPattern.GREEN
                    ColorStatus.PURPLE -> BlinkinPattern.VIOLET
                    ColorStatus.UNKNOWN -> BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE
                }
            } else {
                if (colorDetectionTimer.elapsedTimeSeconds > 3.0) {
                    autoTransfer()
                }
                BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE
            }

        blinkinLedDriver.setPattern(pattern)

        ActiveOpMode.telemetry.addData(
            "Raw distance sensor",
            colorSensor.getDistance(DistanceUnit.CM)
        )
    }

    enum class ColorStatus {
        GREEN,
        PURPLE,
        UNKNOWN
    }
}