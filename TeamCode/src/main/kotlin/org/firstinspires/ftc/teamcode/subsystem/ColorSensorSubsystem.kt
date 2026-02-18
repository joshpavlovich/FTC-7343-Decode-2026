package org.firstinspires.ftc.teamcode.subsystem

import android.graphics.Color
import com.pedropathing.util.Timer
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern
import com.qualcomm.hardware.rev.RevColorSensorV3
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.subsystem.FlywheelShooterSubsystem.autoStopTransfer

/** The timestamp (in seconds) when the end game period starts. */
const val END_GAME_START_TIME_SECONDS = 100.0

private const val DISTANCE_THRESHOLD_CENTIMETER = 5.0

/**
 * ColorSensorSubsystem manages the color sensor and the Blinkin LED driver.
 * It detects the color of artifacts in proximity and updates the LED patterns
 * to provide visual feedback to the drivers. It also triggers automated actions
 * like stopping the transfer mechanism if no color is detected for a period.
 */
object ColorSensorSubsystem : Subsystem {

    private val colorDetectionTimer by lazy { Timer() }

    private lateinit var blinkinLedDriver: RevBlinkinLedDriver
    private lateinit var colorSensor: RevColorSensorV3

    private var hsv = FloatArray(3)

    private val greenRange = 155.0..160.0
    private val purpleRange = 180.0..240.0

    /**
     * Initializes the color sensor and LED driver from the hardware map.
     */
    override fun initialize() {
        colorSensor = ActiveOpMode.hardwareMap.get(RevColorSensorV3::class.java, "color_sensor")
        blinkinLedDriver = ActiveOpMode.hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkin")
        colorSensor.enableLed(true)
    }

    /**
     * Continuously reads color data, determines the detected artifact color,
     * updates LED patterns, and manages automated transfer stopping logic.
     */
    override fun periodic() {
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv)

        val pattern = when {
            ActiveOpMode.opModeIsActive && ActiveOpMode.runtime > END_GAME_START_TIME_SECONDS -> BlinkinPattern.FIRE_MEDIUM

            colorSensor.getDistance(DistanceUnit.CM) <= DISTANCE_THRESHOLD_CENTIMETER -> {
                val hue = hsv[0]
                val currentColorStatus: ColorStatus = when (hue) {
                    in greenRange -> ColorStatus.GREEN
                    in purpleRange -> ColorStatus.PURPLE
                    else -> ColorStatus.UNKNOWN
                }

                ActiveOpMode.telemetry.addData("Current color status", currentColorStatus.name)

                if (currentColorStatus != ColorStatus.UNKNOWN) {
                    colorDetectionTimer.resetTimer()
                }

                when (currentColorStatus) {
                    ColorStatus.GREEN -> BlinkinPattern.GREEN
                    ColorStatus.PURPLE -> BlinkinPattern.VIOLET
                    ColorStatus.UNKNOWN -> BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE
                }
            }

            else -> {
                if (colorDetectionTimer.elapsedTimeSeconds > 3.0) {
                    autoStopTransfer()
                }

                BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE
            }
        }

        blinkinLedDriver.setPattern(pattern)

        ActiveOpMode.telemetry.addData(
            "Raw distance sensor",
            colorSensor.getDistance(DistanceUnit.CM)
        )
    }

    /**
     * Represents the possible color states detected by the sensor.
     */
    enum class ColorStatus {
        GREEN,
        PURPLE,
        UNKNOWN
    }
}
