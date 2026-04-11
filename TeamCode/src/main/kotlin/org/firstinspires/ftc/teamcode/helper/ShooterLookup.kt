package org.firstinspires.ftc.teamcode.helper

import java.util.TreeMap

object ShooterLookup {

    // TreeMap keeps our tuned distances sorted for fast searching
    private val lookupTable by lazy {
        TreeMap(
            // Data points from your manual tuning (17" height, 98mm wheel)
            mapOf(
                38.42 to 2200.0,
                50.93 to 2400.0,
                63.21 to 2600.0,
                75.183 to 2750.0,
                87.53 to 2850.0,
                127.03 to 3200.0,
                138.94 to 3300.0,
                144.71 to 3500.0
            ).toSortedMap()
        )
    }

    fun getTunedVelocity(distance: Double): Double {
        // 1. Check for bounds (Safety first)
        val floor = lookupTable.floorEntry(distance)
        val ceil = lookupTable.ceilingEntry(distance)

        // If distance is less than our closest test (38.42"), use the first entry
        if (floor == null) return lookupTable.firstEntry()?.value ?: 0.0

        // If distance is further than our last test (144.71"), use the last entry
        if (ceil == null) return lookupTable.lastEntry()?.value ?: 0.0

        // 2. If we are exactly on a measured point, return it immediately
        if (floor.key == ceil.key) return floor.value

        // 3. Perform Linear Interpolation between the two closest test points
        val d1 = floor.key
        val v1 = floor.value
        val d2 = ceil.key
        val v2 = ceil.value

        // Calculate the percentage of the way we are between d1 and d2
        val fraction = (distance - d1) / (d2 - d1)

        return v1 + (v2 - v1) * fraction
    }
}