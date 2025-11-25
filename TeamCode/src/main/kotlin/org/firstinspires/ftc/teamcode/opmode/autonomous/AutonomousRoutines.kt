package org.firstinspires.ftc.teamcode.opmode.autonomous

import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.extensions.pedro.FollowPath

object AutonomousRoutines {

    val farParkAutoRoutine
        get() = SequentialGroup(FollowPath(PathManager.farLaunchZoneStartToPark, true))
}