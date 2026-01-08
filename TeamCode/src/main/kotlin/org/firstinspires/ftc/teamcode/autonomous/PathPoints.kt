package org.firstinspires.ftc.teamcode.autonomous

import com.hamosad.lib.math.HaRotation2d

enum class PathPoints(x: Double, y: Double, angle: HaRotation2d) {
    START_CLOSE(21.5, 122.5, HaRotation2d.Companion.fromDegrees(54.0)),
    START_FAR(50.0, 9.0, HaRotation2d.Companion.fromDegrees(90.0)),
    SHOOT_CLOSE(30.0, 112.0, HaRotation2d.Companion.fromDegrees(45.0)),
    SHOOT_FAR(60.0, 22.0, HaRotation2d.Companion.fromDegrees(115.0)),
    CLOSE_ENDPOINT(23.0, 103.0, HaRotation2d.Companion.fromDegrees(45.0)),
    FAR_ENDPOINT(55.0, 25.0, HaRotation2d.Companion.fromDegrees(25.0)),
    START_COLLECTING_CLOSE(41.5, 84.0, HaRotation2d.Companion.fromDegrees(180.0)),
    FINISH_COLLECTING_CLOSE(18.0, 84.0, HaRotation2d.Companion.fromDegrees(180.0)),
    START_COLLECTING_FAR(41.5, 36.0, HaRotation2d.Companion.fromDegrees(180.0)),
    FINISH_COLLECTING_FAR(18.0, 36.0, HaRotation2d.Companion.fromDegrees(180.0)),
    START_COLLECTING_MIDDLE(41.5, 60.0, HaRotation2d.Companion.fromDegrees(180.0)),
    FINISH_COLLECTING_MIDDLE(20.0, 60.0, HaRotation2d.Companion.fromDegrees(180.0))
}