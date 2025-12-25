package org.firstinspires.ftc.teamcode.autonomous.paths

import com.hamosad.lib.math.Rotation2d

enum class PathPoints(x: Double, y: Double, angle: Rotation2d) {
    START_CLOSE_TO_GATE(21.5, 122.5, Rotation2d.fromDegrees(54.0)),
    SHOOT_CLOSE_TO_GATE(30.0, 112.0, Rotation2d.fromDegrees(45.0)),
    START_FAR_FROM_GATE(50.0, 9.0, Rotation2d.fromDegrees(90.0)),
    SHOOT_FAR_FROM_GATE(60.0, 22.0, Rotation2d.fromDegrees(115.0)),
    START_COLLECTING_CLOSE_TO_GATE(41.5, 84.0, Rotation2d.fromDegrees(180.0)),
    FINISH_COLLECTING_CLOSE_TO_GATE(18.0, 84.0, Rotation2d.fromDegrees(180.0)),
    START_COLLECTING_FAR_FROM_GATE(41.5, 36.0, Rotation2d.fromDegrees(180.0)),
    FINISH_COLLECTING_FAR_FROM_GATE(18.0, 36.0, Rotation2d.fromDegrees(180.0))
}