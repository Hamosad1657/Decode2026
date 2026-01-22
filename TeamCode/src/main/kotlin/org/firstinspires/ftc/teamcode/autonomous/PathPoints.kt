package org.firstinspires.ftc.teamcode.autonomous

import com.hamosad.lib.math.HaPose2d
import com.hamosad.lib.math.HaRotation2d
import com.hamosad.lib.math.HaTranslation2d

enum class PathPoints(public val pose: HaPose2d) {
    START_CLOSE(HaPose2d(HaTranslation2d(21.5, 122.5), HaRotation2d.Companion.fromDegrees(54.0))),
    START_FAR(HaPose2d(HaTranslation2d(50.0, 9.0), HaRotation2d.Companion.fromDegrees(90.0))),
    SHOOT_CLOSE(HaPose2d(HaTranslation2d(30.0, 112.0), HaRotation2d.Companion.fromDegrees(45.0))),
    SHOOT_FAR(HaPose2d(HaTranslation2d(60.0, 22.0), HaRotation2d.Companion.fromDegrees(115.0))),
    CLOSE_ENDPOINT(HaPose2d(HaTranslation2d(23.0, 103.0), HaRotation2d.Companion.fromDegrees(45.0))),
    FAR_ENDPOINT(HaPose2d(HaTranslation2d(55.0, 25.0), HaRotation2d.Companion.fromDegrees(25.0))),
    START_COLLECTING_CLOSE(HaPose2d(HaTranslation2d(41.5, 84.0), HaRotation2d.Companion.fromDegrees(180.0))),
    FINISH_COLLECTING_CLOSE(HaPose2d(HaTranslation2d(18.0, 84.0), HaRotation2d.Companion.fromDegrees(180.0))),
    START_COLLECTING_FAR(HaPose2d(HaTranslation2d(41.5, 36.0), HaRotation2d.Companion.fromDegrees(180.0))),
    FINISH_COLLECTING_FAR(HaPose2d(HaTranslation2d(18.0, 36.0), HaRotation2d.Companion.fromDegrees(180.0))),
    START_COLLECTING_MIDDLE(HaPose2d(HaTranslation2d(41.5, 60.0), HaRotation2d.Companion.fromDegrees(180.0))),
    FINISH_COLLECTING_MIDDLE(HaPose2d(HaTranslation2d(18.0, 60.0), HaRotation2d.Companion.fromDegrees(180.0)))
}