package org.firstinspires.ftc.teamcode.subsystems.mecanum

import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.Length
import com.hamosad.lib.math.MPS
import com.hamosad.lib.math.PIDGains
import com.hamosad.lib.math.Rotation2d
import com.hamosad.lib.math.Rotation3d
import com.hamosad.lib.math.Translation3d
import com.hamosad.lib.vision.AprilTagsStdDevs
import com.hamosad.lib.vision.RobotPoseStdDevs
import kotlin.math.PI

object MecanumConstants {
    val MAX_WHEEL_SPEED: AngularVelocity = AngularVelocity.fromRPM(312.0)
    val WHEEL_RADIUS: Length = Length.fromMillimeters(52.0)
    val CHASSIS_RADIUS: Length = Length.fromMillimeters(52.0)

    val MAX_CHASSIS_SPEED: MPS = MAX_WHEEL_SPEED.asRPS * WHEEL_RADIUS.asMeters * 2 * PI

    val MAX_CHASSIS_ANGULAR_VELOCITY: AngularVelocity = AngularVelocity.fromRPS(6.0)

    val wheelGains: PIDGains = PIDGains(
        p = 3.0,
        i = 2.0,
        d = 0.0,
    )

    //Vision
    val MAX_TRUST_RANGE: Length = Length.fromMeters(5.0)
    val CAMERA_POSITION: Translation3d = Translation3d(0.0,0.0,0.5)
    val CAMERA_ROTATION: Rotation3d = Rotation3d(
        Rotation2d.fromDegrees(0.0),
        Rotation2d.fromDegrees(0.0),
        Rotation2d.fromDegrees(0.0)
    )

    val APRIL_TAG_STD_DEVS: AprilTagsStdDevs = AprilTagsStdDevs(
        RobotPoseStdDevs(0.0, 0.0, 0.0),
        RobotPoseStdDevs(0.0, 0.0, 0.0)
    )
}