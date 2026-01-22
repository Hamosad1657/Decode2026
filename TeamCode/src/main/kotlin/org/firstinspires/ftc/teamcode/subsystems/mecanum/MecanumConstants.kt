package org.firstinspires.ftc.teamcode.subsystems.mecanum

import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.Length
import com.hamosad.lib.math.MPS
import com.hamosad.lib.math.PIDGains
import com.hamosad.lib.math.HaRotation2d
import com.hamosad.lib.math.HaTranslation2d
import com.hamosad.lib.math.Rotation3d
import com.hamosad.lib.math.Translation3d
import com.hamosad.lib.vision.AprilTagsStdDevs
import com.hamosad.lib.vision.RobotPoseStdDevs
import kotlin.math.PI

object MecanumConstants {
    const val MAX_CHASSIS_SPEED: MPS = 1.6
    val WHEEL_RADIUS: Length = Length.fromMillimeters(52.0)

    val CHASSIS_DIMENSIONS = HaTranslation2d(16.5, 20.75)

    val WHEEL_CIRCUMFERENCE: Length = WHEEL_RADIUS * 2 * PI

    val MAX_WHEEL_SPEED: AngularVelocity = AngularVelocity.fromRPS(MAX_CHASSIS_SPEED / WHEEL_CIRCUMFERENCE.asMeters)

    val MAX_CHASSIS_ANGULAR_VELOCITY: AngularVelocity = AngularVelocity.fromRPS(MAX_CHASSIS_SPEED / (CHASSIS_DIMENSIONS.length * 2 * PI))

    val WHEEL_GAINS: PIDGains = PIDGains(
        p = 6.0,
        i = 2.0,
        d = 0.0,
        f = 0.0,
    )

    val INVERT_YAW_FOLLOWING = false
    // Uses radians
    val YAW_PID_GAINS: PIDGains = PIDGains(
        p = 0.0,
        i = 0.0,
        d = 0.0,
        f = 0.0,
    )

    // -- Pure pursuit --
    val FOLLOW_RADIUS: Length = Length.fromInches(CHASSIS_DIMENSIONS.y + 3)
    val POSITION_BUFFER: Double = 0.1
    val ANGLE_BUFFER: Double = 0.1


    // -- Vision --
    val MAX_TRUST_RANGE: Length = Length.fromMeters(5.0)
    val CAMERA_POSITION: Translation3d = Translation3d(0.0,0.0,0.5)
    val CAMERA_ROTATION: Rotation3d = Rotation3d(
        HaRotation2d.fromDegrees(0.0),
        HaRotation2d.fromDegrees(0.0),
        HaRotation2d.fromDegrees(0.0)
    )

    val APRIL_TAG_STD_DEVS: AprilTagsStdDevs = AprilTagsStdDevs(
        RobotPoseStdDevs(0.0, 0.0, 0.0),
        RobotPoseStdDevs(0.0, 0.0, 0.0)
    )
}