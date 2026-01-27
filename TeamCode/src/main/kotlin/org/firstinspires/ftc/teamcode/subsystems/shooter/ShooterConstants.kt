package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.hamosad.lib.math.Amps
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.Length
import com.hamosad.lib.math.PIDGains
import com.hamosad.lib.math.HaRotation2d
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo

data class ShooterState(public val angle: HaRotation2d, public val speed: AngularVelocity)

object ShooterConstants {
    val WHEEL_VELOCITY_GAINS = PIDGains(
        0.0,
        0.0,
        0.0
    )

    val HOOD_ANGLE_GAINS = PIDGains(
        0.0,
        0.0,
        0.0
    )


    val LEFT_MOTOR_NAME = ""
    val RIGHT_MOTOR_NAME = ""
    val SERVO_NAME = ""
    const val SPEED_TRANSMISSION_RATIO: Double = 0.0
    const val HOOD_ANGLE_TRANSMISSION_RATIO: Double = 0.222
    val LEFT_MOTOR_DIRECTION: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD
    val RIGHT_MOTOR_DIRECTION: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD
    val SERVO_DIRECTION = DcMotorSimple.Direction.FORWARD

    val MAX_HOOD_ANGLE: HaRotation2d = HaRotation2d.fromDegrees(0.0)
    val MIN_HOOD_ANGLE: HaRotation2d = HaRotation2d.fromDegrees(0.0)

    const val CURRENT_THRESHOLD: Amps = 0.0
    val VELOCITY_TOLERANCE: AngularVelocity = AngularVelocity.fromRPM(0.0)
    val ANGLE_TOLERANCE: HaRotation2d = HaRotation2d.fromDegrees(0.0)

    // SIMPLE CONSTANTS
    val SIMPLE_SPEED: AngularVelocity = AngularVelocity.fromRPM(40.0)
    val SIMPLE_ANGLE: HaRotation2d = HaRotation2d.fromDegrees(30.0)

    // KINEMATICS
    val DISTANCE_TO_ANGLE_TABLE: Map<Length, HaRotation2d> = mapOf(
        Pair(Length.fromMeters(0.0), HaRotation2d.fromDegrees(0.0))
    )

    val DISTANCE_TO_VELOCITY_TABLE: Map<Length, AngularVelocity> = mapOf(
        Pair(Length.fromMeters(0.0), AngularVelocity.fromRPM(0.0))
    )
}