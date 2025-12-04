package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.hamosad.lib.math.PIDGains
import com.hamosad.lib.math.Rotation2d
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo

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
    const val HOOD_ANGLE_TRANSMISSION_RATIO: Double = 0.0
    val LEFT_MOTOR_DIRECTION: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD
    val RIGHT_MOTOR_DIRECTION: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD
    val SERVO_DIRECTION = Servo.Direction.FORWARD

    val MAX_HOOD_ANGLE: Rotation2d = Rotation2d.fromDegrees(0.0)
    val MIN_HOOD_ANGLE: Rotation2d = Rotation2d.fromDegrees(0.0)

}