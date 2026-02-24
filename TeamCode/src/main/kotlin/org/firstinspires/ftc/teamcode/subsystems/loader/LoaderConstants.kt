package org.firstinspires.ftc.teamcode.subsystems.loader

import com.hamosad.lib.components.sensors.Color
import com.hamosad.lib.math.PIDGains
import com.hamosad.lib.math.HaRotation2d
import com.hamosad.lib.math.Volts
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.commands.Ball

object LoaderConstants {
    // NAMES
    const val ROULETTE_SERVO_NAME = "Loader spin servo"
    const val ROULETTE_SERVO2_NAME = "Loader spin 2 servo"
    const val COLOR_SENSOR_NAME = "Loader color sensor"
    const val ARM_MOTOR_NAME = "Loader arm motor"

    // COMPONENT CONFIGS

    // COUNTER CLOCKWISE POSITIVE!
    val ROULETTE_SERVO_DIRECTION = DcMotorSimple.Direction.REVERSE
    val ROULETTE_SERVO2_DIRECTION = DcMotorSimple.Direction.REVERSE
    val ARM_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE
    val LOAD_TO_SHOOTER_VOLTAGE: Volts = 12.0

    // PID GAINS
    val ROULETTE_ANGLE_GAINS = PIDGains(
        1.23,
        0.0,
        0.07,
    )

    const val ROULETTE_SPIN_VOLTAGE: Volts = 6.0

    // ROULETTE VALUES (Counter clockwise increasing)
    val BALL_1_AT_SHOOTER = HaRotation2d.fromDegrees(0.0)
    val BALL_1_AT_INTAKE = BALL_1_AT_SHOOTER + HaRotation2d.fromDegrees(90.0)
    val BALL_1_AT_COLOR_SENSOR = BALL_1_AT_SHOOTER - HaRotation2d.fromDegrees(45.0)
    val BALL_1_FURTHEST_FROM_SHOOTER = BALL_1_AT_SHOOTER + HaRotation2d.fromDegrees(180.0)
    val BALL_1_FURTHEST_FROM_INTAKE = BALL_1_AT_INTAKE + HaRotation2d.fromDegrees(180.0)

    val BALL_2_AT_SHOOTER = HaRotation2d.fromDegrees(120.0)
    val BALL_2_AT_INTAKE = BALL_2_AT_SHOOTER + HaRotation2d.fromDegrees(90.0)
    val BALL_2_AT_COLOR_SENSOR = BALL_2_AT_SHOOTER - HaRotation2d.fromDegrees(45.0)
    val BALL_2_FURTHEST_FROM_SHOOTER = BALL_2_AT_SHOOTER + HaRotation2d.fromDegrees(180.0)
    val BALL_2_FURTHEST_FROM_INTAKE = BALL_2_AT_INTAKE + HaRotation2d.fromDegrees(180.0)


    val BALL_3_AT_SHOOTER = HaRotation2d.fromDegrees(240.0)
    val BALL_3_AT_INTAKE = BALL_3_AT_SHOOTER + HaRotation2d.fromDegrees(90.0)
    val BALL_3_AT_COLOR_SENSOR = BALL_3_AT_SHOOTER - HaRotation2d.fromDegrees(45.0)
    val BALL_3_FURTHEST_FROM_SHOOTER = BALL_3_AT_SHOOTER + HaRotation2d.fromDegrees(180.0)
    val BALL_3_FURTHEST_FROM_INTAKE = BALL_3_AT_INTAKE + HaRotation2d.fromDegrees(180.0)

    val BALL_LOCATION_TOLERANCE = HaRotation2d.fromDegrees(60.0)

    val ROULETTE_TOLERANCE = HaRotation2d.fromDegrees(4.0)

    val COLOR_SENSOR_ANGLE_TOLERANCE = HaRotation2d.fromDegrees(4.0)
    const val COLOR_SENSOR_TOLERANCE: Int = 30
    val GREEN_COLOR = Color(0, 255, 0)
    val PURPLE_COLOR = Color(255, 0, 255)

    // ARM VALUES
    val RETRACTED_ARM_ANGLE = HaRotation2d.fromDegrees(0.0)
    val OPEN_ARM_ANGLE = HaRotation2d.fromDegrees(180.0)

    const val ARM_MOTOR_VOLTAGE: Volts = 6.0
}

enum class BallColor {
    PURPLE,
    GREEN,
    UNKNOWN,
}
enum class ColorPattern(val pattern: Array<BallColor>, val id: Int) {
    PPG(arrayOf(BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN), 23),
    PGP(arrayOf(BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE), 22),
    GPP(arrayOf(BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE), 21),
    UNKNOWN(arrayOf(BallColor.UNKNOWN, BallColor.UNKNOWN, BallColor.UNKNOWN), 0),
}
class BallPattern(private val firstBall: Ball, private val secondBall: Ball, private val thirdBall: Ball) {
    val pattern: Array<Ball> = arrayOf(firstBall, secondBall, thirdBall)
}