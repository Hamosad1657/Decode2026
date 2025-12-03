package org.firstinspires.ftc.teamcode.subsystems.loader

import com.hamosad.lib.math.PIDGains
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo

object LoaderConstants {
    // NAMES
    const val ROULETTE_SERVO_NAME = "Loader spin servo"
    const val COLOR_SENSOR_NAME = "Loader color sensor"
    const val ARM_SERVO_NAME = "Loader arm servo"
    const val ARM_MOTOR_NAME = "Loader arm motor"

    // COMPONENT CONFIGS
    val ROULETTE_SERVO_DIRECTION = DcMotorSimple.Direction.FORWARD
    val ARM_SERVO_DIRECTION = Servo.Direction.FORWARD
    val ARM_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD

    // PID GAINS
    val ROULETTE_GAINS = PIDGains(
        0.0,
        0.0,
        0.0,
    )
}