package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.hamosad.lib.math.PIDGains
import com.qualcomm.robotcore.hardware.DcMotorSimple

object ShooterConstants {
    val WHEEL_GAINS = PIDGains(
        0.0,
        0.0,
        0.0
    )

    const val TRANSMISSION_RATIO: Double = 0.0
    val LEFT_MOTOR_DIRECTION: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD
    val RIGHT_MOTOR_DIRECTION: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD

}