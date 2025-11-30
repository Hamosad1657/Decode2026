package org.firstinspires.ftc.teamcode.subsystems.intake

import com.hamosad.lib.math.Volts
import com.qualcomm.robotcore.hardware.DcMotorSimple

object IntakeConstants {
    // Motor
    const val MOTOR_NAME = "Intake"
    const val INTAKING_VOLTAGE: Volts = 12.0
    val DIRECTION = DcMotorSimple.Direction.FORWARD
}