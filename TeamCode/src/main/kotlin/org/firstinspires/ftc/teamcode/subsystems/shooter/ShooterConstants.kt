package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.hamosad.lib.math.Amps
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.Length
import com.hamosad.lib.math.PIDGains
import com.hamosad.lib.math.HaRotation2d
import com.hamosad.lib.math.Volts
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo

data class ShooterState(public val angle: HaRotation2d, public val speedVoltage: Volts)

object ShooterConstants {
    val HOOD_ANGLE_GAINS = PIDGains(
        0.0191,
        0.0,
        0.0
    )


    val LEFT_MOTOR_NAME = "LM Shooter"
    val RIGHT_MOTOR_NAME = "RM Shooter"
    val SERVO_NAME = "Shooter Servo"

    const val HOOD_ANGLE_TRANSMISSION_RATIO: Double = 0.2245

    val LEFT_MOTOR_DIRECTION: DcMotorSimple.Direction = DcMotorSimple.Direction.REVERSE
    val RIGHT_MOTOR_DIRECTION: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD
    val SERVO_DIRECTION = DcMotorSimple.Direction.FORWARD

    val MAX_HOOD_ANGLE: HaRotation2d = HaRotation2d.fromDegrees(45.0)
    val MIN_HOOD_ANGLE: HaRotation2d = HaRotation2d.fromDegrees(0.0)

    const val CURRENT_THRESHOLD: Amps = 0.0
    val VELOCITY_TOLERANCE: AngularVelocity = AngularVelocity.fromRPM(0.0)
    val ANGLE_TOLERANCE: HaRotation2d = HaRotation2d.fromDegrees(5.0)

    // KINEMATICS
    val DISTANCE_TO_ANGLE_TABLE: Map<Length, HaRotation2d> = mapOf(
        Pair(Length.fromMeters(0.0), HaRotation2d.fromDegrees(0.0))
    )

    val DISTANCE_TO_VELOCITY_TABLE: Map<Length, AngularVelocity> = mapOf(
        Pair(Length.fromMeters(0.0), AngularVelocity.fromRPM(0.0))
    )

    //Shooter States
    val ShooterState1: ShooterState = ShooterState(HaRotation2d.fromDegrees(20.0), 10.0)
    val ShooterState2: ShooterState = ShooterState(HaRotation2d.fromDegrees(0.0), 0.0)
    val ShooterState3: ShooterState = ShooterState(HaRotation2d.fromDegrees(0.0), 0.0)
    val ShooterState4: ShooterState = ShooterState(HaRotation2d.fromDegrees(0.0), 0.0)
}