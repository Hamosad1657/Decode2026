package com.hamosad.lib.components.motors

import com.hamosad.lib.math.Amps
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.PIDController
import com.hamosad.lib.math.Rotation2d
import com.hamosad.lib.math.Volts
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

enum class DCMotorStopMode {
    COAST,
    BRAKE,
}

enum class MotorType(val ticksPerRotation: Double) {
    GO_BUILDA5202(537.7),
    REV_THROUGH_BORE_ENCODER(8192.0),
}


class HaMotor(name: String, hardwareMap: HardwareMap, val type: MotorType) {
    // Property declarations
    private val motor = hardwareMap.get(DcMotorEx::class.java, name)

    init {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    var direction: DcMotorSimple.Direction
        get() = motor.direction
        set(value) {
            motor.direction = value
        }

    val currentVelocity: AngularVelocity get() = AngularVelocity.fromRPS(motor.velocity / type.ticksPerRotation)
    val currentPosition: Rotation2d get() = Rotation2d.fromRotations(motor.currentPosition / type.ticksPerRotation)
    val motorCurrent: Amps get() = motor.getCurrent(CurrentUnit.AMPS)

    // Basic motor control
    fun setVoltage(voltage: Volts) {
        motor.power = voltage / 12.0
    }

    fun isCurrentOver(current: Amps): Boolean {
        return current < motorCurrent
    }

    fun stopMotor() {
        motor.power = 0.0
    }

    fun resetEncoder() {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun setStopMode(dcMotorStopMode: DCMotorStopMode) {
        when (dcMotorStopMode) {
            DCMotorStopMode.COAST -> motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            DCMotorStopMode.BRAKE -> motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }
}