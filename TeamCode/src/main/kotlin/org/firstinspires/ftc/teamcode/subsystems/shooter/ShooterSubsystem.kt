package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.components.motors.HaMotor
import com.hamosad.lib.components.motors.MotorType
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.PIDController
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

object ShooterSubsystem: Subsystem() {

    val pidController = PIDController(ShooterConstants.WHEEL_GAINS)
    val rightMotor: HaMotor? = null
    val leftMotor: HaMotor? = null

    override fun init(newHardwareMap: HardwareMap) {
        super.init(newHardwareMap)
        val rightMotor = HaMotor("Right Shooter Motor", hardwareMap!!, MotorType.GO_BUILDA5202)
        val leftMotor = HaMotor("Left Shooter Motor", hardwareMap!!, MotorType.GO_BUILDA5202)
        leftMotor.setDirection(ShooterConstants.LEFT_MOTOR_DIRECTION)
        rightMotor.setDirection(ShooterConstants.RIGHT_MOTOR_DIRECTION)
    }

    val currentMotorVelocity: AngularVelocity get() = rightMotor?.currentVelocity ?: AngularVelocity.fromRPM(0.0)
    val currentWheelVelocity: AngularVelocity get() = rightMotor?.currentVelocity?.times(
        ShooterConstants.TRANSMISSION_RATIO) ?: AngularVelocity.fromRPM(0.0)

    fun setWheelVelocity(desiredVelocity: AngularVelocity) {
        rightMotor?.setVoltage(pidController.calculate(currentWheelVelocity.asRPS, desiredVelocity.asRPS / ShooterConstants.TRANSMISSION_RATIO))
        leftMotor?.setVoltage(pidController.calculate(currentWheelVelocity.asRPS, desiredVelocity.asRPS / ShooterConstants.TRANSMISSION_RATIO))
    }

    fun stopWheel() {
        rightMotor?.stopMotor()
        leftMotor?.stopMotor()
    }

    override fun periodic() {

    }

    override fun updateTelemetry(telemetry: Telemetry, dashboardPacket: TelemetryPacket) {

    }
}