package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.components.motors.HaCRServoMotor
import com.hamosad.lib.components.motors.HaMotor
import com.hamosad.lib.components.motors.HaServoMotor
import com.hamosad.lib.components.motors.MotorType
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.PIDController
import com.hamosad.lib.math.Rotation2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

object ShooterSubsystem: Subsystem() {

    val speedPIDController = PIDController(ShooterConstants.WHEEL_VELOCITY_GAINS)
    var rightMotor: HaMotor? = null
    var leftMotor: HaMotor? = null
    var servo: HaCRServoMotor? = null
    val hoodAnglePIDController = PIDController(ShooterConstants.HOOD_ANGLE_GAINS)

    override fun init(newHardwareMap: HardwareMap) {
        super.init(newHardwareMap)
        rightMotor = HaMotor(ShooterConstants.RIGHT_MOTOR_NAME, hardwareMap!!, MotorType.GO_BUILDA5202)
        leftMotor = HaMotor(ShooterConstants.LEFT_MOTOR_NAME, hardwareMap!!, MotorType.GO_BUILDA5202)
        servo = HaCRServoMotor(ShooterConstants.SERVO_NAME, hardwareMap!!)
        leftMotor?.direction = ShooterConstants.LEFT_MOTOR_DIRECTION
        rightMotor?.direction = ShooterConstants.RIGHT_MOTOR_DIRECTION
        servo?.direction = ShooterConstants.SERVO_DIRECTION
        leftMotor?.resetEncoder() // shooter must start at min angle
    }

    val currentMotorVelocity: AngularVelocity get() = rightMotor?.currentVelocity ?: AngularVelocity.fromRPM(0.0)
    val currentServoAngle: Rotation2d get() = leftMotor?.currentPosition ?: Rotation2d.fromDegrees(0.0) //we are using the left motor's encoder slot for the servo's encoder
    val currentHoodAngle: Rotation2d get() = ShooterConstants.MIN_HOOD_ANGLE + currentServoAngle * ShooterConstants.HOOD_ANGLE_TRANSMISSION_RATIO
    val isCurrentAboveThreshold: Boolean get() = rightMotor?.isCurrentOver(ShooterConstants.CURRENT_THRESHOLD) ?: false

    val currentShooterVelocity: AngularVelocity get() = rightMotor?.currentVelocity?.times(
        ShooterConstants.SPEED_TRANSMISSION_RATIO) ?: AngularVelocity.fromRPM(0.0)

    fun updateShooterVelocityControl(desiredVelocity: AngularVelocity) {
        rightMotor?.setVoltage(speedPIDController.calculate(currentShooterVelocity.asRPS, desiredVelocity.asRPS / ShooterConstants.SPEED_TRANSMISSION_RATIO))
        leftMotor?.setVoltage(speedPIDController.calculate(currentShooterVelocity.asRPS, desiredVelocity.asRPS / ShooterConstants.SPEED_TRANSMISSION_RATIO))
    }

    fun stopWheel() {
        rightMotor?.stopMotor()
        leftMotor?.stopMotor()
    }

    fun setHoodAngle(desiredAngle: Rotation2d) {
        if (desiredAngle.asDegrees in ShooterConstants.MIN_HOOD_ANGLE.asDegrees..ShooterConstants.MAX_HOOD_ANGLE.asDegrees) {
            servo?.setVoltage(hoodAnglePIDController.calculate(currentHoodAngle.asDegrees, desiredAngle.asDegrees))
        }
    }

    override fun periodic() {
    }

    override fun updateTelemetry(telemetry: Telemetry, dashboardPacket: TelemetryPacket) {

    }
}