package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.components.motors.HaCRServoMotor
import com.hamosad.lib.components.motors.HaMotor
import com.hamosad.lib.components.motors.HaServoMotor
import com.hamosad.lib.components.motors.MotorType
import com.hamosad.lib.math.AngularVelocity
import com.arcrobotics.ftclib.controller.PIDFController
import com.hamosad.lib.math.HaRotation2d
import com.hamosad.lib.math.Volts
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

object ShooterSubsystem: Subsystem() {

    val speedPIDController = PIDFController(
        ShooterConstants.WHEEL_VELOCITY_GAINS.p,
        ShooterConstants.WHEEL_VELOCITY_GAINS.i,
        ShooterConstants.WHEEL_VELOCITY_GAINS.d,
        ShooterConstants.WHEEL_VELOCITY_GAINS.f)
    var rightMotor: HaMotor? = null
    var leftMotor: HaMotor? = null
    var servo: HaCRServoMotor? = null
    val hoodAnglePIDController = PIDFController(
        ShooterConstants.HOOD_ANGLE_GAINS.p,
        ShooterConstants.HOOD_ANGLE_GAINS.i,
        ShooterConstants.HOOD_ANGLE_GAINS.d,
        ShooterConstants.HOOD_ANGLE_GAINS.f)

    var desiredVelocity: AngularVelocity = AngularVelocity.fromRPM(0.0)
    var desiredHoodAngle: HaRotation2d = HaRotation2d.fromDegrees(0.0)



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
    val currentServoAngle: HaRotation2d get() = leftMotor?.currentPosition ?: HaRotation2d.fromDegrees(0.0) //we are using the left motor's encoder slot for the servo's encoder
    val currentHoodAngle: HaRotation2d get() = ShooterConstants.MIN_HOOD_ANGLE + currentServoAngle * ShooterConstants.HOOD_ANGLE_TRANSMISSION_RATIO
    val isCurrentAboveThreshold: Boolean get() = rightMotor?.isCurrentOver(ShooterConstants.CURRENT_THRESHOLD) ?: false
    val isWithinVelocityTolerance: Boolean get() = currentMotorVelocity.asRPS in (desiredVelocity.asRPS - ShooterConstants.VELOCITY_TOLERANCE.asRPS)..(desiredVelocity.asRPS + ShooterConstants.VELOCITY_TOLERANCE.asRPS)
    val isWithinAngleTolerance: Boolean get() = currentHoodAngle.asDegrees in (currentHoodAngle.asDegrees - ShooterConstants.ANGLE_TOLERANCE.asDegrees)..(currentHoodAngle.asDegrees + ShooterConstants.ANGLE_TOLERANCE.asDegrees)

    val currentShooterVelocity: AngularVelocity get() = rightMotor?.currentVelocity?.times(
        ShooterConstants.SPEED_TRANSMISSION_RATIO) ?: AngularVelocity.fromRPM(0.0)

    fun setDesiredVelocity(newDesiredVelocity: AngularVelocity) {
        desiredVelocity = newDesiredVelocity
    }

    fun setDesiredHoodAngle(newDesiredAngle: HaRotation2d) {
        desiredHoodAngle = newDesiredAngle
    }


    fun updateShooterVelocityControl() {
        rightMotor?.setVoltage(speedPIDController.calculate(currentShooterVelocity.asRPS, desiredVelocity.asRPS / ShooterConstants.SPEED_TRANSMISSION_RATIO))
        leftMotor?.setVoltage(speedPIDController.calculate(currentShooterVelocity.asRPS, desiredVelocity.asRPS / ShooterConstants.SPEED_TRANSMISSION_RATIO))
    }

    fun stopWheel() {
        rightMotor?.stopMotor()
        leftMotor?.stopMotor()
    }

    fun updateHoodAngleControl() {
        if (desiredHoodAngle.asDegrees in ShooterConstants.MIN_HOOD_ANGLE.asDegrees..ShooterConstants.MAX_HOOD_ANGLE.asDegrees) {
            servo?.setVoltage(hoodAnglePIDController.calculate(
                currentHoodAngle.asDegrees,
                   (desiredHoodAngle.asDegrees - ShooterConstants.MIN_HOOD_ANGLE.asDegrees) / ShooterConstants.HOOD_ANGLE_TRANSMISSION_RATIO))
        }
    }

    fun setWheelMotorsVoltage(voltage: Volts) {
        rightMotor?.setVoltage(voltage)
        leftMotor?.setVoltage(voltage)
    }

    override fun periodic() {
        updateShooterVelocityControl()
        updateHoodAngleControl()
    }

    override fun updateTelemetry(telemetry: Telemetry, dashboardPacket: TelemetryPacket) {

    }
}