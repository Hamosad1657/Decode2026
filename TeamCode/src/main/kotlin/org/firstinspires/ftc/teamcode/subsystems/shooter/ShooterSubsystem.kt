package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.components.motors.HaCRServoMotor
import com.hamosad.lib.components.motors.HaMotor
import com.hamosad.lib.components.motors.MotorType
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.HaRotation2d
import com.hamosad.lib.math.Volts
import com.hamosad.lib.math.toPIDFController
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

object ShooterSubsystem: Subsystem() {
    private var rightMotor: HaMotor? = null
    private var leftMotor: HaMotor? = null
    private var servo: HaCRServoMotor? = null
    private val hoodAnglePIDController = ShooterConstants.HOOD_ANGLE_GAINS.toPIDFController()

    override fun init(newHardwareMap: HardwareMap) {
        super.init(newHardwareMap)
        rightMotor = HaMotor(ShooterConstants.RIGHT_MOTOR_NAME, hardwareMap!!, MotorType.GO_BUILDA5202)
        leftMotor = HaMotor(ShooterConstants.LEFT_MOTOR_NAME, hardwareMap!!, MotorType.REV_THROUGH_BORE_ENCODER)
        servo = HaCRServoMotor(ShooterConstants.SERVO_NAME, hardwareMap!!)
        leftMotor?.direction = ShooterConstants.LEFT_MOTOR_DIRECTION
        rightMotor?.direction = ShooterConstants.RIGHT_MOTOR_DIRECTION
        servo?.direction = ShooterConstants.SERVO_DIRECTION
        leftMotor?.resetEncoder() // shooter must start at min angle
    }

    val currentServoAngle: HaRotation2d get() = HaRotation2d.fromDegrees(-1 * (leftMotor?.currentPosition?.asDegrees ?: 0.0)) //we are using the left motor's encoder slot for the servo's encoder
    val currentHoodAngle: HaRotation2d get() = ShooterConstants.MIN_HOOD_ANGLE + currentServoAngle * ShooterConstants.HOOD_ANGLE_TRANSMISSION_RATIO

    val isCurrentAboveThreshold: Boolean get() = rightMotor?.isCurrentOver(ShooterConstants.CURRENT_THRESHOLD) ?: false
    val isWithinTolerance: Boolean get() = currentHoodAngle.asDegrees in (currentHoodAngle.asDegrees - ShooterConstants.ANGLE_TOLERANCE.asDegrees)..(currentHoodAngle.asDegrees + ShooterConstants.ANGLE_TOLERANCE.asDegrees)
    var desiredHoodAngle: HaRotation2d = HaRotation2d.fromDegrees(0.0)

    fun updateHoodAngleControl(newAngleSetpoint: HaRotation2d = desiredHoodAngle) {
        if (desiredHoodAngle.asDegrees in ShooterConstants.MIN_HOOD_ANGLE.asDegrees..ShooterConstants.MAX_HOOD_ANGLE.asDegrees) {
            desiredHoodAngle = newAngleSetpoint
            servo?.setVoltage(hoodAnglePIDController.calculate(
                currentHoodAngle.asDegrees,
                   desiredHoodAngle.asDegrees / ShooterConstants.HOOD_ANGLE_TRANSMISSION_RATIO))
        }
    }

    fun stopWheelMotors() {
        rightMotor?.stopMotor()
        leftMotor?.stopMotor()
    }

    fun setWheelMotorsVoltage(voltage: Volts) {
        rightMotor?.setVoltage(voltage)
        leftMotor?.setVoltage(voltage)
    }

    fun setServoVoltage(voltage: Volts) {
        servo?.setVoltage(voltage)
    }


    override fun periodic() {
    }

    override fun updateTelemetry(telemetry: Telemetry) {
        telemetry.addData("Is current above threshold", isCurrentAboveThreshold)


        telemetry.addData("Servo angle deg", currentServoAngle.asDegrees)

        telemetry.addData("Hood angle deg", currentHoodAngle.asDegrees)
        telemetry.addData("desired hood angle deg", desiredHoodAngle.asDegrees)

        telemetry.addData("Is current above threshold", isCurrentAboveThreshold)

        telemetry.addData("Is within angle tolerance", isWithinTolerance)
    }
}