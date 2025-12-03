package org.firstinspires.ftc.teamcode.subsystems.loader

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.components.motors.HaCRServoMotor
import com.hamosad.lib.components.motors.HaMotor
import com.hamosad.lib.components.motors.HaServoMotor
import com.hamosad.lib.components.motors.MotorType
import com.hamosad.lib.components.sensors.HaColorSensor
import com.hamosad.lib.math.PIDController
import com.hamosad.lib.math.Rotation2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderConstants as Constants

object LoaderSubsystem: Subsystem() {
    private var rouletteServo: HaCRServoMotor? = null
    private val rouletteController = PIDController(Constants.ROULETTE_GAINS)

    private var colorSensor: HaColorSensor? = null

    private var armServo: HaServoMotor? = null
    private var armMotor: HaMotor? = null

    override fun init(newHardwareMap: HardwareMap) {
        super.init(newHardwareMap)
        rouletteServo = HaCRServoMotor(Constants.ROULETTE_SERVO_NAME, hardwareMap!!)
        colorSensor = HaColorSensor(Constants.COLOR_SENSOR_NAME, hardwareMap!!)
        armServo = HaServoMotor(Constants.ARM_SERVO_NAME, hardwareMap!!, Rotation2d.fromDegrees(180.0))
        armMotor = HaMotor(Constants.ARM_MOTOR_NAME, hardwareMap!!, MotorType.REV_THROUGH_BORE_ENCODER)

        rouletteServo?.direction = Constants.ROULETTE_SERVO_DIRECTION
        armServo?.direction = Constants.ARM_SERVO_DIRECTION
        armMotor?.direction = Constants.ARM_MOTOR_DIRECTION
    }
    // Property getters
    private val rouletteAngle: Rotation2d
        get() = armMotor?.currentPosition ?: Rotation2d.fromDegrees(0.0) // Encoder is connected to the arm motor's encoder port

    // Roulette
    private var angleSetpoint = Rotation2d.fromDegrees(0.0)
    private fun updateRouletteControl(newSetpoint: Rotation2d = angleSetpoint) {
        angleSetpoint = newSetpoint
        rouletteServo?.setVoltage(rouletteController.calculate(rouletteAngle.asRadians, angleSetpoint.asRadians))
    }

    // Periodic
    override fun periodic() {

    }

    // Telemetry
    override fun updateTelemetry(telemetry: Telemetry, dashboardPacket: TelemetryPacket) {

    }
}