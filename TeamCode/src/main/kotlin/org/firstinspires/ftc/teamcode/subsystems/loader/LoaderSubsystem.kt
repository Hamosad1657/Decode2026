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
    private val rouletteController = PIDController(Constants.ROULETTE_ANGLE_GAINS)

    private var colorSensor: HaColorSensor? = null

    private var armServo: HaServoMotor? = null
    private var armMotor: HaMotor? = null

    override fun init(newHardwareMap: HardwareMap) {
        super.init(newHardwareMap)
        rouletteServo = HaCRServoMotor(Constants.ROULETTE_SERVO_NAME, hardwareMap!!)
        colorSensor = HaColorSensor(Constants.COLOR_SENSOR_NAME, hardwareMap!!)
        armServo = HaServoMotor(Constants.ARM_SERVO_NAME, hardwareMap!!, Constants.ARM_SERVO_RANGE)
        armMotor = HaMotor(Constants.ARM_MOTOR_NAME, hardwareMap!!, MotorType.REV_THROUGH_BORE_ENCODER)

        rouletteServo?.direction = Constants.ROULETTE_SERVO_DIRECTION
        armServo?.direction = Constants.ARM_SERVO_DIRECTION
        armMotor?.direction = Constants.ARM_MOTOR_DIRECTION
    }
    // Properties
    private val rouletteAngle: Rotation2d
        get() = armMotor?.currentPosition ?: Rotation2d.fromDegrees(0.0) // Encoder is connected to the arm motor's encoder port

    val isAtSetpoint: Boolean
        get() = (rouletteAngle.asDegrees in (
                angleSetpoint.asDegrees - Constants.ROULETTE_TOLERANCE.asDegrees)..(angleSetpoint.asDegrees + Constants.ROULETTE_TOLERANCE.asDegrees))

    var ball1Color: BallColor = BallColor.UNKNOWN
        private set
    var ball2Color: BallColor = BallColor.UNKNOWN
        private set
    var ball3Color: BallColor = BallColor.UNKNOWN
        private set

    // Roulette functions
    private var angleSetpoint = Rotation2d.fromDegrees(0.0)
    fun updateRouletteControl(newSetpoint: Rotation2d = angleSetpoint) {
        // TODO: Add closer side approximation
        angleSetpoint = newSetpoint
        rouletteServo?.setVoltage(rouletteController.calculate(rouletteAngle.asRadians, angleSetpoint.asRadians))
    }

    private fun determineCurrentColor(): BallColor {
        if (colorSensor?.isInColorRange(Constants.GREEN_COLOR, Constants.COLOR_SENSOR_TOLERANCE) ?: false) {
            return BallColor.GREEN
        } else if (colorSensor?.isInColorRange(Constants.PURPLE_COLOR, Constants.COLOR_SENSOR_TOLERANCE) ?: false) {
            return BallColor.PURPLE
        } else {
            return BallColor.UNKNOWN
        }
    }

    private fun updateBallColors() {
        when (rouletteAngle.asDegrees) {
            in (Constants.BALL_1_AT_COLOR_SENSOR.asDegrees - Constants.COLOR_SENSOR_ANGLE_TOLERANCE.asDegrees)..(
                    Constants.BALL_1_AT_COLOR_SENSOR.asDegrees + Constants.COLOR_SENSOR_ANGLE_TOLERANCE.asDegrees) -> {
                ball1Color = determineCurrentColor()
            }

            in (Constants.BALL_2_AT_COLOR_SENSOR.asDegrees - Constants.COLOR_SENSOR_ANGLE_TOLERANCE.asDegrees)..(
                    Constants.BALL_2_AT_COLOR_SENSOR.asDegrees + Constants.COLOR_SENSOR_ANGLE_TOLERANCE.asDegrees) -> {
                ball2Color = determineCurrentColor()
            }

            in (Constants.BALL_3_AT_COLOR_SENSOR.asDegrees - Constants.COLOR_SENSOR_ANGLE_TOLERANCE.asDegrees)..(
                    Constants.BALL_3_AT_COLOR_SENSOR.asDegrees + Constants.COLOR_SENSOR_ANGLE_TOLERANCE.asDegrees) -> {
                ball3Color = determineCurrentColor()
            }
        }
    }

    // Arm functions
    fun loadToShooter() {
        armServo?.currentCommandedPosition = Constants.OPEN_ARM_ANGLE
        armMotor?.setVoltage(Constants.ARM_MOTOR_VOLTAGE)
    }

    fun stopLoadingFromShooter() {
        armServo?.currentCommandedPosition = Constants.RETRACTED_ARM_ANGLE
        armMotor?.stopMotor()
    }

    // Periodic
    override fun periodic() {
        updateBallColors()
    }

    // Telemetry
    override fun updateTelemetry(telemetry: Telemetry, dashboardPacket: TelemetryPacket) {
        telemetry.addData("Ball 1 color", ball1Color.name)
        telemetry.addData("Ball 2 color", ball2Color.name)
        telemetry.addData("Ball 3 color", ball3Color.name)
    }
}