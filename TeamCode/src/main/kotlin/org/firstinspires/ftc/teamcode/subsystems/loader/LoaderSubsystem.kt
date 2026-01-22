package org.firstinspires.ftc.teamcode.subsystems.loader

import android.provider.SyncStateContract
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.components.motors.HaCRServoMotor
import com.hamosad.lib.components.motors.HaMotor
import com.hamosad.lib.components.motors.HaServoMotor
import com.hamosad.lib.components.motors.MotorType
import com.hamosad.lib.components.sensors.HaColorSensor
import com.arcrobotics.ftclib.controller.PIDFController
import com.hamosad.lib.math.HaRotation2d
import com.hamosad.lib.math.Volts
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.commands.Ball
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.floor
import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderConstants as Constants

object LoaderSubsystem: Subsystem() {
    private var rouletteServo: HaCRServoMotor? = null
    private val rouletteController = PIDFController(
        Constants.ROULETTE_ANGLE_GAINS.p,
        Constants.ROULETTE_ANGLE_GAINS.i,
        Constants.ROULETTE_ANGLE_GAINS.d,
        Constants.ROULETTE_ANGLE_GAINS.f
    )

    private var colorSensor: HaColorSensor? = null

    private var armServo: HaServoMotor? = null
    private var armMotor: HaMotor? = null
    private var angleSetpoint: HaRotation2d = HaRotation2d.fromDegrees(0.0)

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
    private val rouletteAngle: HaRotation2d
        get() = armMotor?.currentPosition ?: HaRotation2d.fromDegrees(0.0) // Encoder is connected to the arm motor's encoder port
    private val absoluteRouletteAngle: HaRotation2d
        get() {
            val currentPosition = rouletteAngle
            return HaRotation2d.fromDegrees(currentPosition.asDegrees - floor(currentPosition.asDegrees / 360) * 360)
        }

    val isAtSetpoint: Boolean
        get() = (absoluteRouletteAngle.asDegrees in (
                angleSetpoint.asDegrees - Constants.ROULETTE_TOLERANCE.asDegrees)..(angleSetpoint.asDegrees + Constants.ROULETTE_TOLERANCE.asDegrees))

    var ball1Color: BallColor = BallColor.UNKNOWN
    var ball2Color: BallColor = BallColor.UNKNOWN
    var ball3Color: BallColor = BallColor.UNKNOWN

    fun returnBallColor(ball: Ball): BallColor {
        return when (ball) {
            Ball.BALL_1 -> ball1Color
            Ball.BALL_2 -> ball2Color
            Ball.BALL_3 -> ball3Color
        }
    }

    val closestBallToShooter: Ball get() {
        if (absoluteRouletteAngle.asDegrees in 360 - (Constants.BALL_1_AT_SHOOTER.asDegrees - Constants.BALL_LOCATION_TOLERANCE.asDegrees).absoluteValue.. 360.0
            || absoluteRouletteAngle.asDegrees in Constants.BALL_1_AT_SHOOTER.asDegrees.. Constants.BALL_1_AT_SHOOTER.asDegrees + Constants.BALL_LOCATION_TOLERANCE.asDegrees)
            return Ball.BALL_1
        else if (absoluteRouletteAngle.asDegrees in Constants.BALL_2_AT_SHOOTER.asDegrees - Constants.BALL_LOCATION_TOLERANCE.asDegrees.. Constants.BALL_2_AT_SHOOTER.asDegrees + Constants.BALL_LOCATION_TOLERANCE.asDegrees)
            return Ball.BALL_2
        return Ball.BALL_3
    }

    val furthestBallFromShooter: Ball get() {
        if (absoluteRouletteAngle.asDegrees in 360 - (Constants.BALL_1_FURTHEST_FROM_SHOOTER.asDegrees - Constants.BALL_LOCATION_TOLERANCE.asDegrees).absoluteValue.. 360.0
            || absoluteRouletteAngle.asDegrees in Constants.BALL_1_FURTHEST_FROM_SHOOTER.asDegrees.. Constants.BALL_1_FURTHEST_FROM_SHOOTER.asDegrees + Constants.BALL_LOCATION_TOLERANCE.asDegrees)
            return Ball.BALL_1
        else if (absoluteRouletteAngle.asDegrees in Constants.BALL_2_FURTHEST_FROM_SHOOTER.asDegrees - Constants.BALL_LOCATION_TOLERANCE.asDegrees.. Constants.BALL_2_FURTHEST_FROM_SHOOTER.asDegrees + Constants.BALL_LOCATION_TOLERANCE.asDegrees)
            return Ball.BALL_2
        return Ball.BALL_3
    }

    val middleBallFromShooter: Ball get() {
        return when (closestBallToShooter) {
            Ball.BALL_1 -> if (furthestBallFromShooter == Ball.BALL_2) Ball.BALL_3 else Ball.BALL_2
            Ball.BALL_2 -> if (furthestBallFromShooter == Ball.BALL_1) Ball.BALL_3 else Ball.BALL_1
            Ball.BALL_3 -> if (furthestBallFromShooter == Ball.BALL_2) Ball.BALL_1 else Ball.BALL_2
        }
    }


    val closestBallToIntake: Ball get() =
        if (rouletteAngle.asDegrees in 360 - (Constants.BALL_1_AT_INTAKE.asDegrees - Constants.BALL_LOCATION_TOLERANCE.asDegrees).absoluteValue..0.0
            || rouletteAngle.asDegrees in 0.0..Constants.BALL_1_AT_INTAKE.asDegrees + Constants.BALL_LOCATION_TOLERANCE.asDegrees) {
            Ball.BALL_1
        } else if (rouletteAngle.asDegrees in Constants.BALL_2_AT_INTAKE.asDegrees - Constants.BALL_LOCATION_TOLERANCE.asDegrees..Constants.BALL_2_AT_INTAKE.asDegrees + Constants.BALL_LOCATION_TOLERANCE.asDegrees) {
            Ball.BALL_2
        } else {
            Ball.BALL_3
        }
    val furthestBallFromIntake: Ball get() =
        if (rouletteAngle.asDegrees in 360 - (Constants.BALL_1_FURTHEST_FROM_INTAKE.asDegrees - Constants.BALL_LOCATION_TOLERANCE.asDegrees).absoluteValue..0.0
            || rouletteAngle.asDegrees in 0.0..Constants.BALL_1_FURTHEST_FROM_INTAKE.asDegrees + Constants.BALL_LOCATION_TOLERANCE.asDegrees) {
            Ball.BALL_1
        } else if (rouletteAngle.asDegrees in Constants.BALL_2_FURTHEST_FROM_INTAKE.asDegrees - Constants.BALL_LOCATION_TOLERANCE.asDegrees..Constants.BALL_2_FURTHEST_FROM_INTAKE.asDegrees + Constants.BALL_LOCATION_TOLERANCE.asDegrees) {
            Ball.BALL_2
        } else {
            Ball.BALL_3
        }

    val middleBallFromIntake: Ball get() {
        return when (closestBallToIntake) {
            Ball.BALL_1 -> if (furthestBallFromIntake == Ball.BALL_2) Ball.BALL_3 else Ball.BALL_2
            Ball.BALL_2 -> if (furthestBallFromIntake == Ball.BALL_1) Ball.BALL_3 else Ball.BALL_1
            Ball.BALL_3 -> if (furthestBallFromIntake == Ball.BALL_2) Ball.BALL_1 else Ball.BALL_2
        }
    }
    // Roulette functions
    fun updateRouletteControl(newSetpoint: HaRotation2d = angleSetpoint) {
        angleSetpoint = newSetpoint
        var errorRad = angleSetpoint.asRadians - absoluteRouletteAngle.asRadians

        val shouldMoveCounterClockwise = (errorRad) in 0.0..PI || (errorRad) in -2*PI..-PI

        errorRad = if (shouldMoveCounterClockwise) errorRad.absoluteValue else -errorRad.absoluteValue

        rouletteServo?.setVoltage(rouletteController.calculate(
            rouletteAngle.asRadians,
            rouletteAngle.asRadians + errorRad
        ))
    }

    private fun determineCurrentColor(): BallColor {
        return if (colorSensor?.isInColorRange(Constants.GREEN_COLOR, Constants.COLOR_SENSOR_TOLERANCE) ?: false) {
            BallColor.GREEN
        } else if (colorSensor?.isInColorRange(Constants.PURPLE_COLOR, Constants.COLOR_SENSOR_TOLERANCE) ?: false) {
            BallColor.PURPLE
        } else {
            BallColor.UNKNOWN
        }
    }

    private fun updateBallColors() {
        when (absoluteRouletteAngle.asDegrees) {
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

    fun setServoVoltage(volt: Volts) {
        rouletteServo?.setVoltage(volt)
    }

    // Arm functions
    fun loadToShooter() {
        armServo?.currentCommandedPosition = Constants.OPEN_ARM_ANGLE
        armMotor?.setVoltage(Constants.ARM_MOTOR_VOLTAGE)
    }

    fun stopLoadingToShooter() {
        armServo?.currentCommandedPosition = Constants.RETRACTED_ARM_ANGLE
        armMotor?.stopMotor()
    }

    // Periodic
    override fun periodic() {
        updateBallColors()
    }

    // Telemetry
    override fun updateTelemetry(telemetry: Telemetry, dashboardPacket: TelemetryPacket) {
        telemetry.addData("Current roulette angle deg", rouletteAngle.asDegrees)
        telemetry.addData("Current absolute roulette angle deg", absoluteRouletteAngle.asDegrees)
        telemetry.addData("Current angle setpoint deg", angleSetpoint)

        telemetry.addData("Is at setpoint", isAtSetpoint)


        telemetry.addData("Ball 1 color", ball1Color.name)
        telemetry.addData("Ball 2 color", ball2Color.name)
        telemetry.addData("Ball 3 color", ball3Color.name)

        telemetry.addData("Closest ball to shooter", closestBallToShooter.name)
        telemetry.addData("Closest ball to intake", closestBallToIntake.name)
    }
}