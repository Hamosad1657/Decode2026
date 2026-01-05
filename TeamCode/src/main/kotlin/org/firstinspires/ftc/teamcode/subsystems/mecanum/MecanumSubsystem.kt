package org.firstinspires.ftc.teamcode.subsystems.mecanum

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds
import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.components.motors.HaMotor
import com.hamosad.lib.components.motors.MotorType
import com.hamosad.lib.components.sensors.HaIMU
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.HaPose2d
import com.hamosad.lib.math.HaRobotPoseEstimation
import com.hamosad.lib.math.HaRotation2d
import com.hamosad.lib.math.HaTranslation2d
import com.hamosad.lib.math.toPIDFController
import com.hamosad.lib.vision.HaAprilTagCamera
import com.hamosad.lib.vision.RobotPoseStdDevs
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants as Constants

object MecanumSubsystem: Subsystem() {
    // FL, BR, FR, BL
    private var motors: List<HaMotor> = listOf()
    private val controllers: List<PIDFController> = listOf(
        Constants.wheelGains.toPIDFController(),
        Constants.wheelGains.toPIDFController(),
        Constants.wheelGains.toPIDFController(),
        Constants.wheelGains.toPIDFController(),
    )
    private var imu: HaIMU? = null

    private val kinematics = MecanumDriveKinematics(
        Translation2d(Constants.CHASSIS_DIMENSIONS.x, Constants.CHASSIS_DIMENSIONS.y),
        Translation2d(Constants.CHASSIS_DIMENSIONS.x, -Constants.CHASSIS_DIMENSIONS.y),
        Translation2d(-Constants.CHASSIS_DIMENSIONS.x, Constants.CHASSIS_DIMENSIONS.y),
        Translation2d(-Constants.CHASSIS_DIMENSIONS.x, -Constants.CHASSIS_DIMENSIONS.y),
    )

    const val USE_VISION = false
    var blobCamera: HaAprilTagCamera? = null

    override fun init(newHardwareMap: HardwareMap) {
        super.init(newHardwareMap)
        motors = listOf(
            HaMotor("FL", hardwareMap!!, MotorType.GO_BUILDA5202),
            HaMotor("BR", hardwareMap!!, MotorType.GO_BUILDA5202),
            HaMotor("FR", hardwareMap!!, MotorType.GO_BUILDA5202),
            HaMotor("BL", hardwareMap!!, MotorType.GO_BUILDA5202),
        )
        motors[0].direction = DcMotorSimple.Direction.REVERSE
        motors[1].direction = DcMotorSimple.Direction.FORWARD
        motors[2].direction = DcMotorSimple.Direction.FORWARD
        motors[3].direction = DcMotorSimple.Direction.REVERSE

        imu = HaIMU("IMU", hardwareMap!!)

        // VISION
        if (USE_VISION) {
            blobCamera = HaAprilTagCamera(
                hardwareMap!!,
                "Webcam 1",
                0,
                Constants.MAX_TRUST_RANGE,
                10.0,
                Constants.CAMERA_POSITION,
                Constants.CAMERA_ROTATION,
                Constants.APRIL_TAG_STD_DEVS
            )
        }
    }


    private val visionEstimation: HaRobotPoseEstimation get() =
        blobCamera?.estimatedPose ?: HaRobotPoseEstimation(
            HaPose2d(HaTranslation2d(0.0, 0.0),
            HaRotation2d.fromDegrees(0.0)),
            RobotPoseStdDevs(0.0, 0.0, 0.0)
        )

    private val currentAngle: HaRotation2d
        get() = imu?.currentYaw ?: HaRotation2d.fromDegrees(0.0)

    // Low level functions
    fun resetGyro() {
        imu?.resetYaw()
    }

    // Low level motor PID
    private var wheelVelocitySetpoints: MecanumDriveWheelSpeeds = MecanumDriveWheelSpeeds()

    /** Called in loops to control PID of motors. */
    private fun controlMotors(newSetpoints: MecanumDriveWheelSpeeds = wheelVelocitySetpoints) {
        wheelVelocitySetpoints = newSetpoints

        motors[0].setVoltage(controllers[0].calculate(
            motors[0].currentVelocity.asRPS, newSetpoints.frontLeftMetersPerSecond / Constants.WHEEL_CIRCUMFERENCE.asMeters))
        motors[1].setVoltage(controllers[1].calculate(
            motors[1].currentVelocity.asRPS, newSetpoints.rearRightMetersPerSecond / Constants.WHEEL_CIRCUMFERENCE.asMeters))
        motors[2].setVoltage(controllers[2].calculate(
            motors[2].currentVelocity.asRPS, newSetpoints.frontRightMetersPerSecond / Constants.WHEEL_CIRCUMFERENCE.asMeters))
        motors[3].setVoltage(controllers[3].calculate(
            motors[3].currentVelocity.asRPS, newSetpoints.rearLeftMetersPerSecond / Constants.WHEEL_CIRCUMFERENCE.asMeters))
    }

    fun spinClockwise(angularVelocity: AngularVelocity) {
        controlMotors(kinematics.toWheelSpeeds(
            ChassisSpeeds(0.0, 0.0, angularVelocity.asRadPS)))
    }

    fun drive(fieldRelative: Boolean, chassisSpeeds: ChassisSpeeds) {
        val updatedSpeeds = if (fieldRelative) ChassisSpeeds.fromFieldRelativeSpeeds(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond,
            currentAngle.toFTCLibR2d(),
        ) else chassisSpeeds

        controlMotors(kinematics.toWheelSpeeds(updatedSpeeds))
    }

    // Testing
    fun testMotors(motor: Int) {
        motors[motor].setVoltage(4.0)
    }

    fun testPID() {
        for (i in 0..3) {
            motors[i].setVoltage(controllers[i].calculate(motors[i].currentVelocity.asRPM, 2.0))
        }
    }

    // Periodic
    override fun periodic() {

    }

    // Telemetry
    override fun updateTelemetry(telemetry: Telemetry, dashboardPacket: TelemetryPacket) {
        // FL, BR, FR, BL
        if (wheelVelocitySetpoints.frontLeftMetersPerSecond == 0.0) {
            dashboardPacket.put("FL setpoint RPM", wheelVelocitySetpoints.frontLeftMetersPerSecond)
            dashboardPacket.put("BR setpoint RPM", wheelVelocitySetpoints.rearRightMetersPerSecond)
            dashboardPacket.put("FR setpoint RPM", wheelVelocitySetpoints.frontRightMetersPerSecond)
            dashboardPacket.put("BL setpoint RPM", wheelVelocitySetpoints.rearLeftMetersPerSecond)

            dashboardPacket.put("FL velocity RPM", motors[0].currentVelocity.asRPM)
            dashboardPacket.put("BR velocity RPM", motors[1].currentVelocity.asRPM)
            dashboardPacket.put("FR velocity RPM", motors[2].currentVelocity.asRPM)
            dashboardPacket.put("BL velocity RPM", motors[3].currentVelocity.asRPM)
        }
    }
}