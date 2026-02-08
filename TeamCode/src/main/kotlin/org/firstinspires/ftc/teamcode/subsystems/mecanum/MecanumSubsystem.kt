package org.firstinspires.ftc.teamcode.subsystems.mecanum

import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry
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
import kotlin.math.absoluteValue
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants as Constants

object MecanumSubsystem: Subsystem() {
    // -- Components --

    // FL, BR, FR, BL
    private var motors: List<HaMotor> = listOf()
    private val controllers: List<PIDFController> = listOf(
        Constants.WHEEL_GAINS.toPIDFController(),
        Constants.WHEEL_GAINS.toPIDFController(),
        Constants.WHEEL_GAINS.toPIDFController(),
        Constants.WHEEL_GAINS.toPIDFController(),
    )
    private var imu: HaIMU? = null
    var apriltagCamera: HaAprilTagCamera? = null

    // -- Fields --

    private val kinematics = MecanumDriveKinematics(
        Translation2d(Constants.CHASSIS_DIMENSIONS.x, Constants.CHASSIS_DIMENSIONS.y),
        Translation2d(Constants.CHASSIS_DIMENSIONS.x, -Constants.CHASSIS_DIMENSIONS.y),
        Translation2d(-Constants.CHASSIS_DIMENSIONS.x, Constants.CHASSIS_DIMENSIONS.y),
        Translation2d(-Constants.CHASSIS_DIMENSIONS.x, -Constants.CHASSIS_DIMENSIONS.y),
    )

    private val odometry = MecanumDriveOdometry(
        kinematics, Rotation2d(0.0)
    )

    const val USE_VISION = false

    val yawPIDController = Constants.YAW_PID_GAINS.toPIDFController()

    // -- Init --

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
            apriltagCamera = HaAprilTagCamera(
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

        resetGyro()
    }

    // -- Property getters --

    private val visionEstimation: HaRobotPoseEstimation =
        apriltagCamera?.estimatedPose ?: HaRobotPoseEstimation(
            HaPose2d(HaTranslation2d(0.0, 0.0),
            HaRotation2d.fromDegrees(0.0)),
            RobotPoseStdDevs(0.0, 0.0, 0.0)
        )

    private val odometryEstimation: HaRobotPoseEstimation =
        HaRobotPoseEstimation(
        HaPose2d(
            HaTranslation2d(0.0, 0.0),
            HaRotation2d.fromDegrees(0.0)
            ),
        RobotPoseStdDevs(0.0, 0.0, 0.0)
        )


    val robotPose: HaPose2d get() =
        if (USE_VISION && visionEstimation.pose.translation2d.y in -0.01..0.01 && visionEstimation.pose.translation2d.y in -0.01..0.01)
            odometryEstimation.addPoseEstimate(visionEstimation) else odometryEstimation.pose

    val currentAbsoluteAngle: HaRotation2d
        get() = imu?.currentYaw ?: HaRotation2d.fromDegrees(0.0)

    val currentSpeeds: MecanumDriveWheelSpeeds
        get() = MecanumDriveWheelSpeeds(
            motors[0].currentVelocity.asRPS * Constants.WHEEL_CIRCUMFERENCE.asMeters,
            motors[2].currentVelocity.asRPS * Constants.WHEEL_CIRCUMFERENCE.asMeters,
            motors[3].currentVelocity.asRPS * Constants.WHEEL_CIRCUMFERENCE.asMeters,
            motors[1].currentVelocity.asRPS * Constants.WHEEL_CIRCUMFERENCE.asMeters,
        )

    val currentVelocity: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(currentSpeeds)

    var currentAngle: HaRotation2d = HaRotation2d.fromDegrees(0.0)
        private set

    // -- Low level functions --

    private var rotations = 0
    private var isFirstLoop = true
    fun updateAngle() {
        var newAngle = currentAbsoluteAngle
        if (newAngle.asDegrees < 0.0) newAngle = HaRotation2d.fromDegrees(360.0 + newAngle.asDegrees)

        if (((currentAngle.asDegrees - (rotations * 360.0)) - newAngle.asDegrees).absoluteValue > 180.0 && !isFirstLoop) {
            if (currentAngle.asDegrees - (rotations * 360.0) > 180.0) {
                rotations++
            } else {
                rotations--
            }
        }
        currentAngle = HaRotation2d.fromDegrees(newAngle.asDegrees + 360.0 * rotations)
        isFirstLoop = false
    }

    var currentTime = System.currentTimeMillis() / 1000.0
    fun updateOdometry() {
        currentTime = System.currentTimeMillis() / 1000.0
        val newPose = odometry.updateWithTime(currentTime, currentAbsoluteAngle.toFTCLibR2d(), currentSpeeds)
        odometryEstimation.pose = HaPose2d(
            HaTranslation2d(newPose.x, newPose.y), HaRotation2d.fromDegrees(newPose.rotation.degrees)
        )
    }

    fun resetPose(newPose: Pose2d) {
        odometry.resetPosition(newPose, currentAbsoluteAngle.toFTCLibR2d())
        odometryEstimation.pose = HaPose2d(
            HaTranslation2d(newPose.x, newPose.y), HaRotation2d.fromDegrees(newPose.rotation.degrees)
        )
    }
    fun resetGyro() {
        odometry.resetPosition(odometry.poseMeters, currentAbsoluteAngle.toFTCLibR2d())

        imu?.resetYaw()
        currentAngle = HaRotation2d.fromDegrees(0.0)
        rotations = 0
    }


    // -- Motor and components control --
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
            currentAbsoluteAngle.toFTCLibR2d(),
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

    // -- Periodic --
    override fun periodic() {
        updateAngle()
        updateOdometry()
    }

    // -- Telemetry --
    override fun updateTelemetry(telemetry: Telemetry) {
        val translation = robotPose.translation2d

        // FL, BR, FR, BL
        if (wheelVelocitySetpoints.frontLeftMetersPerSecond != 0.0) {
            telemetry.addData("FL Speed setpoint MPS", wheelVelocitySetpoints.frontLeftMetersPerSecond)
            telemetry.addData("BR Speed setpoint MPS", wheelVelocitySetpoints.rearRightMetersPerSecond)
            telemetry.addData("FR Speed setpoint MPS", wheelVelocitySetpoints.frontRightMetersPerSecond)
            telemetry.addData("BL Speed setpoint MPS", wheelVelocitySetpoints.rearLeftMetersPerSecond)

            telemetry.addData("FL velocity MPS", motors[0].currentVelocity.asRPS * Constants.WHEEL_CIRCUMFERENCE.asMeters)
            telemetry.addData("BR velocity MPS", motors[1].currentVelocity.asRPS * Constants.WHEEL_CIRCUMFERENCE.asMeters)
            telemetry.addData("FR velocity MPS", motors[2].currentVelocity.asRPS * Constants.WHEEL_CIRCUMFERENCE.asMeters)
            telemetry.addData("BL velocity MPS", motors[3].currentVelocity.asRPS * Constants.WHEEL_CIRCUMFERENCE.asMeters)

            telemetry.addData("Odometry X", translation.x)
            telemetry.addData("Odometry Y", translation.y)

            telemetry.addData("Non absolute angle deg", currentAngle.asDegrees)
        }
    }
}