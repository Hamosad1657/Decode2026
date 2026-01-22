package com.hamosad.lib.math

import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.hamosad.lib.vision.RobotPoseStdDevs
import com.pedropathing.ftc.FTCCoordinates
import com.pedropathing.ftc.PoseConverter
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

private const val INCH_TO_METER_RATIO = 0.0254


class Length private constructor(private val lengthMeters: Double) {
    companion object {
        fun fromMeters(lengthMeters: Double): Length = Length(lengthMeters)

        fun fromInches(lengthInches: Double): Length = Length(lengthInches * INCH_TO_METER_RATIO)

        fun fromCentimeters(lengthCentimeters: Double): Length = Length(lengthCentimeters / 100)

        fun fromMillimeters(lengthMillimeters: Double): Length = Length(lengthMillimeters / 1000)
    }

    val asMeters get() = lengthMeters
    val asInches get () = lengthMeters / INCH_TO_METER_RATIO
    val asCentimeters get() = lengthMeters * 100
    val asMillimeters get() = lengthMeters * 1000

    operator fun plus(other: Length): Length = Length(this.lengthMeters + other.lengthMeters)
    operator fun minus(other: Length): Length = Length(this.lengthMeters - other.lengthMeters)
    operator fun times(other: Double): Length = Length(this.lengthMeters * other)
    operator fun times(other: Int): Length = Length(this.lengthMeters * other)
    operator fun div(other: Double): Length = Length(this.lengthMeters / other)
    operator fun div(other: Int): Length = Length(this.lengthMeters / other)
}

class HaRotation2d private constructor(private val angleRotations: Double) {
    companion object {
        fun fromRotations(angleRotations: Double): HaRotation2d = HaRotation2d(angleRotations)

        fun fromDegrees(angleDegrees: Double): HaRotation2d = HaRotation2d(angleDegrees / 360)

        fun fromRadians(angleRadians: Double): HaRotation2d = HaRotation2d(angleRadians / (2 * PI))
    }

    val asRotations = angleRotations
    val asDegrees = angleRotations * 360
    val asRadians = angleRotations * 2 * PI

    val cosine = cos(asRadians)
    val sine = sin(asRadians)

    fun toFTCLibR2d() = Rotation2d.fromDegrees(asDegrees)

    operator fun plus(other: HaRotation2d): HaRotation2d = HaRotation2d(this.angleRotations + other.angleRotations)
    operator fun minus(other: HaRotation2d): HaRotation2d = HaRotation2d(this.angleRotations - other.angleRotations)
    operator fun times(other: Double): HaRotation2d = HaRotation2d(this.angleRotations * other)
    operator fun times(other: Int): HaRotation2d = HaRotation2d(this.angleRotations * other)
    operator fun div(other: Double): HaRotation2d = HaRotation2d(this.angleRotations / other)
    operator fun div(other: Int): HaRotation2d = HaRotation2d(this.angleRotations / other)
}

class Rotation3d(
    val pitchAngle: HaRotation2d,
    val yawAngle: HaRotation2d,
    val rollAngle: HaRotation2d
    ) {
    companion object {
        fun fromRotations(pitchAngleRotations: Double, yawAngleRotations: Double, rollAngleRotations: Double): Rotation3d =
            Rotation3d(
                HaRotation2d.fromRotations(pitchAngleRotations),
                HaRotation2d.fromRotations(yawAngleRotations),
                HaRotation2d.fromRotations(rollAngleRotations)
            )

        fun fromDegrees(pitchAngleDegrees: Double, yawAngleDegrees: Double, rollAngleDegrees: Double): Rotation3d =
            Rotation3d(
                HaRotation2d.fromDegrees(pitchAngleDegrees),
                HaRotation2d.fromDegrees(yawAngleDegrees),
                HaRotation2d.fromDegrees(rollAngleDegrees)
            )

        fun fromRadians(pitchAngleRadians: Double, yawAngleRadians: Double, rollAngleRadians: Double): Rotation3d =
            Rotation3d(
                HaRotation2d.fromRadians(pitchAngleRadians),
                HaRotation2d.fromRadians(yawAngleRadians),
                HaRotation2d.fromRadians(rollAngleRadians)
                )
    }

    val asRotations get() = Triple(pitchAngle.asRotations, yawAngle.asRotations, rollAngle.asRotations)
    val asDegrees get() = Triple(pitchAngle.asDegrees, yawAngle.asDegrees, rollAngle.asDegrees)
    val asRadians get() = Triple(pitchAngle.asRadians, yawAngle.asRadians, rollAngle.asRadians)


    operator fun plus(other: Rotation3d): Rotation3d = Rotation3d(
        this.pitchAngle + other.pitchAngle,
        this.yawAngle + other.yawAngle,
        this.rollAngle + other.rollAngle
    )
    operator fun minus(other: Rotation3d): Rotation3d = Rotation3d(
        this.pitchAngle - other.pitchAngle,
        this.yawAngle - other.yawAngle,
        this.rollAngle - other.rollAngle
    )
    operator fun times(other: Double): Rotation3d = Rotation3d(
        this.pitchAngle * other,
        this.yawAngle * other,
        this.rollAngle * other
    )
    operator fun times(other: Int): Rotation3d = Rotation3d(
        this.pitchAngle * other,
        this.yawAngle * other,
        this.rollAngle * other
    )
    operator fun div(other: Double): Rotation3d = Rotation3d(
        this.pitchAngle / other,
        this.yawAngle / other,
        this.rollAngle / other
    )
    operator fun div(other: Int): Rotation3d = Rotation3d(
        this.pitchAngle / other,
        this.yawAngle / other,
        this.rollAngle / other
    )
}

class AngularVelocity private constructor(private val rps: Double) {
    companion object {
        fun fromRPS(rps: Double): AngularVelocity = AngularVelocity(rps)

        fun fromRPM(rpm: Double): AngularVelocity = AngularVelocity(rpm / 60)

        fun fromRadPS(radPS: Double): AngularVelocity = AngularVelocity(radPS / (2 * PI))

        fun fromDegPS(degPS: Double): AngularVelocity = AngularVelocity(degPS / 360)
    }

    val asRPS get() = rps
    val asRPM get() = rps * 60
    val asRadPS get() = rps * 2 * PI
    val asDegPS get() = rps * 360

    operator fun plus(other: AngularVelocity): AngularVelocity = AngularVelocity(this.rps + other.rps)
    operator fun minus(other: AngularVelocity): AngularVelocity = AngularVelocity(this.rps - other.rps)
    operator fun times(other: Double): AngularVelocity = AngularVelocity(this.rps * other)
    operator fun times(other: Int): AngularVelocity = AngularVelocity(this.rps * other)
    operator fun div(other: Double): AngularVelocity = AngularVelocity(this.rps / other)
    operator fun div(other: Int): AngularVelocity = AngularVelocity(this.rps / other)
}

class HaTranslation2d(val x: Double, val y: Double) {
    constructor(length: Double, angle: HaRotation2d): this(
        length * angle.cosine,
        length * angle.sine,
    )

    val length: Double
        get() = sqrt(x*x + y*y)

    val rotation: HaRotation2d get() = HaRotation2d.fromRadians(atan2(y, x))

    fun toFTCLibT2d() = Translation2d(x, y)

    operator fun plus(other: HaTranslation2d): HaTranslation2d = HaTranslation2d(this.x + other.x, this.y + other.y)
    operator fun minus(other: HaTranslation2d): HaTranslation2d = HaTranslation2d(this.x - other.x, this.y - other.y)
    operator fun times(other: Double): HaTranslation2d = HaTranslation2d(this.x * other, this.y * other)
    operator fun times(other: Int): HaTranslation2d = HaTranslation2d(this.x * other, this.y * other)
    operator fun div(other: Double): HaTranslation2d = HaTranslation2d(this.x / other, this.y / other)
    operator fun div(other: Int): HaTranslation2d = HaTranslation2d(this.x / other, this.y / other)
}

class Translation3d(val x: Double, val y: Double, val z: Double) {
    operator fun plus(other: Translation3d): Translation3d = Translation3d(this.x + other.x, this.y + other.y, this.z + other.z)
    operator fun minus(other: Translation3d): Translation3d = Translation3d(this.x - other.x, this.y - other.y, this.z - other.z)
    operator fun times(other: Double): Translation3d = Translation3d(this.x * other, this.y * other, this.z * other)
    operator fun times(other: Int): Translation3d = Translation3d(this.x * other, this.y * other, this.z * other)
    operator fun div(other: Double): Translation3d = Translation3d(this.x / other, this.y / other, this.z / other)
    operator fun div(other: Int): Translation3d = Translation3d(this.x / other, this.y / other, this.z / other)
}

class HaPose2d(val translation2d: HaTranslation2d, val rotation2d: HaRotation2d) {
    fun toPedroPose(): Pose {
        val pose2D =
            Pose2D(DistanceUnit.METER, translation2d.x, translation2d.y, AngleUnit.RADIANS, rotation2d.asRadians)
        return PoseConverter.pose2DToPose(pose2D, FTCCoordinates.INSTANCE)
    }
    fun toFTCLibPose(): Pose2d {
        return Pose2d(translation2d.toFTCLibT2d(), rotation2d.toFTCLibR2d())
    }
}

class HaRobotPoseEstimation(var pose: HaPose2d, var robotPoseStdDevs: RobotPoseStdDevs) {
    fun addPoseEstimate(newPose: HaRobotPoseEstimation): HaPose2d {
        val translationXVariance = robotPoseStdDevs.translationX.pow(2)
        val translationYVariance = robotPoseStdDevs.translationY.pow(2)
        val rotationVariance = robotPoseStdDevs.rotation.pow(2)

        val newTranslationXVariance = newPose.robotPoseStdDevs.translationX.pow(2)
        val newTranslationYVariance = newPose.robotPoseStdDevs.translationY.pow(2)
        val newRotationVariance = newPose.robotPoseStdDevs.rotation.pow(2)

        // K is short for kalman filter gain
        val translationXK = translationXVariance / (translationXVariance + newTranslationXVariance)
        val translationYK = translationYVariance / (translationYVariance + newTranslationYVariance)
        val rotationK = rotationVariance / (rotationVariance + newRotationVariance)

        val newTranslationX = pose.translation2d.x + translationXK * (newPose.pose.translation2d.x - pose.translation2d.x)
        val newTranslationY = pose.translation2d.y + translationYK * (newPose.pose.translation2d.y - pose.translation2d.y)
        val newRotation: HaRotation2d = HaRotation2d.fromRotations(pose.rotation2d.asRotations + rotationK * (newPose.pose.rotation2d.asRotations - pose.rotation2d.asRotations))

        return HaPose2d(HaTranslation2d(newTranslationX, newTranslationY), newRotation)
    }
}