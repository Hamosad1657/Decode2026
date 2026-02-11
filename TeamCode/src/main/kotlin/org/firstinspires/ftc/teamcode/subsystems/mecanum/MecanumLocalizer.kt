package org.firstinspires.ftc.teamcode.subsystems.mecanum

import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.pedropathing.ftc.FTCCoordinates
import com.pedropathing.ftc.PoseConverter
import com.pedropathing.geometry.Pose
import com.pedropathing.localization.Localizer
import com.pedropathing.math.Vector
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class MecanumLocalizer: Localizer {
    val metersToInchesMultiplier = 39.37

    override fun getPose(): Pose? {
        return MecanumSubsystem.robotPose.toPedroPose()
    }

    override fun getIMUHeading(): Double {
        return MecanumSubsystem.currentAbsoluteAngle.asRadians
    }

    override fun resetIMU() {
        MecanumSubsystem.resetGyro()
    }

    override fun getForwardMultiplier(): Double {
        return 1.0
    }

    override fun getLateralMultiplier(): Double {
        return 1.0
    }

    override fun getTurningMultiplier(): Double {
        return 1.0
    }

    override fun getTotalHeading(): Double {
        return MecanumSubsystem.currentAngle.asRadians
    }

    override fun getVelocity(): Pose? {
        val velocity = MecanumSubsystem.currentVelocity
        return Pose(velocity.vxMetersPerSecond * metersToInchesMultiplier, velocity.vyMetersPerSecond * metersToInchesMultiplier, velocity.omegaRadiansPerSecond,
            FTCCoordinates.INSTANCE)
    }

    override fun getVelocityVector(): Vector? {
        return Vector(velocity)
    }

    override fun isNAN(): Boolean {
        return false
    }

    override fun setPose(setPose: Pose?) {
        val pose = PoseConverter.poseToPose2D(setPose, FTCCoordinates.INSTANCE)
        MecanumSubsystem.resetPose(Pose2d(pose.getX(DistanceUnit.METER), pose.getY(DistanceUnit.METER),
            Rotation2d(pose.getHeading(AngleUnit.RADIANS))))
    }

    override fun setStartPose(setStart: Pose?) {
        setPose(setStart)
    }

    override fun update() {

    }
}