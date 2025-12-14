package org.firstinspires.ftc.teamcode.pedropathing

import com.pedropathing.follower.Follower
import com.pedropathing.follower.FollowerConstants
import com.pedropathing.ftc.FollowerBuilder
import com.pedropathing.ftc.drivetrains.MecanumConstants
import com.pedropathing.ftc.localization.Encoder
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants
import com.pedropathing.paths.PathConstraints
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

object PedroConstants {
    // Technical constants
    val driveConstants: MecanumConstants = MecanumConstants().apply {
        maxPower(1.0)
        leftFrontMotorName ="FL"
        rightRearMotorName = "BR"
        rightFrontMotorName = "FR"
        leftRearMotorName = "BL"
        leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE
        rightRearMotorDirection = DcMotorSimple.Direction.FORWARD
        rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD
        leftRearMotorDirection = DcMotorSimple.Direction.REVERSE
    }

    val localizerConstants: DriveEncoderConstants = DriveEncoderConstants().apply {
        leftFrontMotorName = "FL"
        rightRearMotorName = "BR"
        rightFrontMotorName = "FR"
        leftRearMotorName = "BL"
        leftFrontEncoderDirection = Encoder.REVERSE
        rightRearEncoderDirection = Encoder.REVERSE
        rightFrontEncoderDirection = Encoder.REVERSE
        leftRearEncoderDirection = Encoder.REVERSE

        robot_Length = 13.188
        robot_Width = 16.535

        forwardTicksToInches = -5.449
    }

    // Following tuning specific constants
    var followerConstants: FollowerConstants = FollowerConstants().apply {
        mass = 5.0
    }

    var pathConstraints: PathConstraints = PathConstraints(0.99, 100.0, 1.0, 1.0)

    @JvmStatic
    fun createFollower(hardwareMap: HardwareMap): Follower {
        return FollowerBuilder(followerConstants, hardwareMap).apply {
            driveEncoderLocalizer(localizerConstants)
            pathConstraints(pathConstraints)
            mecanumDrivetrain(driveConstants)
        }.build()
    }
}