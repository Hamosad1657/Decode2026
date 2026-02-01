package org.firstinspires.ftc.teamcode.pedropathing

import com.pedropathing.control.FilteredPIDFCoefficients
import com.pedropathing.control.PIDFCoefficients
import com.pedropathing.follower.Follower
import com.pedropathing.follower.FollowerConstants
import com.pedropathing.ftc.FollowerBuilder
import com.pedropathing.ftc.drivetrains.MecanumConstants
import com.pedropathing.paths.PathConstraints
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumLocalizer

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

    val localizer = MecanumLocalizer()

    // Following tuning specific constants
    var followerConstants: FollowerConstants = FollowerConstants().apply {
        mass = 5.0
        translationalPIDFCoefficients(PIDFCoefficients(0.1, 0.0, 0.01, 0.0))
        headingPIDFCoefficients(PIDFCoefficients(0.1, 0.0, 0.01, 0.0))
        drivePIDFCoefficients(FilteredPIDFCoefficients(0.1, 0.0, 0.01, 0.6, 0.0))
        centripetalScaling = 0.005
    }

    var pathConstraints: PathConstraints = PathConstraints(0.99, 100.0, 1.0, 1.0)

    @JvmStatic
    fun createFollower(hardwareMap: HardwareMap): Follower {
        return FollowerBuilder(followerConstants, hardwareMap).apply {
            setLocalizer(localizer)
            pathConstraints(pathConstraints)
            mecanumDrivetrain(driveConstants)
        }.build()
    }
}