package org.firstinspires.ftc.teamcode

import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.opModes.PedroOpMode
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.ShootClose.Paths
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem


class ShootClose3Times : PedroOpMode() {
    override var subsystemsToUse: List<Subsystem> = listOf(MecanumSubsystem)
    var paths: Paths? = null

    override fun redefinePaths() {
        if (super.follower != null) {
            paths = Paths(super.follower!!)
        }
    }
    class Paths(follower: Follower) {
        var Path1: PathChain?
        var Path2: PathChain?
        var Path3: PathChain?
        var Path4: PathChain?
        var Path5: PathChain?
        var Path6: PathChain?
        var Path7: PathChain?
        var Path8: PathChain?

        init {
            Path1 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(21.500, 122.500),

                    Pose(30.000, 112.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(54.0), Math.toRadians(45.0))

                .build()

            Path2 = follower.pathBuilder().addPath(
                BezierCurve(
                    Pose(30.000, 112.000),
                    Pose(50.000, 97.000),
                    Pose(41.500, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(45.0), Math.toRadians(180.0))

                .build()

            Path3 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(41.500, 84.000),

                    Pose(20.000, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))

                .build()

            Path4 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(20.000, 84.000),

                    Pose(30.000, 112.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(45.0))

                .build()

            Path5 = follower.pathBuilder().addPath(
                BezierCurve(
                    Pose(30.000, 112.000),
                    Pose(44.189, 90.145),
                    Pose(41.500, 60.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(45.0), Math.toRadians(180.0))

                .build()

            Path6 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(41.500, 60.000),

                    Pose(20.000, 60.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))

                .build()

            Path7 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(20.000, 60.000),

                    Pose(30.000, 112.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(45.0))

                .build()

            Path8 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(30.000, 112.000),

                    Pose(23.000, 103.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(45.0), Math.toRadians(45.0))

                .build()
        }
    }


    override fun configureDefaultCommands() {
        TODO("that")
    }

    override fun runPathStateCommands() {
        TODO("Not yet implemented")
    }
}