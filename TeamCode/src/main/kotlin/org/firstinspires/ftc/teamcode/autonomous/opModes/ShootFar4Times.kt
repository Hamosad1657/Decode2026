package org.firstinspires.ftc.teamcode

import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.opModes.PedroOpMode
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem


class ShootFar4Times : PedroOpMode() {
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
        var Path9: PathChain?
        var Path10: PathChain?
        var Path11: PathChain?
        var Path8: PathChain?

        init {
            Path1 = follower.pathBuilder().addPath(
                BezierCurve(
                    Pose(50.000, 9.000),
                    Pose(70.000, 11.000),
                    Pose(60.000, 22.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(25.0))

                .build()

            Path2 = follower.pathBuilder().addPath(
                BezierCurve(
                    Pose(60.000, 22.000),
                    Pose(49.806, 35.613),
                    Pose(41.500, 36.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(25.0), Math.toRadians(180.0))

                .build()

            Path3 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(41.500, 36.000),

                    Pose(20.000, 36.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))

                .build()

            Path4 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(20.000, 36.000),

                    Pose(60.000, 22.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(25.0))

                .build()

            Path5 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(60.000, 22.000),

                    Pose(41.500, 60.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(25.0), Math.toRadians(180.0))

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

                    Pose(60.000, 22.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(25.0))

                .build()

            Path9 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(60.000, 22.000),

                    Pose(41.500, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(25.0), Math.toRadians(180.0))

                .build()

            Path10 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(41.500, 84.000),

                    Pose(20.000, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))

                .build()

            Path11 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(20.000, 84.000),

                    Pose(60.000, 22.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(25.0))

                .build()

            Path8 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(60.000, 22.000),

                    Pose(55.000, 25.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(25.0), Math.toRadians(25.0))

                .build()
        }
    }

    override fun configureDefaultCommands() {
        TODO()
    }

    override fun runPathStateCommands() {
        TODO()
    }
}