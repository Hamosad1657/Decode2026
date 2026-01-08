package org.firstinspires.ftc.teamcode

import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.opModes.PedroOpMode
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.ShootClose.Paths
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem

class ShootFar : PedroOpMode() {
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

        init {
            Path1 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(50.000, 9.000),

                    Pose(60.000, 18.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(25.0))

                .build()

            Path2 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(60.000, 18.000),

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