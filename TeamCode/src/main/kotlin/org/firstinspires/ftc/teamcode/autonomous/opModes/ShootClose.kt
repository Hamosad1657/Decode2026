package org.firstinspires.ftc.teamcode

import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.commands.runOnce
import com.hamosad.lib.opModes.PedroOpMode
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem

class ShootClose : PedroOpMode() {
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
                    Pose(21.500, 122.500),

                    Pose(30.000, 112.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(54.0), Math.toRadians(45.0))

                .build()

            Path2 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(30.000, 112.000),

                    Pose(23.000, 103.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(45.0), Math.toRadians(45.0))

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