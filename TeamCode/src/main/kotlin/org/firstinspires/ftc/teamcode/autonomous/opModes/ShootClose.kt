package org.firstinspires.ftc.teamcode

import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.commands.runOnce
import com.hamosad.lib.math.Length
import com.hamosad.lib.opModes.PedroOpMode
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.shootAllBallsInColorPatternCommand
import org.firstinspires.ftc.teamcode.commands.shootAllBallsInPatternCommand
import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderSubsystem
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.shooter.interpolateDistanceToShooterState

@Autonomous
class ShootClose : PedroOpMode() {
    override var subsystemsToUse: List<Subsystem> = listOf(MecanumSubsystem)

        var Path1: PathChain? = null
        var Path2: PathChain? = null

    override fun runPathStateCommands() {
//        if (pathState == 1) {
//            shootAllBallsInColorPatternCommand(LoaderSubsystem.currentPattern,
//                {interpolateDistanceToShooterState(Length.fromInches(
//                    MecanumSubsystem.apriltagCamera?.closestTarget?.ftcPose?.range ?: 1.0))}, 2.0)
//        }
    }

    override fun definePaths() {
        Path1 = follower?.pathBuilder()?.addPath(
            BezierLine(
                Pose(21.500, 122.500),

                Pose(30.000, 112.000)
            )
        )?.setLinearHeadingInterpolation(Math.toRadians(54.0), Math.toRadians(45.0))

            ?.build()

        Path2 = follower?.pathBuilder()?.addPath(
            BezierLine(
                Pose(30.000, 112.000),

                Pose(23.000, 103.000)
            )
        )?.setLinearHeadingInterpolation(Math.toRadians(45.0), Math.toRadians(45.0))

            ?.build()
    }

    override fun autonomousPathUpdate() {
        when (pathState) {
            0 -> { if (Path1 != null) {follower?.followPath(Path1!!)}; setStartTime(); if ((currentTime - startTime) / 1000 >= 2L) {pathState = 1}}
            1 -> {setStartTime(); if ((currentTime - startTime) / 1000 == 3L) {pathState = 2}}
            2 ->  {if (Path2 != null) {follower?.followPath(Path2!!)}}
        }
    }
}