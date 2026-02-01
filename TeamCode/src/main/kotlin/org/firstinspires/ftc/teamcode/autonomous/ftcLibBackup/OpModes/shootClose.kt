package org.firstinspires.ftc.teamcode.autonomous.ftcLibBackup.OpModes

import com.hamosad.lib.commands.Command
import com.hamosad.lib.opModes.CommandOpModeAutonomous
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.commands.andThen
import com.hamosad.lib.math.Length
import org.firstinspires.ftc.teamcode.autonomous.ftcLibBackup.Paths
import org.firstinspires.ftc.teamcode.commands.purePursuitFollowPath
import org.firstinspires.ftc.teamcode.commands.shootAllBallsInColorPatternCommand
import org.firstinspires.ftc.teamcode.commands.shootAllBallsInPatternCommand
import org.firstinspires.ftc.teamcode.commands.shootBallCommand
import org.firstinspires.ftc.teamcode.commands.shootClosestBallCommand
import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderSubsystem
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.shooter.interpolateDistanceToShooterState


@Autonomous
class ShootClose : CommandOpModeAutonomous() {
    override var subsystemsToUse: List<Subsystem> = listOf(MecanumSubsystem, ShooterSubsystem, LoaderSubsystem)

    override fun configureDefaultCommands() {}

    override fun getAutonomousCommand(): Command {
        return MecanumSubsystem.purePursuitFollowPath(Paths.shootClosePath) andThen
                shootAllBallsInColorPatternCommand(LoaderSubsystem.currentPattern, {
            interpolateDistanceToShooterState(Length.fromInches(
                MecanumSubsystem.apriltagCamera?.closestTarget?.ftcPose?.range ?: 1.0))
        }, 3.0) andThen MecanumSubsystem.purePursuitFollowPath(Paths.goToCloseEndpointPath)
    }
}