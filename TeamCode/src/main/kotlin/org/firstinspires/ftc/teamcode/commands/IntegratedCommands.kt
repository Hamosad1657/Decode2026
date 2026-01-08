package org.firstinspires.ftc.teamcode.commands


import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.runCommand
import com.hamosad.lib.commands.andThen
import com.hamosad.lib.commands.runCommand
import com.hamosad.lib.commands.runOnce
import com.hamosad.lib.math.HaRotation2d
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem.stopIntake
import org.firstinspires.ftc.teamcode.subsystems.loader.BallColor
import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderSubsystem
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem

fun collectCommand(): Command {
    return LoaderSubsystem.runOnce{LoaderSubsystem.positionBallToIntakeCommand(closestBall)} andThen IntakeSubsystem.runIntakeCommand()
} //ALON HOMO veloh hechin command

fun shootColoredBall(color: BallColor, angle: HaRotation2d): Command {
    return ShooterSubsystem.setHoodAngleCommand(angle) andThen LoaderSubsystem.shootColorCommand(color)
}
