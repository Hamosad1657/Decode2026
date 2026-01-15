package org.firstinspires.ftc.teamcode.commands


import com.hamosad.lib.commands.Command
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.commands.andThen
import com.hamosad.lib.commands.meanwhile
import com.hamosad.lib.commands.runOnce
import com.hamosad.lib.commands.until
import com.hamosad.lib.commands.withTimeout
import com.hamosad.lib.math.HaRotation2d
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem.stopIntake
import org.firstinspires.ftc.teamcode.subsystems.loader.BallColor
import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderSubsystem
import org.firstinspires.ftc.teamcode.subsystems.loader.Pattern
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem

// TODO: Add a command that collects all 3 balls and cycles through the roulette positions

fun collectCommand(): Command {
    return LoaderSubsystem.positionBallToIntakeCommand(closestBall) meanwhile
            IntakeSubsystem.runIntakeCommand()
} //ALON HOMO veloh hechin command

// TODO: Add a basic shoot command that shoots a ball based on number and not color
// TODO: Add a command that shoots all 3 balls in a specific pattern, no matter what
// TODO: Add a command that shoots all known balls in a color pattern. Have it re-check the current ball colors constantly, instead of once at the start
// TODO: Fix the closest ball command, you know what's wrong in it. And remember it needs to shoot the closest known ball, and if there is no such ball, do nothing
// TODO: Add a command that uses the dynamic shooting to... well... shoot dynamically (april tags and shit)


fun shootColoredBall(color: BallColor, angle: HaRotation2d, speed: AngularVelocity): Command {
    // TODO: Make it use until in tolerance after you fix the shooter commands!!
    return ShooterSubsystem.setHoodAngleAndWheelSpeedCommand(angle, speed) andThen (LoaderSubsystem.shootColorCommand(color) withTimeout(2.0))
}

fun shootBallsInPattern(pattern: Pattern, angle: HaRotation2d, speed: AngularVelocity): Command {
    return shootColoredBall(pattern.pattern[0], angle, speed) andThen
            shootColoredBall(pattern.pattern[1], angle, speed) andThen
            shootColoredBall(pattern.pattern[2], angle, speed)
}

fun shootClosestBall(angle: HaRotation2d, speed: AngularVelocity): Command {
    return ShooterSubsystem.runOnce{ShooterSubsystem.setHoodAngleAndWheelSpeedCommand(angle, speed)} andThen
            LoaderSubsystem.positionAndLoadToShooterCommand(closestBall) andThen LoaderSubsystem.loadToShooterCommand()
}
