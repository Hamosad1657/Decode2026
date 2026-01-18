package org.firstinspires.ftc.teamcode.commands


import com.hamosad.lib.commands.Command
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.commands.andThen
import com.hamosad.lib.commands.meanwhile
import com.hamosad.lib.commands.raceWith
import com.hamosad.lib.commands.runOnce
import com.hamosad.lib.commands.until
import com.hamosad.lib.commands.withTimeout
import com.hamosad.lib.math.HaRotation2d
import com.hamosad.lib.math.Seconds
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem.stopIntake
import org.firstinspires.ftc.teamcode.subsystems.loader.BallColor
import org.firstinspires.ftc.teamcode.subsystems.loader.BallPattern
import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderSubsystem
import org.firstinspires.ftc.teamcode.subsystems.loader.Pattern
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem

// It's quite shrimple
fun collectSimpleCommand(slotTimeout: Seconds): Command =
    IntakeSubsystem.runIntakeCommand() meanwhile (
            (LoaderSubsystem.positionBallToIntakeCommand(Ball.BALL_1) withTimeout(slotTimeout)) andThen
                    (LoaderSubsystem.positionBallToIntakeCommand(Ball.BALL_2) withTimeout(slotTimeout)) andThen
                        (LoaderSubsystem.positionBallToIntakeCommand(Ball.BALL_3) withTimeout(slotTimeout))
    )

fun collectAdvancedCommand(slotTimeout: Seconds): Command =
    IntakeSubsystem.runIntakeCommand() meanwhile (
            (LoaderSubsystem.positionBallToIntakeCommand(LoaderSubsystem.closestBallToIntake) withTimeout(slotTimeout)) andThen
                    (LoaderSubsystem.positionBallToIntakeCommand(LoaderSubsystem.middleBallFromIntake) withTimeout(slotTimeout)) andThen
                    (LoaderSubsystem.positionBallToIntakeCommand(LoaderSubsystem.furthestBallFromIntake) withTimeout(slotTimeout))
            )

// TODO: Add a command that shoots all 3 balls in a specific pattern, no matter what
// TODO: Add a command that shoots all known balls in a color pattern. Have it re-check the current ball colors constantly, instead of once at the start
// TODO: Fix the closest ball command, you know what's wrong in it. And remember it needs to shoot the closest known ball, and if there is no such ball, do nothing
// TODO: Add a command that uses the dynamic shooting to... well... shoot dynamically (april tags and shit)

fun shootBallCommand(ball: Ball, angle: HaRotation2d, speed: AngularVelocity): Command =
    (ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(angle, speed) raceWith (
            (LoaderSubsystem.positionBallToShooterCommand(ball) until { LoaderSubsystem.isAtSetpoint && ShooterSubsystem.isWithinTolerance }) andThen
                    (LoaderSubsystem.loadToShooterCommand() withTimeout(3.0))
            )
    )

fun shootBallCommand(ball: Ball, angle: () -> HaRotation2d, speed: () -> AngularVelocity): Command =
    (ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(angle, speed) raceWith (
            (LoaderSubsystem.positionBallToShooterCommand(ball) until { LoaderSubsystem.isAtSetpoint && ShooterSubsystem.isWithinTolerance }) andThen
                    (LoaderSubsystem.loadToShooterCommand() withTimeout(3.0))
            )
    )

fun shootAllBallsInPatternCommand(pattern: BallPattern, angle: () -> HaRotation2d, speed: () -> AngularVelocity): Command =
    (ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(angle, speed) raceWith (
            ((LoaderSubsystem.positionBallToShooterCommand(pattern.pattern[0]) until { LoaderSubsystem.isAtSetpoint && ShooterSubsystem.isWithinTolerance }) andThen
                    (LoaderSubsystem.loadToShooterCommand() withTimeout(3.0)))
                    andThen ((LoaderSubsystem.positionBallToShooterCommand(pattern.pattern[1]) until { LoaderSubsystem.isAtSetpoint && ShooterSubsystem.isWithinTolerance }) andThen
                    (LoaderSubsystem.loadToShooterCommand() withTimeout(3.0))) andThen
                    ((LoaderSubsystem.positionBallToShooterCommand(pattern.pattern[2]) until { LoaderSubsystem.isAtSetpoint && ShooterSubsystem.isWithinTolerance }) andThen
                            (LoaderSubsystem.loadToShooterCommand() withTimeout(3.0)))
            )
    )

fun shootColoredBallCommand(color: BallColor, angle: HaRotation2d, speed: AngularVelocity): Command =
    (ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(angle, speed) raceWith (
            (LoaderSubsystem.positionColorToShooterCommand(color) until { LoaderSubsystem.isAtSetpoint && ShooterSubsystem.isWithinTolerance }) andThen
                    (LoaderSubsystem.loadToShooterCommand() withTimeout(3.0))
            )
            )

//fun shootClosestBall(angle: HaRotation2d, speed: AngularVelocity): Command {
//    return ShooterSubsystem.runOnce{ShooterSubsystem.setHoodAngleAndWheelSpeedCommand(angle, speed)} andThen
//            LoaderSubsystem.positionAndLoadToShooterCommand(closestBall) andThen LoaderSubsystem.loadToShooterCommand()
//}
