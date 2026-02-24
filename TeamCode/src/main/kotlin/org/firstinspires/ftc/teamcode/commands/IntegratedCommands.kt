package org.firstinspires.ftc.teamcode.commands


import com.seattlesolvers.solverslib.geometry.Translation2d
import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.andThen
import com.hamosad.lib.commands.meanwhile
import com.hamosad.lib.commands.raceWith
import com.hamosad.lib.commands.runOnce
import com.hamosad.lib.commands.until
import com.hamosad.lib.commands.withTimeout
import com.hamosad.lib.math.Length
import com.hamosad.lib.math.Seconds
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.loader.BallColor
import org.firstinspires.ftc.teamcode.subsystems.loader.BallPattern
import org.firstinspires.ftc.teamcode.subsystems.loader.ColorPattern
import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderSubsystem
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterState
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.shooter.interpolateDistanceToShooterState

// -- INTAKE --

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

// -- NON COLOR DEPENDANT SHOOTING --

fun shootBallCommand(ball: Ball, shooterState: ShooterState): Command =
    (ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(shooterState) raceWith (
            (LoaderSubsystem.positionBallToShooterCommand(ball) until { LoaderSubsystem.isAtSetpoint && ShooterSubsystem.isWithinTolerance }) andThen
                    (LoaderSubsystem.loadToShooterCommand())
            )
    )

fun shootBallCommand(ball: Ball, shooterState: () -> ShooterState): Command =
    (ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(shooterState) raceWith (
            (LoaderSubsystem.positionBallToShooterCommand(ball) until { LoaderSubsystem.isAtSetpoint && ShooterSubsystem.isWithinTolerance }) andThen
                    (LoaderSubsystem.loadToShooterCommand())
            )
    )

fun shootClosestBallCommand(shooterState: ShooterState): Command =
    (ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(shooterState) raceWith (
            (LoaderSubsystem.positionBallToShooterCommand(LoaderSubsystem.closestBallToShooter) until { LoaderSubsystem.isAtSetpoint && ShooterSubsystem.isWithinTolerance }) andThen
                    (LoaderSubsystem.loadToShooterCommand())
            )
            )

fun shootClosestBallCommand(shooterState: () -> ShooterState): Command =
    (ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(shooterState) raceWith (
            (LoaderSubsystem.positionBallToShooterCommand(LoaderSubsystem.closestBallToShooter) until { LoaderSubsystem.isAtSetpoint && ShooterSubsystem.isWithinTolerance }) andThen
                    (LoaderSubsystem.loadToShooterCommand())
            )
            )

fun shootAllBallsInPatternCommand(pattern: BallPattern, shooterState: () -> ShooterState, shootingTimeout: Seconds): Command =
    (ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(shooterState) raceWith (
            ((LoaderSubsystem.positionBallToShooterCommand(pattern.pattern[0]) withTimeout(3.5)) andThen
                    (LoaderSubsystem.loadToShooterCommand() withTimeout (shootingTimeout)))
                    andThen ((LoaderSubsystem.positionBallToShooterCommand(pattern.pattern[1]) withTimeout(3.5) ) andThen
                    (LoaderSubsystem.loadToShooterCommand() withTimeout(shootingTimeout))) andThen
                    ((LoaderSubsystem.positionBallToShooterCommand(pattern.pattern[2] ) withTimeout(3.5)) andThen
                            (LoaderSubsystem.loadToShooterCommand() withTimeout(shootingTimeout)))
            )
    )

// COLOR DEPENDANT SHOOTING

fun shootClosestKnownBallCommand(shooterState: ShooterState, shootingTimeout: Seconds): Command {
    var closestKnownBall: Ball = Ball.BALL_1
    var hasKnownBall = false
    if (LoaderSubsystem.returnBallColor(LoaderSubsystem.furthestBallFromShooter) != BallColor.UNKNOWN) {
        closestKnownBall = LoaderSubsystem.furthestBallFromShooter
        hasKnownBall = true
    }
    if (LoaderSubsystem.returnBallColor(LoaderSubsystem.middleBallFromShooter) != BallColor.UNKNOWN) {
        closestKnownBall = LoaderSubsystem.middleBallFromShooter
        hasKnownBall = true
    }
    if (LoaderSubsystem.returnBallColor(LoaderSubsystem.closestBallToShooter) != BallColor.UNKNOWN) {
        closestKnownBall = LoaderSubsystem.closestBallToShooter
        hasKnownBall = true
    }
    if (!hasKnownBall) return LoaderSubsystem.runOnce {}

    return (ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(shooterState) raceWith (
            (LoaderSubsystem.positionBallToShooterCommand(closestKnownBall) until { LoaderSubsystem.isAtSetpoint && ShooterSubsystem.isWithinTolerance }) andThen
                    (LoaderSubsystem.loadToShooterCommand() withTimeout(shootingTimeout))
            )
            )
}

fun shootClosestKnownBallCommand(shooterState: () -> ShooterState, shootingTimeout: Seconds): Command {
    var closestKnownBall: Ball = Ball.BALL_1
    var hasKnownBall = false
    if (LoaderSubsystem.returnBallColor(LoaderSubsystem.furthestBallFromShooter) != BallColor.UNKNOWN) {
        closestKnownBall = LoaderSubsystem.furthestBallFromShooter
        hasKnownBall = true
    }
    if (LoaderSubsystem.returnBallColor(LoaderSubsystem.middleBallFromShooter) != BallColor.UNKNOWN) {
        closestKnownBall = LoaderSubsystem.middleBallFromShooter
        hasKnownBall = true
    }
    if (LoaderSubsystem.returnBallColor(LoaderSubsystem.closestBallToShooter) != BallColor.UNKNOWN) {
        closestKnownBall = LoaderSubsystem.closestBallToShooter
        hasKnownBall = true
    }
    if (!hasKnownBall) return LoaderSubsystem.runOnce {}

    return (ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(shooterState) raceWith (
            (LoaderSubsystem.positionBallToShooterCommand(closestKnownBall) until { LoaderSubsystem.isAtSetpoint && ShooterSubsystem.isWithinTolerance }) andThen
                    (LoaderSubsystem.loadToShooterCommand() withTimeout(shootingTimeout))
            )
            )
}

fun shootColoredBallCommand(color: BallColor, shooterState: ShooterState, shootingTimeout: Seconds): Command {
    if (LoaderSubsystem.returnBallColor(Ball.BALL_1) != color && LoaderSubsystem.returnBallColor(Ball.BALL_2) != color &&
        LoaderSubsystem.returnBallColor(Ball.BALL_3) != color) {
        return LoaderSubsystem.runOnce {}
    }

    return (ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(shooterState) raceWith (
            (LoaderSubsystem.positionColorToShooterCommand(color) until { LoaderSubsystem.isAtSetpoint && ShooterSubsystem.isWithinTolerance }) andThen
                    (LoaderSubsystem.loadToShooterCommand() withTimeout (shootingTimeout))
            )
            )
}

fun shootColoredBallCommand(color: BallColor, shooterState: () -> ShooterState, shootingTimeout: Seconds): Command {
    if (LoaderSubsystem.returnBallColor(Ball.BALL_1) != color && LoaderSubsystem.returnBallColor(Ball.BALL_2) != color &&
        LoaderSubsystem.returnBallColor(Ball.BALL_3) != color) {
        return LoaderSubsystem.runOnce {}
    }

    return (ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(shooterState) raceWith (
            (LoaderSubsystem.positionColorToShooterCommand(color) until { LoaderSubsystem.isAtSetpoint && ShooterSubsystem.isWithinTolerance }) andThen
                    (LoaderSubsystem.loadToShooterCommand() withTimeout (shootingTimeout))
            )
            )
}

fun shootAllBallsInColorPatternCommand(colorPattern: ColorPattern, shooterState: () -> ShooterState, shootingTimeout: Seconds): Command =
    (ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(shooterState) raceWith (
            ((LoaderSubsystem.positionColorToShooterCommand(colorPattern.pattern[0]) until { LoaderSubsystem.isAtSetpoint && ShooterSubsystem.isWithinTolerance }) andThen
                    (LoaderSubsystem.loadToShooterCommand() withTimeout(shootingTimeout)))
                    andThen ((LoaderSubsystem.positionColorToShooterCommand(colorPattern.pattern[1]) until { LoaderSubsystem.isAtSetpoint && ShooterSubsystem.isWithinTolerance }) andThen
                    (LoaderSubsystem.loadToShooterCommand() withTimeout(shootingTimeout))) andThen
                    ((LoaderSubsystem.positionColorToShooterCommand(colorPattern.pattern[2]) until { LoaderSubsystem.isAtSetpoint && ShooterSubsystem.isWithinTolerance }) andThen
                            (LoaderSubsystem.loadToShooterCommand() withTimeout(shootingTimeout)))
            )
            )

// DYNAMIC SHOOTING

val BLUE_GATE_POSITION = Translation2d(0.0, 3.556)
val RED_GATE_POSITION = Translation2d(3.556, 3.556)

//fun getCurrentDynamicShooterState(isRedAlliance: Boolean): ShooterState {
//    return interpolateDistanceToShooterState(Length.fromMeters(
//        MecanumSubsystem.robotPose.translation2d.toFTCLibT2d().getDistance(if (isRedAlliance) RED_GATE_POSITION else BLUE_GATE_POSITION)
//    ))
//}