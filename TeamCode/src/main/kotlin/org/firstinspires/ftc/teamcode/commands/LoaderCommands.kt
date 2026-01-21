package org.firstinspires.ftc.teamcode.commands

import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderConstants as Constants
import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.andThen
import com.hamosad.lib.commands.finallyDo
import com.hamosad.lib.commands.runCommand
import com.hamosad.lib.commands.runOnce
import com.hamosad.lib.commands.until
import com.hamosad.lib.math.HaRotation2d
import com.hamosad.lib.math.Volts
import org.firstinspires.ftc.teamcode.subsystems.loader.BallColor
import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderSubsystem

enum class Ball {
    BALL_1,
    BALL_2,
    BALL_3,
}

/*
    Commands needed:
    General:
    - Hold roulette position

    Shooting:
    - Position ball to shoot
    - Load to shooter
    - Position and load to shooter
    - Shoot green ball
    - Shoot purple ball
    - Shoot closest

    Intaking:
    - Position ball to intake
 */

// GENERAL
fun LoaderSubsystem.holdRoulettePositionCommand(position: HaRotation2d): Command = runCommand {
    updateRouletteControl(position)
    stopLoadingToShooter()
}

// SHOOTING
fun LoaderSubsystem.positionBallToShooterCommand(ball: Ball): Command = holdRoulettePositionCommand(
    when (ball) {
        Ball.BALL_1 -> Constants.BALL_1_AT_SHOOTER
        Ball.BALL_2 -> Constants.BALL_2_AT_SHOOTER
        Ball.BALL_3 -> Constants.BALL_3_AT_SHOOTER
    }
)

fun LoaderSubsystem.loadToShooterCommand(): Command = runCommand {
    updateRouletteControl()
    loadToShooter()
} finallyDo { stopLoadingToShooter() }

fun LoaderSubsystem.positionColorToShooterCommand(color: BallColor): Command =
    when (color) {
        BallColor.UNKNOWN -> runOnce { }
        returnBallColor(closestBallToShooter) -> {
            positionBallToShooterCommand(Ball.BALL_1)
        }
        returnBallColor(middleBallFromShooter) -> {
            positionBallToShooterCommand(Ball.BALL_2)
        }
        returnBallColor(furthestBallFromShooter) -> {
            positionBallToShooterCommand(Ball.BALL_3)
        }
        else -> {
            runOnce { }
        }
    }

// SPIN
fun LoaderSubsystem.setServoVoltageCommand(volt: Volts): Command = runCommand {
    setServoVoltage(volt)
}

// INTAKING
fun LoaderSubsystem.positionBallToIntakeCommand(ball: Ball): Command =
    holdRoulettePositionCommand(
        when (ball) {
            Ball.BALL_1 -> Constants.BALL_1_AT_INTAKE
            Ball.BALL_2 -> Constants.BALL_2_AT_INTAKE
            Ball.BALL_3 -> Constants.BALL_3_AT_INTAKE
        }
    )