package org.firstinspires.ftc.teamcode.commands

import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderConstants as Constants
import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.andThen
import com.hamosad.lib.commands.finallyDo
import com.hamosad.lib.commands.runCommand
import com.hamosad.lib.commands.runOnce
import com.hamosad.lib.commands.until
import com.hamosad.lib.math.Rotation2d
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
fun LoaderSubsystem.holdRoulettePositionCommand(position: Rotation2d): Command = runCommand {
    updateRouletteControl(position)
    stopLoadingToShooter()
}

// SHOOTING
fun LoaderSubsystem.positionBallToShootCommand(ball: Ball): Command = holdRoulettePositionCommand(
    when (ball) {
        Ball.BALL_1 -> Constants.BALL_1_AT_SHOOTER
        Ball.BALL_2 -> Constants.BALL_2_AT_SHOOTER
        Ball.BALL_3 -> Constants.BALL_3_AT_SHOOTER
    }
)

fun LoaderSubsystem.loadToShooterCommand(): Command = runCommand {
    loadToShooter()
} finallyDo { stopLoadingToShooter() }

fun LoaderSubsystem.positionAndLoadToShooterCommand(ball: Ball): Command =
    (positionBallToShootCommand(ball) until { isAtSetpoint }) andThen
            loadToShooterCommand()

fun LoaderSubsystem.shootColorCommand(color: BallColor): Command =
    if (color == BallColor.UNKNOWN) runOnce {  }
    else if (ball1Color == color) {
        positionAndLoadToShooterCommand(Ball.BALL_1)
    } else if (ball2Color == color) {
        positionAndLoadToShooterCommand(Ball.BALL_2)
    } else if (ball3Color == color) {
        positionAndLoadToShooterCommand((Ball.BALL_3))
    } else {
        runOnce {  }
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