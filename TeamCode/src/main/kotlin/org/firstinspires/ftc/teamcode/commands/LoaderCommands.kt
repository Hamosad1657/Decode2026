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

fun LoaderSubsystem.positionAndLoadToShooterCommand(ball: Ball): Command {
    return (positionBallToShootCommand(ball) until { isAtSetpoint }) andThen
            loadToShooterCommand() finallyDo {
                when (returnBallColor(ball)) {
                ball1Color -> ball1Color = BallColor.UNKNOWN
                ball2Color -> ball2Color = BallColor.UNKNOWN
                ball3Color -> ball3Color = BallColor.UNKNOWN
                else -> {}
            } }
}


fun LoaderSubsystem.positionAndLoadColorToShooterCommand(color: BallColor): Command =
    if (color == BallColor.UNKNOWN) runOnce {  }
    else if (returnBallColor(closestBallToShooter) == color) {
        positionAndLoadToShooterCommand(Ball.BALL_1)
    } else if (returnBallColor(middleBallFromShooter) == color) {
        positionAndLoadToShooterCommand(Ball.BALL_2)
    } else if (returnBallColor(furthestBallFromShooter) == color) {
        positionAndLoadToShooterCommand((Ball.BALL_3))
    } else {
        runOnce {  }
    }
//SPINNNNN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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