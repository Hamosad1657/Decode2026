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

// GENERAL
fun LoaderSubsystem.holdRoulettePositionCommand(position: HaRotation2d): Command = runCommand {
    updateRouletteControl(position)
    stopLoadingToShooter()
}

fun LoaderSubsystem.holdRoulettePositionCommand(position: () -> HaRotation2d): Command = runCommand {
    updateRouletteControl(position())
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

fun LoaderSubsystem.positionBallToShooterCommand(ball: () -> Ball): Command = holdRoulettePositionCommand {
    when (ball()) {
        Ball.BALL_1 -> Constants.BALL_1_AT_SHOOTER
        Ball.BALL_2 -> Constants.BALL_2_AT_SHOOTER
        Ball.BALL_3 -> Constants.BALL_3_AT_SHOOTER
    }
}

fun LoaderSubsystem.loadToShooterCommand(): Command = runCommand {
    updateRouletteControl()
    loadToShooter()
} finallyDo { stopLoadingToShooter() }

fun LoaderSubsystem.stopLoadingToShooterCommand(): Command = runCommand {
    stopLoadingToShooter()
}

fun LoaderSubsystem.positionColorToShooterCommand(color: BallColor): Command =
    positionBallToShooterCommand {
        when (color) {
            returnBallColor(closestBallToShooter) -> closestBallToShooter
            returnBallColor(middleBallFromShooter) -> middleBallFromShooter
            returnBallColor(furthestBallFromShooter) -> furthestBallFromShooter
            else -> Ball.BALL_1
        }
    }

// SPIN
fun LoaderSubsystem.setServoVoltageCommand(volt: Volts): Command = runCommand {
    setServoVoltage(volt)
}

fun LoaderSubsystem.manualControlCommand(leftJoyX: () -> Double, isR1Pressed: () -> Boolean) = runCommand {
    setServoVoltage(leftJoyX())
    if (isR1Pressed()) {
        loadToShooter()
    } else {
        stopLoadingToShooter()
    }
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