package org.firstinspires.ftc.teamcode.commands

import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.andThen
import com.hamosad.lib.commands.meanwhile
import com.hamosad.lib.commands.raceWith
import com.hamosad.lib.commands.runCommand
import com.hamosad.lib.commands.runOnce
import com.hamosad.lib.commands.until
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.HaRotation2d
import com.hamosad.lib.math.Volts
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterState
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem

fun ShooterSubsystem.maintainHoodAngleCommand(angle: HaRotation2d): Command = ShooterSubsystem.runCommand { updateHoodAngleControl(angle) }

fun ShooterSubsystem.maintainHoodAngleCommand(angle: () -> HaRotation2d): Command = ShooterSubsystem.runCommand { updateHoodAngleControl(angle()) }

fun ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(shooterState: ShooterState) =
    runCommand {
        updateHoodAngleControl(shooterState.angle)
        setWheelMotorsVoltage(shooterState.speedVoltage)
    }

fun ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(shooterState: () -> ShooterState) =
    runCommand {
        updateHoodAngleControl(shooterState().angle)
        setWheelMotorsVoltage(shooterState().speedVoltage)
    }

// TESTING

fun ShooterSubsystem.setWheelMotorsVoltageCommand(voltage: Volts) = ShooterSubsystem.runCommand { setWheelMotorsVoltage(voltage) }
fun ShooterSubsystem.setServoVoltageCommand(voltage: Volts) = ShooterSubsystem.runCommand { setServoVoltage(voltage) }