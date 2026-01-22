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
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem

fun ShooterSubsystem.maintainHoodAngleCommand(angle: HaRotation2d): Command = ShooterSubsystem.runCommand { updateHoodAngleControl(angle) }

fun ShooterSubsystem.maintainHoodAngleCommand(angle: () -> HaRotation2d): Command = ShooterSubsystem.runCommand { updateHoodAngleControl(angle()) }

fun ShooterSubsystem.maintainWheelSpeedCommand(speed: AngularVelocity): Command = ShooterSubsystem.runCommand { updateShooterVelocityControl(speed) }

fun ShooterSubsystem.maintainWheelSpeedCommand(speed: () -> AngularVelocity): Command = ShooterSubsystem.runCommand { updateShooterVelocityControl(speed()) }


fun ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(angle: HaRotation2d, speed: AngularVelocity) =
    runCommand {
        updateHoodAngleControl(angle)
        updateShooterVelocityControl(speed)
    }

fun ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(angle: () -> HaRotation2d, speed: () -> AngularVelocity) =
    runCommand {
        updateHoodAngleControl(angle())
        updateShooterVelocityControl(speed())
    }

// TESTING

fun ShooterSubsystem.setWheelMotorsVoltageCommand(voltage: Volts) = ShooterSubsystem.runCommand { setWheelMotorsVoltage(voltage) }