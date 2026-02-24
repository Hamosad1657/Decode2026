package org.firstinspires.ftc.teamcode.testingOpModes

import com.hamosad.lib.opModes.CommandOpModeTeleop
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem
import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.commands.runCommand
import com.hamosad.lib.commands.runOnce
import com.hamosad.lib.components.Controllers.HaCommandController
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.HaRotation2d
import com.hamosad.lib.math.Length
import com.hamosad.lib.math.Volts
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.loadToShooterCommand
import org.firstinspires.ftc.teamcode.commands.maintainHoodAngleAndWheelSpeedCommand
import org.firstinspires.ftc.teamcode.commands.maintainHoodAngleCommand
import org.firstinspires.ftc.teamcode.commands.setServoVoltageCommand
import org.firstinspires.ftc.teamcode.commands.setWheelMotorsVoltageCommand
import org.firstinspires.ftc.teamcode.commands.stopLoadingToShooterCommand
import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderSubsystem
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterState
//import org.firstinspires.ftc.teamcode.subsystems.shooter.interpolateDistanceToShooterState

@TeleOp
class ShooterTestOpMode: CommandOpModeTeleop() {
    override var subsystemsToUse: List<Subsystem> = listOf(ShooterSubsystem, LoaderSubsystem)
    val controller = HaCommandController({ super.gamepad1 }, 0.03, 1)
    val controller2 = HaCommandController({ super.gamepad2 }, 0.03, 1)

    var currentAngle = HaRotation2d.fromDegrees(0.0)
    var currentVoltage: Volts = 0.0

    override fun configureDefaultCommands() {
        ShooterSubsystem.defaultCommand = ShooterSubsystem.setWheelMotorsVoltageCommand(0.0)
        LoaderSubsystem.defaultCommand = LoaderSubsystem.stopLoadingToShooterCommand()
    }

    override fun configureBindings() {
        controller.r2Pressed().whileTrue(ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(
            ShooterState(HaRotation2d.fromDegrees(25.0),
                10.0
            )))

        controller.l2Pressed().whileTrue(ShooterSubsystem.setWheelMotorsVoltageCommand(12.0))

        controller.r1().whileTrue(ShooterSubsystem.maintainHoodAngleCommand(HaRotation2d.fromDegrees(50.0)))
        controller.circle().whileTrue(ShooterSubsystem.maintainHoodAngleCommand(HaRotation2d.fromDegrees(0.0)))
        controller.triangle().whileTrue(ShooterSubsystem.maintainHoodAngleCommand(HaRotation2d.fromDegrees(25.0)))
        controller.square().whileTrue(ShooterSubsystem.maintainHoodAngleCommand(HaRotation2d.fromDegrees(50.0)))
        controller.cross().whileTrue(ShooterSubsystem.maintainHoodAngleCommand(HaRotation2d.fromDegrees(70.0)))

        controller2.dpadUp().onTrue(ShooterSubsystem.runOnce { if (currentAngle.asDegrees < 78.0) currentAngle =
            HaRotation2d.fromDegrees(currentAngle.asDegrees + 2.0) })
        controller2.dpadDown().onTrue(ShooterSubsystem.runOnce { if (currentAngle.asDegrees > 2.0) currentAngle =
            HaRotation2d.fromDegrees(currentAngle.asDegrees - 2.0) })

        controller2.dpadRight().onTrue(ShooterSubsystem.runOnce { if (currentVoltage <= 11.0) currentVoltage++ })
        controller2.dpadLeft().onTrue(ShooterSubsystem.runOnce { if (currentVoltage >= 1.0) currentVoltage-- })

        controller2.r2Pressed().whileTrue(ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand { ShooterState(currentAngle, currentVoltage) })
        controller2.l1().whileTrue(LoaderSubsystem.runCommand { LoaderSubsystem.loadToShooter() })
    }

}