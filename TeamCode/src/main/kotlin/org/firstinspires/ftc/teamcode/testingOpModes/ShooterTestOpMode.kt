package org.firstinspires.ftc.teamcode.testingOpModes

import com.hamosad.lib.opModes.CommandOpModeTeleop
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem
import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.components.Controllers.HaCommandController
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.HaRotation2d
import com.hamosad.lib.math.Length
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.maintainHoodAngleAndWheelSpeedCommand
import org.firstinspires.ftc.teamcode.commands.maintainHoodAngleCommand
import org.firstinspires.ftc.teamcode.commands.maintainWheelSpeedCommand
import org.firstinspires.ftc.teamcode.commands.setServoVoltageCommand
import org.firstinspires.ftc.teamcode.commands.setWheelMotorsVoltageCommand
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterState
import org.firstinspires.ftc.teamcode.subsystems.shooter.interpolateDistanceToShooterState

@TeleOp
class ShooterTestOpMode: CommandOpModeTeleop() {
    override var subsystemsToUse: List<Subsystem> = listOf(ShooterSubsystem)
    val controller = HaCommandController({ super.gamepad1 }, 0.03, 1)

    override fun configureDefaultCommands() {
        ShooterSubsystem.defaultCommand = ShooterSubsystem.maintainWheelSpeedCommand(AngularVelocity.fromRPS(0.0))
    }

    override fun configureBindings() {
        controller.r2Pressed().whileTrue(ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(
            ShooterState(HaRotation2d.fromDegrees(25.0),
                AngularVelocity.fromRPS(90.0)
            )))
        controller.l2Pressed().whileTrue(ShooterSubsystem.setWheelMotorsVoltageCommand(9.0))
        controller.r1().whileTrue(ShooterSubsystem.maintainHoodAngleCommand(HaRotation2d.fromDegrees(50.0)))
        controller.l1().whileTrue(ShooterSubsystem.maintainWheelSpeedCommand(AngularVelocity.fromRPS(50.0)))
        controller.cross().whileTrue(ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(interpolateDistanceToShooterState(
            Length.fromMeters(1.0))))
        controller.circle().whileTrue(ShooterSubsystem.setServoVoltageCommand(6.0))
    }

}