package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.purepursuit.Path
import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.runCommand
import com.hamosad.lib.math.AngularVelocity
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants as Constants
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem

fun MecanumSubsystem.angularVelocityDriveCommand(
    fieldRelative: Boolean,
    leftJoyX: () -> Double,
    leftJoyY: () -> Double,
    rightJoyX: () -> Double
) = MecanumSubsystem.runCommand {
    drive(fieldRelative, ChassisSpeeds(
        leftJoyY() * Constants.MAX_CHASSIS_SPEED,
        -leftJoyX() * Constants.MAX_CHASSIS_SPEED,
        -AngularVelocity.fromRPS(rightJoyX() * Constants.MAX_CHASSIS_ANGULAR_VELOCITY.asRPS).asRadPS,
    ))
}

fun MecanumSubsystem.aimAtGoalAndMoveWithVision(leftJoyX: () -> Double, leftJoyY: () -> Double): Command = MecanumSubsystem.runCommand {
    var errorRad: Double? = null
     if (apriltagCamera?.isTagDetected(20) ?: false || apriltagCamera?.isTagDetected(24) ?: false) {
        val detections = apriltagCamera!!.allTargets
         if (detections != null) {
             for (detection in detections) {
                 if (detection?.id == 20 || detection?.id == 24) {
                     errorRad = detection.ftcPose.yaw
                 }
             }
         }
     }
    drive(true, ChassisSpeeds(
        leftJoyY() * Constants.MAX_CHASSIS_SPEED,
        -leftJoyX() * Constants.MAX_CHASSIS_SPEED,
        (if (Constants.INVERT_YAW_FOLLOWING) -1 else 1) * (if (errorRad != null) yawPIDController.calculate(errorRad, 0.0) else 0.0)
        )
    )
}

fun MecanumSubsystem.purePursuitFollowPath(path: Path): Command = MecanumSubsystem.runCommand {
    val chassisSpeeds = path.loop(robotPose.translation2d.x, robotPose.translation2d.y, robotPose.rotation2d.asDegrees)
    drive(true, ChassisSpeeds(chassisSpeeds[1], chassisSpeeds[2], chassisSpeeds[3]))
}