package org.firstinspires.ftc.teamcode.subsystems.mecanum

import com.pedropathing.geometry.Pose
import com.pedropathing.localization.Localizer

class MecanumLocalizer: Localizer {
    override fun getPose(): Pose? {
        return MecanumSubsystem.robotPose.toPedroPose()
    }

    override fun getIMUHeading(): Double {
        return MecanumSubsystem.currentAbsoluteAngle.asRadians
    }

    override fun resetIMU() {
        MecanumSubsystem.resetGyro()
    }

    override fun getForwardMultiplier(): Double {
        return 1.0
    }

    override fun getLateralMultiplier(): Double {
        return 1.0
    }

    override fun getTurningMultiplier(): Double {
        return 1.0
    }

    override fun getTotalHeading(): Double {
        return MecanumSubsystem.currentAngle.asRadians
    }
}