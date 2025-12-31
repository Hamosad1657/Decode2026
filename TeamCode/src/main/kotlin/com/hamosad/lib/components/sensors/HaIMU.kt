package com.hamosad.lib.components.sensors

import com.hamosad.lib.math.HaRotation2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference

class HaIMU(name: String, hardwareMap: HardwareMap) {
    private val imu: IMU = hardwareMap.get(IMU::class.java, name)

    init {
        imu.resetYaw()
    }

    val currentYaw: HaRotation2d get() =
        HaRotation2d.fromDegrees(
            imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle.toDouble()
        )
    val currentPitch: HaRotation2d get() =
        HaRotation2d.fromDegrees(
            imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle.toDouble()
        )
    val currentRoll: HaRotation2d get() =
        HaRotation2d.fromDegrees(
            imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle.toDouble()
        )

    fun resetYaw() {
        imu.resetYaw()
    }
}