package com.hamosad.lib.components.sensors

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.TouchSensor

class HaTouchSensor(name: String, hardwareMap: HardwareMap) {
    private val sensor: TouchSensor = hardwareMap.get(TouchSensor::class.java, name)
    val isTouched get() = sensor.isPressed
}