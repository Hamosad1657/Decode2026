package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.Length
import com.hamosad.lib.math.Rotation2d

private fun interpolateRotation(distance: Length): Rotation2d {
    if (distance.asMeters !in ShooterConstants.DISTANCE_TO_ANGLE_TABLE.keys.first().asMeters..ShooterConstants.DISTANCE_TO_ANGLE_TABLE.keys.last().asMeters)
        return Rotation2d.fromRotations(0.0)

    var foundRange = false
    for (i in 0..(ShooterConstants.DISTANCE_TO_ANGLE_TABLE.size-2)) {
        val key = ShooterConstants.DISTANCE_TO_ANGLE_TABLE.keys.toList()[i]
        val nextKey = ShooterConstants.DISTANCE_TO_ANGLE_TABLE.keys.toList()[i+1]

        if (distance.asMeters > key.asMeters) {
            foundRange = true

            return Rotation2d.fromRotations(ShooterConstants.DISTANCE_TO_ANGLE_TABLE[key]!!.asRotations + (distance.asMeters - key.asMeters)*(
                    ShooterConstants.DISTANCE_TO_ANGLE_TABLE[nextKey]!!.asRotations - ShooterConstants.DISTANCE_TO_ANGLE_TABLE[key]!!.asRotations
                    ) / (
                            nextKey.asMeters - key.asMeters
                            )
            )
        }
    }
    return Rotation2d.fromRotations(0.0)
}

private fun interpolateVelocity(distance: Length): AngularVelocity {
    if (distance.asMeters !in ShooterConstants.DISTANCE_TO_VELOCITY_TABLE.keys.first().asMeters..ShooterConstants.DISTANCE_TO_VELOCITY_TABLE.keys.last().asMeters)
        return AngularVelocity.fromRPM(0.0)

    var foundRange = false
    for (i in 0..(ShooterConstants.DISTANCE_TO_VELOCITY_TABLE.size-2)) {
        val key = ShooterConstants.DISTANCE_TO_VELOCITY_TABLE.keys.toList()[i]
        val nextKey = ShooterConstants.DISTANCE_TO_VELOCITY_TABLE.keys.toList()[i+1]

        if (distance.asMeters > key.asMeters) {
            foundRange = true

            return AngularVelocity.fromRPM(ShooterConstants.DISTANCE_TO_VELOCITY_TABLE[key]!!.asRPM + (distance.asMeters - key.asMeters)*(
                    ShooterConstants.DISTANCE_TO_VELOCITY_TABLE[nextKey]!!.asRPM - ShooterConstants.DISTANCE_TO_VELOCITY_TABLE[key]!!.asRPM
                    ) / (
                    nextKey.asMeters - key.asMeters
                    )
            )
        }
    }
    return AngularVelocity.fromRPM(0.0)
}

fun interpolateDistanceToShooterState(distance: Length): ShooterState {
    return ShooterState(interpolateRotation(distance), interpolateVelocity(distance))
}