package com.hamosad.lib.math

import com.seattlesolvers.solverslib.controller.PIDFController

data class PIDGains(val p: Double = 0.0, val i: Double = 0.0, val d: Double = 0.0, val f: Double = 0.0)

fun PIDGains.toPIDFController(): PIDFController {
    return PIDFController(p, i, d, f)
}