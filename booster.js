'use strict'

const { Interp1D } = require('./utils.js')

const Booster = function() {
    this.mFuel = 0
    this.mFuelCurrent = 0
    this.jRelative = 0
    this.thrustDiagram = new Interp1D()
    this.active = false
}

Booster.prototype.init = function(mFuel, jRelative, TX, MX) {
    this.mFuel = mFuel
    this.mFuelCurrent = this.mFuel
    this.jRelative = jRelative
    this.thrustDiagram.init(TX, MX, 0)
    this.active = true
   
    return this
}

Booster.prototype.getThrust = function(t) {
    
    if(this.mFuelCurrent <=0 ) return [0, 0]
    this.thrustDiagram.checkIndex(t);
    const dM = this.thrustDiagram.interp(t)
    if(dM === 0) return [0, 0]
    const R = dM * this.jRelative
    return [dM, R]
}

Booster.prototype.updFuel = function(t, dT) {    
    if(this.mFuelCurrent <=0 ) return
    this.thrustDiagram.checkIndex(t);
    const dM = this.thrustDiagram.interp(t)
    if(dM > 0) this.mFuelCurrent -= (dM * dT)
}

const ElectricMotor = function() {
    this.totalCharge = 0
    this.thrustMax = 0
    this.drainMax = 0
    this.currentCharge = 0
    this.active = false
    this.drainDiagram = new Interp1D()
}

ElectricMotor.prototype.init = function(totalCharge, thrustMax, drainMax, TX, DX) {
    this.totalCharge = totalCharge
    this.thrustMax = thrustMax
    this.drainMax = drainMax
    this.currentCharge = this.totalCharge
    this.drainDiagram.init(TX, DX, 0)

    this.active = true
   
    return this
}

ElectricMotor.prototype.getThrust = function(t) {
    
    if(this.currentCharge <=0 ) return 0
    this.drainDiagram.checkIndex(t)
    const currentDrain = this.drainDiagram.interp(t)
    return this.thrustMax * currentDrain
}

ElectricMotor.prototype.updCharge = function(t, dT) {
    if(this.currentCharge <=0 ) return
    this.drainDiagram.checkIndex(t)
    const currentDrain = this.drainDiagram.interp(t)
    if(currentDrain <= 0) return
    this.currentCharge -= this.drainMax * currentDrain * dT        
}

module.exports = { Booster, ElectricMotor }