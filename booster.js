'use strict'

const { Interp1D } = require('./utils.js')

const Booster = function() {
    this.mFuel = 0
    this.mFuelCurrent = 0
    this.jRelative = 0
    this.thrustDiagram = new Interp1D()
}

Booster.prototype.init = function(mFuel, jRelative, TX, MX) {
    this.mFuel = mFuel
    this.mFuelCurrent = this.mFuel
    this.jRelative = jRelative
    this.thrustDiagram.init(TX, MX, 0)
   
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

module.exports = { Booster }