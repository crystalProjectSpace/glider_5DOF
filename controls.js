'use strict'

const ControlSystem = function() {
    this.maxDelta = 0
    this.minDelta = 0
    this.currentDelta = 0
    this.deltaSpeed = 0
    this.k_p = 0
    this.k_i = 0
    this.k_d = 0
    this.sparsity = 0

    this.prmPrev = 0
    this.tauPrev = 0

    this.integralError = 0

    this.targetValue = 0
}

ControlSystem.prototype.init = function(minD, maxD, deltaSpeed, sparsity) {
    this.maxDelta = maxD
    this.minDelta = minD
    this.deltaSpeed = deltaSpeed
    this.sparsity = sparsity
}

ControlSystem.prototype.setTarget = function(val) {
    this.targetValue = val
    this.integralError = 0
    this.prmPrev = 0
    this.tauPrev = 0

    this.omegaZprev = 0
}

ControlSystem.prototype.setK_proportional = function(kP) {
    this.k_p = kP
}

ControlSystem.prototype.setK_integral = function(kI) {
    this.k_i = kI
}

ControlSystem.prototype.setK_differential = function(kD) {
    this.k_d = kD
}

ControlSystem.prototype.controlStep = function(state, t) {
    const deltaT = t - this.tauPrev
    if(deltaT < this.sparsity) return
    this.tauPrev = t

    const err = parameter - this.targetValue
    const derivPrm = (parameter - this.prmPrev) / deltaT
    this.prmPrev = parameter
    
    this.integralError += err * this.k_i * deltaT
    const targetControl = err * this.k_p + derivPrm * this.k_d + this.integralError
    let delta = targetControl - this.currentDelta
    if(delta > 0) delta = Math.min(delta, this.deltaSpeed * deltaT)
    if(delta < 0) delta = Math.max(delta, -this.deltaSpeed * deltaT)

    this.currentDelta += delta
    if(this.currentDelta > this.maxDelta) this.currentDelta = this.maxDelta
    if(this.currentDelta < this.minDelta) this.currentDelta = this.minDelta
}

ControlSystem.prototype.pitchControlStep = function(state, t) {
    const deltaT = t - this.tauPrev
    if(deltaT < this.sparsity) return
    this.tauPrev = t
    const thErr = state[1] - this.targetValue
    let kV = 1
    /*if(state[0] > 30) kV = 0.8
    if(state[0] > 60) kV = 0.7
    if(state[0] > 75) kV = 0.5
    if(state[0] > 90) kV = 0.405
    if(state[0] > 105) kV = 0.375
    if(state[0] > 120) kV = 0.315
    if(state[0] > 135) kV = 0.295
    if(state[0] > 145) kV = 0.265*/
    //const alphaErr = state[8] - this.targetValue
    const dOmegaZ = (state[8] - this.omegaZprev) / deltaT
    this.integralError += thErr * this.k_i * deltaT
    const targetControl = (thErr * this.k_p + this.integralError + this.k_d * dOmegaZ) * kV
    let delta = targetControl - this.currentDelta
    
    if(delta > 0) delta = Math.min(delta, this.deltaSpeed * deltaT)
    if(delta < 0) delta = Math.max(delta, -this.deltaSpeed * deltaT)

    this.omegaZprev = state[8]
    this.currentDelta += delta
    if(this.currentDelta > this.maxDelta) this.currentDelta = this.maxDelta
    if(this.currentDelta < this.minDelta) this.currentDelta = this.minDelta
}

ControlSystem.prototype.rollControlStep = function(state, t) {
    const deltaT = t - this.tauPrev
    if(deltaT < this.sparsity) return
    this.tauPrev = t

    const gammaErr = state[9] - this.targetValue
    const dOmegaX = (state[9] - this.prmPrev) / deltaT
    this.integralError += gammaErr * this.k_i * deltaT
    const targetControl = gammaErr * this.k_p + this.integralError + this.k_d * dOmegaX
    let delta = targetControl - this.currentDelta
      
    if(delta > 0) delta = Math.min(delta, this.deltaSpeed * deltaT)
    if(delta < 0) delta = Math.max(delta, -this.deltaSpeed * deltaT)

    this.prmPrev = state[9]
    this.currentDelta += delta
    if(this.currentDelta > this.maxDelta) this.currentDelta = this.maxDelta
    if(this.currentDelta < this.minDelta) this.currentDelta = this.minDelta
}

const GliderControls = function() {
    this.pitchControl = new ControlSystem()
    this.rollControl = new ControlSystem()
    this.pitchControlActive = false
    this.rollControlActive = false
}

/**
 * @description инициализация программы управления в канале тангажа
 * @param {float} minD - минимальный угол отклонения управляющей поверхности, град
 * @param {float} maxD  - максимальный угол отклонения управляющей поверхности, град
 * @param {float} deltaSpeed - быстродействие управляющей поверхности
 * @param {float} sparsity - промежуток времени между управляющими воздействиями
 * @param {float} kP весовой коэффициент пропорциональной составлящей
 * @param {float} kI                     интегральной
 * @param {float} kD                     дифференциальной
 * @param {float} ThTarget - целевой угол (наклон к местному горизонту)
 */
GliderControls.prototype.initPitchCtrl = function(
    minD,
    maxD,
    deltaSpeed,
    sparsity,
    kP,
    kI,
    kD,
    ThTarget
) {
    
    this.pitchControl.init(minD, maxD, deltaSpeed, sparsity)
    this.pitchControl.setTarget(ThTarget)
    this.pitchControl.setK_proportional(kP)
    this.pitchControl.setK_integral(kI)
    this.pitchControl.setK_differential(kD)
}
/**
 * @description инициализация управления в канале курса. Аналогично каналу тангажа, целевая функция - угол крена
 */
GliderControls.prototype.initRollCtrl = function(
    minD,
    maxD,
    deltaSpeed,
    sparsity,
    kP,
    kI,
    kD,
    gammaTarget
) {
    this.rollControl.init(minD, maxD, deltaSpeed, sparsity)
    this.rollControl.setTarget(gammaTarget)
    this.rollControl.setK_proportional(kP)
    this.rollControl.setK_integral(kI)
    this.rollControl.setK_differential(kD)
}

GliderControls.prototype.checkControls = function(state, t) {
    this.pitchControlActive && this.pitchControl.pitchControlStep(state, t)
    this.rollControlActive && this.rollControl.rollControlStep(state, t)
}

GliderControls.prototype.deltaPitch = function(){
    return this.pitchControl.currentDelta
}

GliderControls.prototype.deltaRoll = function(){
    return this.rollControl.currentDelta
}

GliderControls.prototype.setBasePitch = function(deltaElevator){
    this.pitchControl.currentDelta = deltaElevator
}

GliderControls.prototype.setBaseRoll = function(deltaAilerons){
    this.rollControl.currentDelta = deltaAilerons
}

GliderControls.prototype.togglePitch = function(pitchState){
    this.pitchControlActive = pitchState
}

GliderControls.prototype.toggleRoll = function(rollState){
    this.rollControlActive = rollState
}

module.exports = {
    GliderControls
}