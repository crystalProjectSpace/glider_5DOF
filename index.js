'use strict'

const { integrate, printResult, R2G } = require('./trajectory.js')
const { MX, AX, DX, CYA, CXA, MZ, dMZ, dMX } = require('./init_data.json') // импорт ИД
const { GliderControls } = require('./controls.js')
const { Interp1D, Interp2D } = require('./utils.js')
const { argv } = require('node:process')

const getValue = function(raw) {
    return +raw.split('=')[1]
}

const ANGLE_INDICES = [1, 2, 3, 4, 8, 9]

const setTrajectoryExperiment = function() {
    
    const [stateStr, deltaPitch, deltaRoll, tauMax, dT, resSparse] = argv.slice(2)
    const stateValue = stateStr
        .split('=')[1]
        .split(',')
        .map((val, index) => [ANGLE_INDICES].includes(index) ? val / R2G : val)
    const deltaPitchValue = getValue(deltaPitch)
    const deltaRollValue = getValue(deltaRoll)
    const tauMaxValue = getValue(tauMax)
    const dTValue = getValue(dT)
    const resSparseValue = getValue(resSparse)
    /**
    * @description параметры планера
    */
    const testParams = {
        m: 2.75, // масса планера
        S_wing: 0.033, // площадь крыла
        B_chord: 0.04, // средняя аэродинамическая хорда
        Jx: 1, // момент инерции по продольной оси
        Jz: 1.5, // момент инерции по поперечной оси
        interpCYA: new Interp2D(), // коэфф. подъемной силы от числа М и угла атаки
        interpCXA: new Interp2D(), // коэфф. лобового сопротивления от числа М и угла атаки
        interpMZ: new Interp2D(), // коэфф. тангажного момента от числа М и угла атаки
        dMZ_elevator: new Interp1D(),  // зависимость управляющего момента от угла отклонения руля высоты
        dMX_aileron: new Interp1D(),	// зависимость управляющего момента от угла отклонения элеронов
    }
    
    const testControls = new GliderControls()

    testControls.setBasePitch(deltaPitchValue)
    
    testControls.setBaseRoll(deltaRollValue)

    testParams.interpCYA.init(MX, AX, CYA, 0, 0)
    testParams.interpCXA.init(MX, AX, CXA, 0, 0)
    testParams.interpMZ.init(MX, AX, MZ, 0, 0)
    testParams.dMZ_elevator.init(DX, dMZ, 0)
    testParams.dMX_aileron.init(DX, dMX, 0)

    const test_integ = integrate(
        stateValue,
        testParams,
        testControls,
        tauMaxValue,
        dTValue
    );
    
    (async function() {
        await printResult('res_1.txt', test_integ, resSparseValue)
    })()
}

setTrajectoryExperiment()
