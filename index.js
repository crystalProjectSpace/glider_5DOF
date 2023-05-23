'use strict'

const { integrate, printResult, R2G } = require('./trajectory.js')
const { MX, AX, DX, CYA, CXA, MZ, dMZ, dMX } = require('./init_data.json') // импорт ИД
const { GliderControls } = require('./controls.js')
const { Interp1D, Interp2D } = require('./utils.js')

const setTrajectoryExperiment = function(state0, deltaPitch, deltaRoll, tauMax, dT, resSparse) {
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

    testControls.setBasePitch(deltaPitch)
    
    testControls.setBaseRoll(deltaRoll)

    testParams.interpCYA.init(MX, AX, CYA, 0, 0)
    testParams.interpCXA.init(MX, AX, CXA, 0, 0)
    testParams.interpMZ.init(MX, AX, MZ, 0, 0)
    testParams.dMZ_elevator.init(DX, dMZ, 0)
    testParams.dMX_aileron.init(DX, dMX, 0)

    const test_integ = integrate(
        state0,
        testParams,
        testControls,
        tauMax,
        dT
    );
    
    (async function() {
        await printResult('res_1.txt', test_integ, resSparse)
    })()
}

const INITIAL_STATE = [
    225,	//V
    1/R2G,	//Th
    0,		//Psi
    0,		//epsZ
    0,		//epsX
    15,		//X
    14500,	//Y
    0,		//Z
    0/R2G,//alpha
    0		//gamma
]

setTrajectoryExperiment(INITIAL_STATE, -4, 0, 10, 0.001, 100)