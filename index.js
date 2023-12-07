'use strict'

const { integrate, printResult, R2G } = require('./trajectory.js')
const {
    MX,
    AX,
    DX,
    CYA,
    CXA,
    MZ,
    dMZ,
    dMX,
    T_THRUST,
    M_THRUST,
    K_MOTOR,
    T_MOTOR,
    R_MAX,
    MOTOR_DRAIN_MAX,
    CHARGE_TOTAL
} = require('./init_data.json') // импорт ИД
const { GliderControls } = require('./controls.js')
const { Booster, ElectricMotor } = require('./booster.js')
const { Interp1D, Interp2D } = require('./utils.js')
const { argv } = require('node:process')

const getValue = function(raw) {
    return Number(raw.split('=')[1])
}

const ANGLE_INDICES = [1, 2, 3, 4, 8, 9]

const setTrajectoryExperiment = function() {
    
    const [stateStr, deltaPitch, deltaRoll, tauMax, dT, resSparse] = argv.slice(2)
    const stateValue = stateStr
        .split('=')[1]
        .split(',')
        .map((val, index) => [ANGLE_INDICES].includes(index) ? (+val / R2G) : +val)
    const deltaPitchValue = getValue(deltaPitch)
    const deltaRollValue = getValue(deltaRoll)
    const tauMaxValue = getValue(tauMax)
    const dTValue = getValue(dT)
    const resSparseValue = getValue(resSparse)
    /**
    * @description параметры планера
    */
    const testParams = {
        jRelative: 0, // удельный импульс СРС, м/с
        m: 315,
        mFuel: 0.0, // масса планера
        S_wing: 80.0, // площадь крыла
        B_chord: 2.0, // средняя аэродинамическая хорда
        Jx: 1775, // момент инерции по продольной оси
        Jz: 22272, // момент инерции по поперечной оси
        interpCYA: new Interp2D(), // коэфф. подъемной силы от числа М и угла атаки
        interpCXA: new Interp2D(), // коэфф. лобового сопротивления от числа М и угла атаки
        interpMZ: new Interp2D(), // коэфф. тангажного момента от числа М и угла атаки
        dMZ_elevator: new Interp1D(),  // зависимость управляющего момента от угла отклонения руля высоты
        dMX_aileron: new Interp1D(),	// зависимость управляющего момента от угла отклонения элеронов
        booster: new Booster(),
        motor: new ElectricMotor()
    }
    
    const testControls = new GliderControls()
    // previous iteration
    // testControls.initPitchCtrl(
    //     -15.5,
    //     15.5,
    //     360,
    //     0.05,
    //     20.5,
    //     8.75,
    //     27.25,
    //     -2.5/R2G
    // )
    
    // testControls.initRollCtrl(
    //     -29.5,
    //     29.5,
    //     360,
    //     0.1,
    //     -2.0,
    //     -1.0,
    //     -10.0,
    //     0/R2G
    // )

    // controls for high-altitude plane; cruise stage
    // testControls.initPitchCtrl(
    //     -18.5,
    //     18.5,
    //     360,
    //     0.05,
    //     7.5,
    //     5.75,
    //     25.75,
    //     5.25/R2G
    // )

    // controls for high-altitude plane; beginning of ascend
    testControls.initPitchCtrl(
        -18.5,
        18.5,
        360,
        0.05,
        15.5,
        2.75,
        105.75,
        3.75/R2G
    )
    
    testControls.initRollCtrl(
        -10.5,
        10.5,
        360,
        0.1,
        -2.0,
        -1.0,
        -10.0,
        0/R2G
    )

    testControls.togglePitch(true)
    testControls.toggleRoll(true)

    testControls.setBasePitch(deltaPitchValue)
    testControls.setBaseRoll(deltaRollValue)
    
    testParams.interpCYA.init(MX, AX, CYA, 0, 0)
    testParams.interpCXA.init(MX, AX, CXA, 0, 0)
    testParams.interpMZ.init(MX, AX, MZ, 0, 0)
    testParams.dMZ_elevator.init(DX, dMZ, 0)
    testParams.dMX_aileron.init(DX, dMX, 0)

    if(T_THRUST.length && M_THRUST.length) {
        testParams.booster.init(testParams.mFuel, testParams.jRelative, T_THRUST, M_THRUST)
    } else if(K_MOTOR.length && T_MOTOR.length) {
        testParams.motor.init(CHARGE_TOTAL, R_MAX, MOTOR_DRAIN_MAX, T_MOTOR, K_MOTOR)
    }

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
