'use strict'
const DT = 0.001
const { Interp1D, Interp2D } = require('./utils.js')
const { writeFile } = require('node:fs/promises') 
const g0 = 9.807
/**
* @description плотность
*/
const RoH = function(H) {
	return 1.2 * Math.exp(-H/9400)
}
/**
* @descrption скорость звука
*/
const aSnH = function(H) {
	if (H < 11500) return 340 - 0.00391 * H
	return 295
}

const N_VARS = 10 // количество уравнений (3 ускорения + 3 скорости + 2 угл.ускорения + 2 угл.скорости)

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
/**
* @description параметры законов управления
*/
const testControls = {
	_alpha0: 0,
	t0: 0,
	pitch: 0, // угол отклонения руля высоты
	roll: 0, // угол отклонения элеронов
	setRudder(state, t) { // закон управления рулем высоты
		const V = state[0]
		const kEps = this.t0 > 0 ? (state[3] - this._alpha0)/DT : 0
		if(V < 25) return 0
		let delta = 1.25 * (state[1] + 12.5/57.3) + 75.5 * kEps
		
		if(delta < 0) {
			if(delta < -15) delta = -15
			this.pitch = Math.max(-24.5, this.pitch + delta )
			this._alpha0 = state[3]
			this.t0 = t
			return
		} 

		if(delta > 15) delta = 15
		this.pitch = Math.min(24.5, this.pitch + delta)
		this._alpha0 = state[3]
		this.t0 = t
	},
	setAilerons(state, t){ // закон управления элеронами
		this.roll = 0
		return 0
	},	
}
/**
* @description Получить производные от линейный и угловых скоростей и перемещений
*/
const getDerivatives = function(controls, params, state, t) {
	const V = state[0]
	const Th = state[1]
	const Psi = state[2]
	const omegaZ = state[3]
	const omegaX = state[4]
	const Y = state[6]
	const alpha = state[8]
	const gamma = state[9]
	const a_x = alpha * 57.3
	
	const Mach = V / aSnH(Y)
	const QS = 0.5 * RoH(Y) * V * V * params.S_wing
	const QSB = QS * params.B_chord
	
	controls.setRudder(state, t)
	controls.setAilerons(state, t)
	
	const deltaElevator = controls.pitch
	const deltaAileron = controls.roll
	
	const Cya = params.interpCYA.interp(Mach, a_x)
	const Cxa = params.interpCXA.interp(Mach, a_x)
	const mZ0 = params.interpMZ.interp(Mach, a_x)
	const dMZ = params.dMZ_elevator.interp(deltaElevator)
	const MX = params.dMX_aileron.interp(deltaAileron)
	
	const _YA = Cya * QS / params.m
	const _XA = Cxa * QS / params.m
	const MZ = (mZ0 + dMZ) * QSB

	const STH = Math.sin(Th)
	const CTH = Math.cos(Th)
	const SPH = Math.sin(Psi)
	const CPH = Math.cos(Psi)
	const SG = Math.sin(gamma)
	const CG = Math.cos(gamma)
	
	return [
		-_XA - g0 * STH,
		(_YA * CG - g0 * CTH) / V,
		_YA * SG / V,
		MZ / params.Jz,
		MX * QSB/ params.Jx,
		V * CTH * CPH,
		V * STH,
		V * STH * SPH,
		omegaZ,
		omegaX,
	]
}
/**
* @description численное интегрирование (метод Эйлера-Коши второго порядка точности)
* @returns Array<Float64Array> массив состояний планера в отдельные моменты времени
*/
const integrate = function(initialState, params, controls, tauMax, dT) {
	let tau = 0
	let i = 0
	let V = 0
	let H = 0
	let alpha = 0
	let gamma = 0
	let Mach = 0
	const dT_05 = 0.5 * dT
	const dT_6 = dT * 0.1666666666666666667
	const result = [[tau, ...initialState]]
	let K0 = new Float64Array(N_VARS)
	let K1 = new Float64Array(N_VARS)
	let K2 = new Float64Array(N_VARS)
	let K3 = new Float64Array(N_VARS)
	const _state = new Float64Array(N_VARS)
	const _res = new Float64Array(N_VARS + 1)
	const { interpCYA, interpCXA, interpMZ, dMZ_elevator, dMX_aileron } = params
	
	while(tau < tauMax) {
		const state = result[i++].slice(1)
		
		V = state[0]
		H = state[6]
		alpha = state[8] * 57.3
		gamma = state[9] * 57.3
		Mach = V / aSnH(H)
		
		interpCYA.checkIndices(Mach, alpha)
		interpCXA.checkIndices(Mach, alpha)
		interpMZ.checkIndices(Mach, alpha)
		dMZ_elevator.checkIndex(controls.pitch)
		dMX_aileron.checkIndex(controls.roll)	
		
		K0 = getDerivatives(controls, params, state, tau)
		for(let j = 0; j < N_VARS; j++) {
			_state[j] = state[j] + dT_05 * K0[j]
		}
		K1 = getDerivatives(controls, params, _state, tau)
		for(let j = 0; j < N_VARS; j++) {
			_state[j] = state[j] + dT_05 * K1[j]
		}
		K2 = getDerivatives(controls, params, _state, tau)
		for(let j = 0; j < N_VARS; j++) {
			_state[j] = state[j] + dT_05 * K2[j]
		}
		K3 = getDerivatives(controls, params, _state, tau)
		for(let j = 0; j < N_VARS; j++) {
			_state[j] = state[j] + dT * K3[j]
		}

		tau += dT
		_res[0] = tau
		for(let j = 1; j < N_VARS; j++) {
			const j0 = j - 1
			_res[j] = state[j0] + dT_6 * (K0[j0] + 2*(K1[j0] + K2[j0]) + K3[j0])
		}
		result.push(_res.slice())
	}
	
	return result	
}
// здесь заканчивается описание служебных функций и начинается описание ИД

const MX = [0, 0.3, 0.5, 0.9]
const AX = [-30, -20, -10, -8, -6, -4, -2, -1, 0, 1, 2, 4, 6, 8, 10, 20, 30]
const DX = [-25, -20, -15, -10, -5, -2, -1, 0, 1, 2, 5, 10, 15, 20, 25]

// нагрузка на крыло - 75 кг/м2
// длина - 0.8 м
// ЦМ - 0.3 м
// плечо оперения - 0.75 м
// плечо крыла - 0.275 м
// отн.площадь элеронов - 0.1
// отн.площадь рулей высоты - 0.4

const CYA = [
	[-2.34009, -1.58131, -0.82254, -0.67079, -0.51903, -0.36728, -0.21552, -0.13965, -0.06377, 0.01211, 0.08799, 0.23974, 0.3915, 0.54325, 0.695, 1.45378, 2.21255 ],
	[-2.34009, -1.58131, -0.82254, -0.67079, -0.51903, -0.36728, -0.21552, -0.13965, -0.06377, 0.01211, 0.08799, 0.23974, 0.3915, 0.54325, 0.695, 1.45378, 2.21255 ],
	[-2.34009, -1.58131, -0.82254, -0.67079, -0.51903, -0.36728, -0.21552, -0.13965, -0.06377, 0.01211, 0.08799, 0.23974, 0.3915, 0.54325, 0.695, 1.45378, 2.21255 ],
	[-2.34009, -1.58131, -0.82254, -0.67079, -0.51903, -0.36728, -0.21552, -0.13965, -0.06377, 0.01211, 0.08799, 0.23974, 0.3915, 0.54325, 0.695, 1.45378, 2.21255 ],
	[-2.34009, -1.58131, -0.82254, -0.67079, -0.51903, -0.36728, -0.21552, -0.13965, -0.06377, 0.01211, 0.08799, 0.23974, 0.3915, 0.54325, 0.695, 1.45378, 2.21255 ]
]


const CXA = [
	[0.50002, 0.25604, 0.10648, 0.0879, 0.07309, 0.06206, 0.05481, 0.0526, 0.05133, 0.05101, 0.05164, 0.05571, 0.06357, 0.0752, 0.09061, 0.2243, 0.45241],
	[0.50002, 0.25604, 0.10648, 0.0879, 0.07309, 0.06206, 0.05481, 0.0526, 0.05133, 0.05101, 0.05164, 0.05571, 0.06357, 0.0752, 0.09061, 0.2243, 0.45241],
	[0.50002, 0.25604, 0.10648, 0.0879, 0.07309, 0.06206, 0.05481, 0.0526, 0.05133, 0.05101, 0.05164, 0.05571, 0.06357, 0.0752, 0.09061, 0.2243, 0.45241],
	[0.50002, 0.25604, 0.10648, 0.0879, 0.07309, 0.06206, 0.05481, 0.0526, 0.05133, 0.05101, 0.05164, 0.05571, 0.06357, 0.0752, 0.09061, 0.2243, 0.45241],
	[0.50002, 0.25604, 0.10648, 0.0879, 0.07309, 0.06206, 0.05481, 0.0526, 0.05133, 0.05101, 0.05164, 0.05571, 0.06357, 0.0752, 0.09061, 0.2243, 0.45241],
]


const MZ = [
	[2.53222, 1.83047, 1.12872, 0.98837, 0.84802, 0.70767, 0.56732, 0.49715, 0.42697, 0.3568, 0.28662, 0.14627, 0.00592, -0.13443, -0.27478, -0.97652, -1.67827],
	[2.53222, 1.83047, 1.12872, 0.98837, 0.84802, 0.70767, 0.56732, 0.49715, 0.42697, 0.3568, 0.28662, 0.14627, 0.00592, -0.13443, -0.27478, -0.97652, -1.67827],
	[2.53222, 1.83047, 1.12872, 0.98837, 0.84802, 0.70767, 0.56732, 0.49715, 0.42697, 0.3568, 0.28662, 0.14627, 0.00592, -0.13443, -0.27478, -0.97652, -1.67827],
	[2.53222, 1.83047, 1.12872, 0.98837, 0.84802, 0.70767, 0.56732, 0.49715, 0.42697, 0.3568, 0.28662, 0.14627, 0.00592, -0.13443, -0.27478, -0.97652, -1.67827],
	[2.53222, 1.83047, 1.12872, 0.98837, 0.84802, 0.70767, 0.56732, 0.49715, 0.42697, 0.3568, 0.28662, 0.14627, 0.00592, -0.13443, -0.27478, -0.97652, -1.67827],
]

const dMZ = [1.06031, 0.88241, 0.7045, 0.5266, 0.34869, 0.24195, 0.20637, 0.17079, 0.13521, 0.09963, -0.00712, -0.18502, -0.36293, -0.54083, -0.71874]
const dMX= [-0.12687, -0.1015, -0.07612, -0.05075, -0.02537, -0.01015, -0.00507, 0, 0.00507, 0.01015, 0.02537, 0.05075, 0.07612, 0.1015, 0.12687 ]

testParams.interpCYA.init(MX, AX, CYA, 0, 0)
testParams.interpCXA.init(MX, AX, CXA, 0, 0)
testParams.interpMZ.init(MX, AX, MZ, 0, 0)
testParams.dMZ_elevator.init(DX, dMZ, 0)
testParams.dMX_aileron.init(DX, dMX, 0)

const initial_state_test = [
	225,	//V
	1/57.3,	//Th
	0,		//Psi
	0,		//epsZ
	0,		//epsX
	15,		//X
	14500,	//Y
	0,		//Z
	6.2/57.3,		//alpha
	0		//gamma
]

const test_integ = integrate(
	initial_state_test,
	testParams,
	testControls,
	180.5,
	DT
)

const integ_res = test_integ.reduce((res, row, i) => {
	if(i % 100 !== 0) return res
	res += [
		row[0].toFixed(2),
		row[1].toFixed(1),
		(row[2] * 57.3).toFixed(2),
		(row[3] * 57.3).toFixed(2),
		(row[4] * 57.3).toFixed(3),
		(row[5] * 57.3).toFixed(3),
		row[6].toFixed(0),
		row[7].toFixed(0),
		row[8].toFixed(0),
		(row[9] * 57.3).toFixed(2),
		(row[10] * 57.3).toFixed(2),
	].join('\t') + '\n'
	return res
}, '')

const txt = integ_res.replaceAll('.', ',')
writeFile('res.txt', txt)