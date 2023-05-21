'use strict'

const { Interp1D, Interp2D } = require('./utils.js')
const { writeFile } = require('node:fs/promises')
const { performance } = require('perf_hooks')
const { MX, AX, DX, CYA, CXA, MZ, dMZ, dMX } = require('./init_data.json') // импорт ИД

const DT = 0.002 // дефолтный шаг интегрирования по времени
const g0 = 9.807 // ускорение свободного падения
const DEFAULT_HEADER = 't[s]\tV[m/s]\tTh[gr]\tPsi[gr]\twZ[gr/s]\twX[gr/s]\tX[m]\tY[m]\tZ[m]\tpitch[gr]\troll[gr]\n' // заголовок таблицы результатов выдачи
const R2G = 180 / Math.PI // перевод градусы <=> радианы
const N_VARS = 10 // количество уравнений (3 ускорения + 3 скорости + 2 угл.ускорения + 2 угл.скорости)
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
	setRudder(state, t, dT) { // закон управления рулем высоты
		const V = state[0]
		const kEps = this.t0 > 0 ? (state[3] - this._alpha0)/DT : 0
		if (V < 25) return 0
		let delta = 1.25 * (state[1] + 3.695/R2G) + 125.5 * kEps
		const rudderSpeed = dT * 3600
		if (delta < 0) {
			if (delta < -rudderSpeed) delta = -rudderSpeed
			this.pitch = Math.max(-24.5, this.pitch + delta )
			this._alpha0 = state[3]
			this.t0 = t
			return
		} 

		if (delta > rudderSpeed) delta = rudderSpeed
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
const getDerivatives = function(controls, params, state, t, buffer, dT) {
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
	
	controls.setRudder(state, t, dT)
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
	
	buffer[0] = -_XA - g0 * STH
	buffer[1] = (_YA * CG - g0 * CTH) / V
	buffer[2] = _YA * SG / V
	buffer[3] = MZ / params.Jz
	buffer[4] = MX * QSB/ params.Jx
	buffer[5] = V * CTH * CPH
	buffer[6] = V * STH
	buffer[7] = V * CTH * SPH
	buffer[8] = omegaZ
	buffer[9] = omegaX
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
	const t0 = performance.now()
	
	while (tau < tauMax) {
		const state = result[i++].slice(1)
		
		V = state[0]
		H = state[6]
		alpha = state[8] * R2G
		Mach = V / aSnH(H)
		
		interpCYA.checkIndices(Mach, alpha)
		interpCXA.checkIndices(Mach, alpha)
		interpMZ.checkIndices(Mach, alpha)
		dMZ_elevator.checkIndex(controls.pitch)
		dMX_aileron.checkIndex(controls.roll)	
		
		getDerivatives(controls, params, state, tau, K0, dT)
		for(let j = 0; j < N_VARS; j++) {
			_state[j] = state[j] + dT_05 * K0[j]
		}
		getDerivatives(controls, params, _state, tau, K1, dT)
		for(let j = 0; j < N_VARS; j++) {
			_state[j] = state[j] + dT_05 * K1[j]
		}
		getDerivatives(controls, params, _state, tau, K2, dT)
		for(let j = 0; j < N_VARS; j++) {
			_state[j] = state[j] + dT_05 * K2[j]
		}
		getDerivatives(controls, params, _state, tau, K3, dT)
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
	const t1 = performance.now() - t0
	console.log(`calculation time: ${t1.toFixed(3)} ms;`)
	
	return result	
}

testParams.interpCYA.init(MX, AX, CYA, 0, 0)
testParams.interpCXA.init(MX, AX, CXA, 0, 0)
testParams.interpMZ.init(MX, AX, MZ, 0, 0)
testParams.dMZ_elevator.init(DX, dMZ, 0)
testParams.dMX_aileron.init(DX, dMX, 0)

const initial_state_test = [
	225,	//V
	1/R2G,	//Th
	0,		//Psi
	0,		//epsZ
	0,		//epsX
	15,		//X
	14500,	//Y
	0,		//Z
	6.2/R2G,//alpha
	0		//gamma
]

/**
 * @description Преобразовать результаты в человеко-читаемый вид и записать в текстовый файл
 * @param {string} path путь к полученному файлу-результату
 * @param {Array<Float64Array>} result массив данных расчета
 * @param {number} sparce скважность выборки записей в результат
 * @param {boolean} comaSwitch флаг замены точки на запятые (для совместимости с Excel/PlanMaker)
 */
const printResult = async function(path, data, sparce, tableHeader = DEFAULT_HEADER, comaSwitch = true) {
	let result = tableHeader
	const size = Math.floor(data.length / sparce)
	let k = 0
	let tempStr = ''
	for(let i = 0; i < size; i++) {
		tempStr = [
			data[k][0].toFixed(2),
			data[k][1].toFixed(1),
			(data[k][2] * R2G).toFixed(2),
			(data[k][3] * R2G).toFixed(2),
			(data[k][4] * R2G).toFixed(3),
			(data[k][5] * R2G).toFixed(3),
			data[k][6].toFixed(0),
			data[k][7].toFixed(0),
			data[k][8].toFixed(0),
			(data[k][9] * R2G).toFixed(2),
			(data[k][10] * R2G).toFixed(2),
		].join('\t') + '\n'
		if (comaSwitch) tempStr = tempStr.replaceAll('.', ',')
		result += tempStr
		k += sparce
	}
	try {
		await writeFile(path, result)
		console.log(`Trajectory data succesfully written to ${path}`)
	} catch(err) {
		console.log(`An error occured while writing results: ${err}`)
	}
}

const test_integ = integrate(
	initial_state_test,
	testParams,
	testControls,
	120.5,
	DT
);

(async function() {
	await printResult('res_1.txt', test_integ, 10)
})()
