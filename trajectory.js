'use strict'

const { writeFile } = require('node:fs/promises')
const { performance } = require('perf_hooks')

const g0 = 9.807 // ускорение свободного падения
const DEFAULT_HEADER = [
	't[s]',
	'V[m/s]',
	'Th[gr]',
	'Psi[gr]',
	'wZ[gr/s]',
	'wX[gr/s]',
	'X[m]',
	'Y[m]',
	'Z[m]',
	'pitch[gr]',
	'roll[gr]',
	'dElev[gr]',
	'dAil[gr]',
	'm[kg]',
	'\n'] // заголовок таблицы результатов выдачи
const R2G = 180 / Math.PI // перевод градусы <=> радианы
const N_VARS = 11 // количество уравнений (3 ускорения + 3 скорости + 2 угл.ускорения + 2 угл.скорости + масса)
const N_PRMS = 3 // количество независимых параметров (время/откл.руля высоты/откл.элеронов)
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
* @description Получить производные от линейный и угловых скоростей и перемещений
*/
const getDerivatives = function(controls, params, state, t, buffer) {
	const V = state[0]
	const Th = state[1]
	const Psi = state[2]
	const omegaZ = state[3]
	const omegaX = state[4]
	const Y = state[6]
	const alpha = state[8]
	const gamma = state[9]
	const a_x = alpha * 57.3
	const m = state[10]

	let dM = 0
	let R = 0
	if (params.booster) {
		const thrust = params.booster.getThrust()
		dM = thrust[0]
		dR = thrust[1]
	}

	const Mach = V / aSnH(Y)
	const QS = 0.5 * RoH(Y) * V * V * params.S_wing
	const QSB = QS * params.B_chord
	
	const deltaElevator = controls.deltaPitch()
	const deltaAileron = controls.deltaRoll()

	const Cya = params.interpCYA.interp(Mach, a_x)
	const Cxa = params.interpCXA.interp(Mach, a_x)
	const mZ0 = params.interpMZ.interp(Mach, a_x)
	const dMZ = params.dMZ_elevator.interp(deltaElevator)
	const MX = params.dMX_aileron.interp(deltaAileron)
	
	const _YA = Cya * QS / m
	const _XA = Cxa * QS / m
	const MZ = (mZ0 + dMZ) * QSB

	const STH = Math.sin(Th)
	const CTH = Math.cos(Th)
	const SPH = Math.sin(Psi)
	const CPH = Math.cos(Psi)
	const SG = Math.sin(gamma)
	const CG = Math.cos(gamma)

	const dRaxial = R ? R * Math.cos(alpha) / m : 0
	const dRnormal = R ? R * Math.sin(alpha) / m : 0
	
	buffer[0] = dRaxial -_XA - g0 * STH
	buffer[1] = (dRnormal - _YA * CG - g0 * CTH) / V
	buffer[2] = _YA * SG / V
	buffer[3] = MZ / params.Jz
	buffer[4] = MX * QSB/ params.Jx
	buffer[5] = V * CTH * CPH
	buffer[6] = V * STH
	buffer[7] = V * CTH * SPH
	buffer[8] = omegaZ
	buffer[9] = omegaX
	buffer[10] = dM ? -dM : 0
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
	const result = [Float64Array.from([tau, ...initialState, params.m, controls.deltaPitch(), controls.deltaRoll() ])]
	const K0 = new Float64Array(N_VARS)
	const K1 = new Float64Array(N_VARS)
	const K2 = new Float64Array(N_VARS)
	const K3 = new Float64Array(N_VARS)
	const _state = new Float64Array(N_VARS)
	const _res = new Float64Array(N_VARS + N_PRMS)
	const { interpCYA, interpCXA, interpMZ, dMZ_elevator, dMX_aileron } = params
	const t0 = performance.now()
	
	while (tau < tauMax) {
		const state = result[i++].slice(1, 12)
		
		V = state[0]
		H = state[6]
		alpha = state[8] * R2G
		Mach = V / aSnH(H)

		interpCYA.checkIndices(Mach, alpha)
		interpCXA.checkIndices(Mach, alpha)
		interpMZ.checkIndices(Mach, alpha)

		const dElevator = controls.deltaPitch()
		const dAileron = controls.deltaRoll()

		dMZ_elevator.checkIndex(dElevator)
		dMX_aileron.checkIndex(dAileron)
		controls.checkControls(state, tau)

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
		for(let j = 1; j < N_VARS + 1; j++) {
			const j0 = j - 1
			_res[j] = state[j0] + dT_6 * (K0[j0] + 2*(K1[j0] + K2[j0]) + K3[j0])
		}
		_res[N_VARS + 1] = dElevator
		_res[N_VARS + 2] = dAileron
		result.push(_res.slice())
	}
	const t1 = performance.now() - t0
	console.log(`calculation time: ${t1.toFixed(3)} ms;`)
	
	return result	
}

/**
 * @description Преобразовать результаты в человеко-читаемый вид и записать в текстовый файл
 * @param {string} path путь к полученному файлу-результату
 * @param {Array<Float64Array>} result массив данных расчета
 * @param {number} sparce скважность выборки записей в результат
 * @param {boolean} comaSwitch флаг замены точки на запятые (для совместимости с Excel/PlanMaker)
 */
const printResult = async function(path, data, sparce, tableHeader = DEFAULT_HEADER.join('\t'), comaSwitch = true) {
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
			data[k][11].toFixed(2),
			data[k][12].toFixed(1),
			data[k][13].toFixed(1),
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

module.exports = { integrate, printResult, R2G }