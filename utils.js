'use strict'
function indexLookup(index, X, X0, VX) {
	const size = VX.length
	if(X > X0) {
		if(X < VX[index + 1]) return
		let i0 = index
		for(let i = i0; i < size; i++) {
			if(VX[i0 + 1] > X) return i0
		}
	}
	let i1 = index
	let i0 = i1 - 1
	for(let i = i1; i > -1; i--) {		
		if(VX[X > i0]) return i0
	}
}

/**
* @description Класс одномерной линейной интерполяции
*/
const Interp1D = function() {
	this.arg = null
	this.val = null
	this.size = 0
	this.index = 0
	this.kX = 0
	
}

Interp1D.prototype.init = function(VX, VY, X0) {
	this.size = VX.length
	this.arg = Float64Array.from(VX)
	this.val = Float64Array.from(VY)
	
	for(let i = 0; i < this.size; i++) {
		if (this.arg[i + 1] > X0) {
			this.index = i
			this.Y0 = this.val[this.index]
			this.X0 = this.arg[this.index]
			this.kX = (this.val[this.index + 1] - this.Y0) / (this.arg[this.index + 1] - this.X0)
			break
		}
	}
}

Interp1D.prototype.interp = function(X) {
	return this.Y0 + this.kX * (X - this.X0) 
}

Interp1D.prototype.checkIndex = function(X) {
	this.index = indexLookup(this.index, X, this.X0, this.arg)
	const i1 = this.index + 1
	this.Y0 = this.val[this.index]
	this.X0 = this.arg[this.index]
	this.kX = (this.val[i1] - this.Y0) / (this.arg[i1] - this.X0)
}
/**
* @description Класс двухмерной линейной интерполяции
*/
const Interp2D = function() {
	this.arg1 = null
	this.arg2 = null
	this.val = null
	this.size1 = 0
	this.size2 = 0
	this.index1 = 0
	this.index2 = 0
	
	this.Y00 = 0
	this.Y01 = 0
	
	this.Y10 = 0
	this.Y11 = 0
	
	this.X0_0 = 0
	this.X1_0 = 0
	
	this.X0_1 = 0
	this.X1_1 = 0
	
	this.kX1 = 0
	this.kX2 = 0
}

Interp2D.prototype.init = function(VX1, VX2, VY) {
	this.size1 = VX1.length
	this.size2 = VX2.length
	this.arg1 = Float64Array.from(VX1)
	this.arg2 = Float64Array.from(VX2)
	this.val = Float64Array.from(VY)
	
	for(let i = 0; i < this.size1; i++) {
		if (this.arg1[i + 1] > X0) {
			this.index1 = i
			break
		}
	}
	
	for(let i = 0; i < this.size1; i++) {
		if (this.arg2[i + 1] > X0) {
			this.index2 = i
			break
		}
	}
	this.Y00 = this.val[this.index1][this.index2]
	this.Y01 = this.val[this.index1][this.index2 + 1]
	
	this.Y10 = this.val[this.index1 + 1][this.index2]
	this.Y11 = this.val[this.index1 + 1][this.index2 + 1]
	
	this.X0_0 = this.arg1[this.index1]
	this.X1_0 = this.arg2[this.index2]
	
	this.X0_1 = this.arg1[this.index1 + 1]
	this.X1_1 = this.arg2[this.index2 + 1]
	
	this.kX1 = (Y01 - Y00) / (X0_1 - X0_0)
	this.kX2 = (Y11 - Y10) / (X0_1 - X0_0)
}

Interp2D.prototype.interp = function(X1, X2) {
	const Y_0 = this.Y00 + this.kX1 * (X1 - this.X0_0)
	const Y_1 = this.Y10 + this.kX2 * (X1 - this.X0_0)
	
	retutn Y_0 + (Y_1 - Y_1) * (X2 - this.X1_0) / (this.X1_1 - this.X1_0)
}

Interp2D.prototype.checkIndices = function(X1, X2) {
	this.index1 = indexLookup(this.index1, X1, this.X0_0, this.arg1)
	this.index2 = indexLookup(this.index2, X2, this.X1_0, this.arg2)
	
	const i10 = this.index1 + 1
	const i11 = this.index2 + 1
	this.Y00 = this.val[this.index1][this.index2]
	this.Y01 = this.val[this.index1][i11]
	
	this.Y10 = this.val[i10][this.index2]
	this.Y11 = this.val[i10][i11]
	
	this.X0_0 = this.arg1[this.index1]
	this.X1_0 = this.arg2[this.index2]
	
	this.X0_1 = this.arg1[i10]
	this.X1_1 = this.arg2[i11]
	
	this.kX1 = (Y01 - Y00) / (X0_1 - X0_0)
	this.kX2 = (Y11 - Y10) / (X0_1 - X0_0)
}

module.exports = {
	Interp1D,
	Interp2D,
}