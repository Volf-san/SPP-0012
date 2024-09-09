/* CR266 fixed */
// From NPointToNPointTransformer
console.log('Loading CogMath')

var cogUtils = {}

if (process['platform'] === 'win32') {
  cogUtils = require('./CogUtils.js')
} else {  
  cogUtils = require('CogUtils.js')
}

const version = Object.freeze(new cogUtils.Version(2, 0, 1))
console.log('CogMath Version: ' + version.toString())

function Vector2 (x, y) {
  this.x = (x === undefined) ? 0 : x
  this.y = (y === undefined) ? 0 : y

  this.set = function (x, y) {
    this.x = x || 0
    this.y = y || 0
  }

  this.clone = function () {
    return new Vector2(this.x, this.y)
  }

  this.add = function (vector) {
    return new Vector2(this.x + vector.x, this.y + vector.y)
  }

  this.subtract = function (vector) {
    return new Vector2(this.x - vector.x, this.y - vector.y)
  }

  this.scale = function (scalar) {
    return new Vector2(this.x * scalar, this.y * scalar)
  }

  this.dot = function (vector) {
    return (this.x * vector.x + this.y + vector.y)
  },

  moveTowards = function (vector, t) {
    // Linearly interpolates between vectors A and B by t.
    // t = 0 returns A, t = 1 returns B
    t = Math.min(t, 1) // still allow negative t
    var diff = vector.subtract(this)
    return this.add(diff.scale(t))
  }

  this.magnitude = function () {
    return Math.sqrt(this.magnitudeSqr())
  }

  this.magnitudeSqr = function () {
    return (this.x * this.x + this.y * this.y)
  }

  this.distance = function (vector) {
    return Math.sqrt(this.distanceSqr(vector))
  }

  this.distanceSqr = function (vector) {
    var deltaX = this.x - vector.x
    var deltaY = this.y - vector.y
    return (deltaX * deltaX + deltaY * deltaY)
  }

  this.normalize = function () {
    var mag = this.magnitude()
    var vector = this.clone()
    if (Math.abs(mag) < 1e-9) {
      vector.x = 0
      vector.y = 0
    } else {
      vector.x /= mag
      vector.y /= mag
    }
    return vector
  }

  this.angle = function () {
    return Math.atan2(this.y, this.x)
  },

  this.rotate = function (alpha) {
    var cos = Math.cos(alpha)
    var sin = Math.sin(alpha)
    var vector = new Vector2()
    vector.x = this.x * cos - this.y * sin
    vector.y = this.x * sin + this.y * cos
    return vector
  }

  this.toPrecision = function (precision) {
    var vector = this.clone()
    vector.x = vector.x.toFixed(precision)
    vector.y = vector.y.toFixed(precision)
    return vector
  }

  this.toString = function () {
    var vector = this.toPrecision(1)
    return ('[' + vector.x + '; ' + vector.y + ']')
  }
};

function setPythag (a, b) {
  var at = Math.abs(a)
  var bt = Math.abs(b)
  var ct
  if (at > bt) {
    ct = bt / at
    return at * Math.sqrt(1.0 + ct * ct)
  } else {
    if (bt !== 0) {
      ct = at / bt
      return bt * Math.sqrt(1.0 + ct * ct)
    } else { return 0.0 }
  }
};

function setMax (a, b) {
  var maxarg1 = a
  var maxarg2 = b
  var c = (maxarg1 > maxarg2) ? maxarg1 : maxarg2
  return c
};

function sign (a, b) {
  if (b >= 0.0) { return Math.abs(a) } else { return -Math.abs(a) }
};

function getDistance (p1, p2) {
  if (p1.length !== 3 || p2.length !== 3) { throw new Error('p1.length !== 3 || p2.length !== 3') }
  var distance = 0.0
  distance += (p1[0] - p2[0]) * (p1[0] - p2[0])
  distance += (p1[1] - p2[1]) * (p1[1] - p2[1])
  distance += (p1[2] - p2[2]) * (p1[2] - p2[2])
  return Math.sqrt(distance)
};

function cfNormalizePoints (points, // array of array of length 3
  scale, // array of length 1
  transX, // array of length 1
  transY, // array of length 1
  transZ // array of length 1
) {
  if (points.length < 2) { throw new Error('points.length < 2') }
  var i
  // Compute mean position of points
  var sumXyz = [0.0, 0.0, 0.0]
  for (i = 0; i < points.length; i++) {
    sumXyz[0] += points[i][0]
    sumXyz[1] += points[i][1]
    sumXyz[2] += points[i][2]
  }
  var meanXyz = [0.0, 0.0, 0.0]
  meanXyz[0] = sumXyz[0] / points.length
  meanXyz[1] = sumXyz[1] / points.length
  meanXyz[2] = sumXyz[2] / points.length
  // Compute mean distance of points from mean position
  var sumDist = 0.0
  for (i = 0; i < points.length; i++) { sumDist += getDistance(meanXyz, points[i]) }
  var meanDist = sumDist / points.length
  // Compute similarity transform parameters
  if (meanDist === 0) { scale[0] = 1.0 } else { scale[0] = Math.sqrt(2.0) / meanDist }
  transX[0] = -meanXyz[0]
  transY[0] = -meanXyz[1]
  transZ[0] = -meanXyz[2]
  // Normalize points
  for (i = 0; i < points.length; i++) {
    points[i][0] = (points[i][0] - meanXyz[0]) * scale[0]
    points[i][1] = (points[i][1] - meanXyz[1]) * scale[0]
    points[i][2] = (points[i][2] - meanXyz[2]) * scale[0]
  }
};

var ccNmMatrixOp = {

  add: function (m1, m2) {
    if (m1.length != m2.length) { throw new Error('m1 and m2 do not have the same dimensions') }

    for (var row = 0; row < m1.length; row++) {
      if (m1[row].length != m2[row].length) { throw new Error('m1 and m2 do not have the same dimensions') }

      if (m1[0].length !== m1[row].length) { throw new Error('input is not rectangular') }
    }

    var output = []
    if (Array.isArray(m1[0]) === true) {
      for (var row = 0; row < m1.length; row++) {
        output[row] = []
        for (var col = 0; col < m1[row].length; col++) {
          output[row][col] = m1[row][col] + m2[row][col]
        }
      }
    } else {
      for (var row = 0; row < m1.length; row++) { output[row] = m1[row] + m2[row] }
    }

    return output
  },

  subtract: function (m1, m2) {
    if (m1.length != m2.length) { throw new Error('m1 and m2 do not have the same dimensions') }

    for (var row = 0; row < m1.length; row++) {
      if (m1[row].length != m2[row].length) { throw new Error('m1 and m2 do not have the same dimensions') }

      if (m1[0].length !== m1[row].length) { throw new Error('input is not rectangular') }
    }

    var output = []
    if (Array.isArray(m1[0]) === true) {
      for (var row = 0; row < m1.length; row++) {
        output[row] = []
        for (var col = 0; col < m1[row].length; col++) {
          output[row][col] = m1[row][col] - m2[row][col]
        }
      }
    } else {
      for (var row = 0; row < m1.length; row++) { output[row] = m1[row] - m2[row] }
    }

    return output
  },

  scale: function (m, a) {
    for (var row = 0; row < m.length; row++) {
      if (m[0].length !== m[row].length) { throw new Error('input is not rectangular') }
    }

    var output = []
    if (Array.isArray(m[0]) === true) {
      for (var row = 0; row < m.length; row++) {
        output[row] = []
        for (var col = 0; col < m[row].length; col++) {
          output[row][col] = m[row][col] * a
        }
      }
    } else {
      for (var row = 0; row < m.length; row++) { output[row] = m[row] * a }
    }

    return output
  },

  transpose: function (m1) {
    if (m1.length === 0) { throw new Error('Array size = 0') }

    for (var row = 0; row < m1.length; row++) {
      if (m1[0].length !== m1[row].length) { throw new Error('m1 is not rectangular') }
      if (Array.isArray(m1[row]) === false) { throw new Error('m1 is not two dimensional') }
    }

    var output = []
    for (var col = 0; col < m1[0].length; col++) {
      output[col] = []
      for (var row = 0; row < m1.length; row++) {
        output[col][row] = m1[row][col]
      }
    }

    return output
  },

  multiply: function (m1, m2) {
    if (m1.length === 0) { throw new Error('m1.length === 0') }
    if (m2.length === 0) { throw new Error('m2.length === 0') }

    for (var row = 0; row < m1.length; row++) {
      if (m1[0].length !== m1[row].length) { throw new Error('m1 is not rectangular') }
      if (Array.isArray(m1[row]) === false) { throw new Error('m1 is not two dimensional') }
    }

    for (var row = 0; row < m2.length; row++) {
      if (m2[0].length !== m2[row].length) { throw new Error('m2 is not rectangular') }
      if (Array.isArray(m2[row]) === false) { throw new Error('m2 is not two dimensional') }
    }

    if (m1[0].length !== m2.length) { throw new Error('Num m1 cols !== Num m2 rows') }

    var result = []
    for (var i = 0; i < m1.length; i++) {
      result[i] = []
      for (var j = 0; j < m2[0].length; j++) {
        var sum = 0
        for (var k = 0; k < m1[0].length; k++) {
          sum += m1[i][k] * m2[k][j]
        }
        result[i][j] = sum
      }
    }
    return result
  },

  inverse: function (m) {
    if (m.length === 0) { throw new Error('m.length === 0') }

    for (var row = 0; row < m.length; row++) {
      if (m[0].length !== m[row].length) { throw new Error('m is not rectangular') }
      if (Array.isArray(m[row]) === false) { throw new Error('m is not two dimensional') }
    }

    // I use Gaussian Elimination to calculate the inverse:
    // (1) 'augment' the matrix (left) by the identity (on the right)
    // (2) Turn the matrix on the left into the identity by elementary row ops
    // (3) The matrix on the right is the inverse (was the identity matrix)
    // There are 3 elementary row ops: (I combine b and c in my code)
    // (a) Swap 2 rows
    // (b) Multiply a row by a scalar
    // (c) Add 2 rows

    // if the matrix isn't square: exit (error)
    if (m.length !== m[0].length) { return }

    // create the identity matrix (I), and a copy (C) of the original
    var i = 0; var ii = 0; var j = 0; var dim = m.length; var e = 0; var t = 0
    var I = []; var C = []
    for (i = 0; i < dim; i += 1) {
      // Create the row
      I[I.length] = []
      C[C.length] = []
      for (j = 0; j < dim; j += 1) {
        // if we're on the diagonal, put a 1 (for identity)
        if (i === j) { I[i][j] = 1 } else { I[i][j] = 0 }

        // Also, make the copy of the original
        C[i][j] = m[i][j]
      }
    }

    // Perform elementary row operations
    for (i = 0; i < dim; i += 1) {
      // get the element e on the diagonal
      e = C[i][i]

      // if we have a 0 on the diagonal (we'll need to swap with a lower row)
      if (e === 0) {
        // look through every row below the i'th row
        for (ii = i + 1; ii < dim; ii += 1) {
          // if the ii'th row has a non-0 in the i'th col
          if (C[ii][i] !== 0) {
            // it would make the diagonal have a non-0 so swap it
            for (j = 0; j < dim; j++) {
              e = C[i][j] // temp store i'th row
              C[i][j] = C[ii][j]// replace i'th row by ii'th
              C[ii][j] = e // replace ii'th by temp
              e = I[i][j] // temp store i'th row
              I[i][j] = I[ii][j]// replace i'th row by ii'th
              I[ii][j] = e // replace ii'th by temp
            }
            // don't bother checking other rows since we've swapped
            break
          }
        }
        // get the new diagonal
        e = C[i][i]
        // if it's still 0, not invertible (error)
        if (e === 0) { return }
      }

      // Scale this row down by e (so we have a 1 on the diagonal)
      for (j = 0; j < dim; j++) {
        C[i][j] = C[i][j] / e // apply to original matrix
        I[i][j] = I[i][j] / e // apply to identity
      }

      // Subtract this row (scaled appropriately for each row) from ALL of
      // the other rows so that there will be 0's in this column in the
      // rows above and below this one
      for (ii = 0; ii < dim; ii++) {
        // Only apply to other rows (we want a 1 on the diagonal)
        if (ii === i) { continue }

        // We want to change this element to 0
        e = C[ii][i]

        // Subtract (the row above(or below) scaled by e) from (the
        // current row) but start at the i'th column and assume all the
        // stuff left of diagonal is 0 (which it should be if we made this
        // algorithm correctly)
        for (j = 0; j < dim; j++) {
          C[ii][j] -= e * C[i][j] // apply to original matrix
          I[ii][j] -= e * I[i][j] // apply to identity
        }
      }
    }

    // we've done all operations, C should be the identity
    // matrix I should be the inverse:
    return I
  }

  /*
	add: function (m1, m2) {
		if (m1.length !== m2.length)
			throw new Error("m1 and m2 do not have the same dimensions");
		var row;
		for (row = 0; row < m1.length; row++) {
			if (m1[row].length !== m2[row].length)
				throw new Error("m1 and m2 do not have the same dimensions");
			if (m1[0].length !== m1[row].length)
				throw new Error("input is not rectangular");
		}
		var output = [];
		if (Array.isArray(m1[0])) {
			for (row = 0; row < m1.length; row++) {
				output[row] = [];
				for (var col = 0; col < m1[row].length; col++) {
					output[row][col] = m1[row][col] + m2[row][col];
				}
			}
		}
		else {
			for (row = 0; row < m1.length; row++)
				output[row] = m1[row] + m2[row];
		}
		return output;
	},
	subtract: function (m1, m2) {
		if (m1.length !== m2.length)
			throw new Error("m1 and m2 do not have the same dimensions");
		var row;
		for (row = 0; row < m1.length; row++) {
			if (m1[row].length !== m2[row].length)
				throw new Error("m1 and m2 do not have the same dimensions");
			if (m1[0].length !== m1[row].length)
				throw new Error("input is not rectangular");
		}
		var output = [];
		if (Array.isArray(m1[0])) {
			for (row = 0; row < m1.length; row++) {
				output[row] = [];
				for (var col = 0; col < m1[row].length; col++) {
					output[row][col] = m1[row][col] - m2[row][col];
				}
			}
		}
		else {
			for (row = 0; row < m1.length; row++)
				output[row] = m1[row] - m2[row];
		}
		return output;
	},
	scale: function (m, a) {
		var row;
			for (row = 0; row < m.length; row++) {
				if (m[0].length !== m[row].length)
					throw new Error("input is not rectangular");
			}
			var output = [];
			if (Array.isArray(m[0])) {
				for (row = 0; row < m.length; row++) {
					output[row] = [];
					for (var col = 0; col < m[row].length; col++) {
						output[row][col] = m[row][col] * a;
					}
				}
			}
			else {
				for (row = 0; row < m.length; row++)
					output[row] = m[row] * a;
			}
			return output;
	},
	transpose: function (m1) {
		if (m1.length === 0)
			throw new Error("Array size = 0");
		var row;
		for (row = 0; row < m1.length; row++) {
			if (m1[0].length !== m1[row].length)
				throw new Error("m1 is not rectangular");
			if (Array.isArray(m1[row]) === false)
				throw new Error("m1 is not two dimensional");
		}
		var output = [];
		for (var col = 0; col < m1[0].length; col++) {
			output[col] = [];
			for (row = 0; row < m1.length; row++) {
				output[col][row] = m1[row][col];
			}
		}
		return output;
	},
	multiply: function (m1, m2) {
		if (m1.length === 0)
			throw new Error("m1.length === 0");
		if (m2.length === 0)
			throw new Error("m2.length === 0");
		var row;
		for (row = 0; row < m1.length; row++) {
			if (m1[0].length !== m1[row].length)
				throw new Error("m1 is not rectangular");
			if (Array.isArray(m1[row]) === false)
				throw new Error("m1 is not two dimensional");
		}
		for (row = 0; row < m2.length; row++) {
			if (m2[0].length !== m2[row].length)
				throw new Error("m2 is not rectangular");
			if (Array.isArray(m2[row]) === false)
				throw new Error("m2 is not two dimensional");
		}
		if (m1[0].length !== m2.length)
			throw new Error("Num m1 cols !== Num m2 rows");
		var result = [];
		for (var i = 0; i < m1.length; i++) {
			result[i] = [];
			for (var j = 0; j < m2[0].length; j++) {
				var sum = 0;
				for (var k = 0; k < m1[0].length; k++) {
					sum += m1[i][k] * m2[k][j];
				}
				result[i][j] = sum;
			}
		}
		return result;
	},

	inverse: function (m) {
		if (m.length === 0)
			throw new Error("m.length === 0");
		for (var row = 0; row < m.length; row++) {
			if (m[0].length !== m[row].length)
				throw new Error("m is not rectangular");
			if (Array.isArray(m[row]) === false)
				throw new Error("m is not two dimensional");
		}
		// I use Gaussian Elimination to calculate the inverse:
		// (1) 'augment' the matrix (left) by the identity (on the right)
		// (2) Turn the matrix on the left into the identity by elementary row ops
		// (3) The matrix on the right is the inverse (was the identity matrix)
		// There are 3 elementary row ops: (I combine b and c in my code)
		// (a) Swap 2 rows
		// (b) Multiply a row by a scalar
		// (c) Add 2 rows
		//if the matrix isn't square: exit (error)
		if (m.length !== m[0].length) {
			return;
		}
		//create the identity matrix (I), and a copy (C) of the original
		var i = 0, ii = 0, j = 0, dim = m.length, e = 0;
		var I = [], c = [];
		for (i = 0; i < dim; i += 1) {
			// Create the row
			I[I.length] = [];
			c[c.length] = [];
			for (j = 0; j < dim; j += 1) {
				//if we're on the diagonal, put a 1 (for identity)
				if (i === j) {
					I[i][j] = 1;
				}
				else {
					I[i][j] = 0;
				}
				// Also, make the copy of the original
				c[i][j] = m[i][j];
			}
		}
		// Perform elementary row operations
		for (i = 0; i < dim; i += 1) {
			// get the element e on the diagonal
			e = c[i][i];
			// if we have a 0 on the diagonal (we'll need to swap with a lower row)
			if (e === 0) {
				//look through every row below the i'th row
				for (ii = i + 1; ii < dim; ii += 1) {
				//if the ii'th row has a non-0 in the i'th col
					if (c[ii][i] !== 0) {
						//it would make the diagonal have a non-0 so swap it
						for (j = 0; j < dim; j++) {
							e = c[i][j]; //temp store i'th row
							c[i][j] = c[ii][j]; //replace i'th row by ii'th
							c[ii][j] = e; //replace ii'th by temp
							e = I[i][j]; //temp store i'th row
							I[i][j] = I[ii][j]; //replace i'th row by ii'th
							I[ii][j] = e; //replace ii'th by temp
						}
						//don't bother checking other rows since we've swapped
						break;
					}
				}
				//get the new diagonal
				e = c[i][i];
				//if it's still 0, not invertible (error)
				if (e === 0) {
					return;
				}
			}
			// Scale this row down by e (so we have a 1 on the diagonal)
			for (j = 0; j < dim; j++) {
				c[i][j] = c[i][j] / e; //apply to original matrix
				I[i][j] = I[i][j] / e; //apply to identity
			}
			// Subtract this row (scaled appropriately for each row) from ALL of
			// the other rows so that there will be 0's in this column in the
			// rows above and below this one
			for (ii = 0; ii < dim; ii++) {
				// Only apply to other rows (we want a 1 on the diagonal)
				if (ii === i) {
					continue;
				}
				// We want to change this element to 0
				e = c[ii][i];
				// Subtract (the row above(or below) scaled by e) from (the
				// current row) but start at the i'th column and assume all the
				// stuff left of diagonal is 0 (which it should be if we made this
				// algorithm correctly)
				for (j = 0; j < dim; j++) {
					c[ii][j] -= e * c[i][j]; //apply to original matrix
					I[ii][j] -= e * I[i][j]; //apply to identity
				}
			}
		}
		//we've done all operations, C should be the identity
		//matrix I should be the inverse:
		return I;
	},

  copySub: function (dst, src, srcRowOffset, srcColOffset, dstRowOffset, dstColOffset, nrows, ncols) {
		if (src.length === 0)
			throw new Error("src.length === 0");
		if (dst.length === 0)
			throw new Error("dst.length === 0");
		if (srcRowOffset < 0 || srcColOffset < 0 || src.length < (srcRowOffset + nrows) || src[0].length < (srcColOffset + ncols))
			throw new Error("size error");
		else if (dstRowOffset < 0 || dstColOffset < 0 || dst.length < (dstRowOffset + nrows) || dst[0].length < (dstColOffset + ncols))
			throw new Error("size error");
		for (var i = 0; i < nrows; i++)
			for (var j = 0; j < ncols; j++)
				dst[dstRowOffset + i][dstColOffset + j] = src[srcRowOffset + i][srcColOffset + j];
	},

  cfLin_svdcmp: function (a, m, n, w, v) {
		var flag, i, its, j, jj, k, c, f, h, s, x, y, z;
		var l = 0;
		var nm = 0;
		var anorm = 0.0;
		var g = 0.0;
		var scale = 0.0;
		var rv1 = [];
		var eps = 1e-15;
		/* matrix not 0 augmented *//*
if (m < n)
	return false;
rv1 = new Array(n);
for (i = 0; i < n; i++) {
	l = i + 1;
	gssUtils.assert(i >= 0 && i < n, "i >= 0 && i < n");
	rv1[i] = scale * g;
	g = s = scale = 0.0;
	if (i < m) {
		for (k = i; k < m; k++)
			scale += Math.abs(a[k][i]);
		if (scale !== 0.0) {
			for (k = i; k < m; k++) {
				a[k][i] /= scale;
				s += a[k][i] * a[k][i];
			}
			f = a[i][i];
			g = -sign(Math.sqrt(s), f);
			h = f * g - s;
			a[i][i] = f - g;
			if (i !== n - 1)
				for (j = l; j < n; j++) {
					for (s = 0.0, k = i; k < m; k++)
						s += a[k][i] * a[k][j];
					f = s / h;
					for (k = i; k < m; k++)
						a[k][j] += f * a[k][i];
			}
			for (k = i; k < m; k++)
				a[k][i] *= scale;
		}
	}
	w[i] = scale * g;
	g = s = scale = 0.0;
	if ((i < m) && (i !== n - 1)) {
		for (k = l; k < n; k++)
			scale += Math.abs(a[i][k]);
		if (scale !== 0.0) {
			for (k = l; k < n; k++) {
				a[i][k] /= scale;
				s += a[i][k] * a[i][k];
			}
			f = a[i][l];
			g = -sign(Math.sqrt(s), f);
			h = f * g - s;
			a[i][l] = f - g;
			gssUtils.assert(l >= 0, "l >= 0");
			for (k = l; k < n; k++)
				rv1[k] = a[i][k] / h;
			if (i !== m - 1)
				for (j = l; j < m; j++) {
					for (s = 0.0, k = l; k < n; k++)
						s += a[j][k] * a[i][k];
					for (k = l; k < n; k++)
						a[j][k] += s * rv1[k];
				}
				for (k = l; k < n; k++)
					a[i][k] *= scale;
		}
	}
	gssUtils.assert(i >= 0 && i < n, "i >= 0 && i < n");
	anorm = setMax(anorm, (Math.abs(w[i]) + Math.abs(rv1[i])));
}
for (i = (n - 1); i >= 0; i--) {
	if (i < (n - 1)) {
		if (g !== 0.0) {
			for (j = l; j < n; j++)
				v[j][i] = (a[i][j] / a[i][l]) / g;
			for (j = l; j < n; j++) {
				for (s = 0.0, k = l; k < n; k++)
					s += a[i][k] * v[k][j];
				for (k = l; k < n; k++)
					v[k][j] += s * v[k][i];
			}
		}
		for (j = l; j < n; j++)
			v[i][j] = v[j][i] = 0.0;
	}
	v[i][i] = 1.0;
	g = rv1[i];
	l = i;
}
for (i = (n - 1); i >= 0; i--) {
	l = i + 1;
	g = w[i];
	if (i < (n - 1))
		for (j = l; j < n; j++)
			a[i][j] = 0.0;
		if (g !== 0.0) {
			g = 1.0 / g;
			if (i !== (n - 1))
				for (j = l; j < n; j++) {
					for (s = 0.0, k = l; k < m; k++)
						s += a[k][i] * a[k][j];
						f = (s / a[i][i]) * g;
						for (k = i; k < m; k++)
							a[k][j] += f * a[k][i];
				}
				for (j = i; j < m; j++)
					a[j][i] *= g;
		}
		else
			for (j = i; j < m; j++)
				a[j][i] = 0.0;
			++a[i][i];
}
eps *= anorm;
for (k = (n - 1); k >= 0; k--) {
	for (its = 1; its <= 100; its++) {
		flag = 1;
		for (l = k; l >= 0; l--) {
			nm = l - 1;
			if (Math.abs(rv1[l]) <= eps) {
				flag = 0;
				break;
			}
			if (Math.abs(w[nm]) <= eps)
				break;
		}
		if (flag !== 0.0) {
			c = 0.0;
			s = 1.0;
			for (i = l; i <= k; i++) {
				f = s * rv1[i];
				if (Math.abs(f) > eps) {
					g = w[i];
					h = setPythag(f, g);
					w[i] = h;
					h = 1.0 / h;
					c = g * h;
					s = -f * h;
					for (j = 0; j < m; j++) {
						y = a[j][nm];
						z = a[j][i];
						a[j][nm] = y * c + z * s;
						a[j][i] = z * c - y * s;
					}
				}
			}
		}
		z = w[k];
		if (l === k) {
			if (z < 0.0) {
				w[k] = -z;
				for (j = 0; j < n; j++)
					v[j][k] = -v[j][k];
			}
			break;
		}
		if (its === 100) {
			rv1 = [];
			return false;
		}
		x = w[l];
		nm = k - 1;
		y = w[nm];
		g = rv1[nm];
		h = rv1[k];
		f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
		g = setPythag(f, 1.0);
		f = ((x - z) * (x + z) + h * ((y / (f + sign(g, f))) - h)) / x;
		c = s = 1.0;
		for (j = l; j <= nm; j++) {
			i = j + 1;
			g = rv1[i];
			y = w[i];
			h = s * g;
			g = c * g;
			z = setPythag(f, h);
			gssUtils.assert(j >= 0 && j < n, "j >= 0 && j < n");
			rv1[j] = z;
			c = f / z;
			s = h / z;
			f = x * c + g * s;
			g = g * c - x * s;
			h = y * s;
			y = y * c;
			for (jj = 0; jj < n; jj++) {
				x = v[jj][j];
				z = v[jj][i];
				v[jj][j] = x * c + z * s;
				v[jj][i] = z * c - x * s;
			}
			z = setPythag(f, h);
			w[j] = z;
			if (z !== 0.0) {
				z = 1.0 / z;
				c = f * z;
				s = h * z;
			}
			f = (c * g) + (s * y);
			x = (c * y) - (s * g);
			for (jj = 0; jj < m; jj++) {
				y = a[jj][j];
				z = a[jj][i];
				a[jj][j] = y * c + z * s;
				a[jj][i] = z * c - y * s;
			}
		}
		gssUtils.assert(l >= 0 && l < n, "l >= 0 && l < n");
		gssUtils.assert(k >= 0 && k < n, "k >= 0 && k < n");
		rv1[l] = 0.0;
		rv1[k] = f;
		w[k] = x;
	}
}
rv1 = [];
return true;
},

SVD: function (m, q1, v, q2) {
if (m.length === 0)
	throw new Error("m.length === 0");
if (m[0].length === 0)
	throw new Error("m[0].length === 0");
var nrows = m.length;
var ncols = m[0].length;
var nrowsrows = (nrows < ncols) ? ncols : nrows;
var aaug = new Array(nrowsrows);
var i;
for (i = 0; i < nrowsrows; i++)
	aaug[i] = new Array(ncols);
this.copySub(aaug, m, 0, 0, 0, 0, nrows, ncols);
var eigenvalues = new Array(ncols);
if (!this.cfLin_svdcmp(aaug, aaug.length, aaug[0].length, eigenvalues, q2))
	throw new Error("Unstable Error");
for (i = 0; i < ncols; i++)
	v[i][i] = eigenvalues[i];
this.copySub(q1, aaug, 0, 0, 0, 0, nrows, ncols);
},
pseudoinverse: function (m, maxCnum) {
var minW, maxW, threshold;
var mCopy;
if (m.length === 0)
	throw new Error("m.length === 0");
if (m[0].length === 0)
	throw new Error("m[0].length === 0");
var flag = (m.length >= m[0].length);
if (flag)
	mCopy = m;
else
	mCopy = this.transpose(m);
var nrows = mCopy.length;
var ncols = mCopy[0].length;
var q1 = new Array(nrows);
var i;
var array;
var j;
for (i = 0; i < nrows; i++) {
	q1[i] = new Array(ncols);
	array = q1[i];
	for (j in array)
	if (array.hasOwnProperty(j))
		array[j] = 0.0;
}
var v = new Array(ncols);
for (i = 0; i < ncols; i++) {
	v[i] = new Array(ncols);
	array = v[i];
	for (j in array)
		if (array.hasOwnProperty(j))
			array[j] = 0.0;
}
var q2 = new Array(ncols);
for (i = 0; i < ncols; i++) {
	q2[i] = new Array(ncols);
	array = q2[i];
	for (j in array)
		if (array.hasOwnProperty(j))
			array[j] = 0.0;
}
var answer;
this.SVD(mCopy, q1, v, q2);
maxW = minW = v[0][0];
for (i = 1; i < v[0].length; i++) {
	if (maxW < v[i][i])
		maxW = v[i][i];
	if (minW > v[i][i])
		minW = v[i][i];
}
threshold = maxW / Math.sqrt(maxCnum);
if (threshold < 10e-10)
	answer = mCopy;
else {
	var vinv = new Array(v[0].length);
	for (i = 0; i < v[0].length; i++) {
		vinv[i] = new Array(v.length);
		array = vinv[i];
		for (j in array)
			if (array.hasOwnProperty(j))
				array[j] = 0.0;
	}
	for (i = 0; i < vinv[0].length; i++) {
		if (v[i][i] < threshold)
			vinv[i][i] = v[i][i] / (threshold * threshold);
		else
			vinv[i][i] = 1.0 / v[i][i];
	}
	var psinv = this.multiply(q2, vinv);
	answer = this.multiply(psinv, this.transpose(q1));
}
if (flag)
	return answer;
else
	return this.transpose(answer);
} */
}

function ccResiduals (maxResidual, rmsResidual) {
  this.maxResidual = maxResidual
  this.rmsResidual = rmsResidual

  this.initFromObject = function (obj) {
    this.maxResidual = obj['maxResidual']
    this.rmsResidual = obj['rmsResidual']
  }

  this.getMaxResidual = function () { return this.maxResidual }
  this.getrmsResidual = function () { return this.rmsResidual }
}

// set a vector of ccResiduals from 2 vector's of doubles
function setMaxAndRmsResiduals (vecResiduals, // array of ccResiduals
  maxResiduals,
  rmsResiduals) {
  assert(rmsResiduals.lenth === maxResiduals.length)

  for (var i = 0; i < vecResiduals.length; i++) {
    vecResiduals[i] = new ccResiduals()
    vecResiduals[i].maximumAndRms(maxResiduals[i], rmsResiduals[i])
  }
}

function getFlipHandednessXform (flipHandedness) {
  xform = new cc2XformLinear()
  var a11
  if (flipHandedness) { a11 = -1 } else { a11 = 1 }
  xform.setXform([[1.0, 0.0, 0.0], [0.0, a11, 0.0]])
  return xform
}

function isHandednessFlipped (x) {
  return x.xform[0][0] * x.xform[1][1] - x.xform[0][1] * x.xform[1][0] < 0
}

function cc2Rigid () {
  // Stored as a 2x3 array
  this.xform = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
  this.cosAngle = 1.0
  this.sinAngle = 0.0
  this.initFromObject = function (obj) {
    this.xform = obj['xform']
    this.cosAngle = obj['cosAngle']
    this.sinAngle = obj['sinAngle']
  }

  // Returns 1D array of length 2
  this.trans = function () {
    return [this.xform[0][2], this.xform[1][2]]
  }

  this.setXform = function (thetaInDegrees, trans) {
    if (trans.length !== 2) { throw new Error('trans length !== 2') }
    var thetaInRadians = Math.PI * thetaInDegrees / 180.0
    var cosAngle = Math.cos(thetaInRadians)
    var sinAngle = Math.sin(thetaInRadians)
    this.xform[0][0] = cosAngle
    this.xform[0][1] = -sinAngle
    this.xform[0][2] = trans[0]
    this.xform[1][0] = sinAngle
    this.xform[1][1] = cosAngle
    this.xform[1][2] = trans[1]
  }

  this.compose = function (xform) {
    var res = new cc2Rigid()
    res.xform = [[
      this.xform[0][0] * xform.xform[0][0] + this.xform[0][1] * xform.xform[1][0],
      this.xform[0][0] * xform.xform[0][1] + this.xform[0][1] * xform.xform[1][1],
      this.xform[0][0] * xform.xform[0][2] + this.xform[0][1] * xform.xform[1][2] + this.xform[0][2]
    ],
    [
      this.xform[1][0] * xform.xform[0][0] + this.xform[1][1] * xform.xform[1][0],
      this.xform[1][0] * xform.xform[0][1] + this.xform[1][1] * xform.xform[1][1],
      this.xform[1][0] * xform.xform[0][2] + this.xform[1][1] * xform.xform[1][2] + this.xform[1][2]
    ]]
    return res
  }

  this.mapPoint = function (point) {
    if (point.length !== 2) { throw new Error('Array length !== 2') }
    var mappedX = this.xform[0][0] * point[0] + this.xform[0][1] * point[1] + this.xform[0][2]
    var mappedY = this.xform[1][0] * point[0] + this.xform[1][1] * point[1] + this.xform[1][2]
    return [mappedX, mappedY]
  }

  this.inverse = function () {
    var det = this.xform[0][0] * this.xform[1][1] - this.xform[1][0] * this.xform[0][1]
    if (det === 0) { throw Error('Cannot be inverted') }
    var inv = new cc2XformLinear()
    inv.setXform([[this.xform[1][1] / det, -this.xform[0][1] / det, (this.xform[0][1] * this.xform[1][2] - this.xform[0][2] * this.xform[1][1]) / det],
      [-this.xform[1][0] / det, this.xform[0][0] / det, (this.xform[0][2] * this.xform[1][0] - this.xform[0][0] * this.xform[1][2]) / det]])
    return inv
  }

  this.angleInDegrees = function () {
    // Full angle so use arctan(y,x) = atan2(y,x)
    return Math.atan2(this.xform[1][0], this.xform[0][0]) * 180.0 / Math.PI
  }

  this.cosAngle = function () {
    return this.xform[0][0]
  }

  this.sinAngle = function () {
    return this.xform[1][0]
  }
}

function cc2XformLinear () {
  // Stored as a 2x3 array
  this.xform = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
  var dt_ = 1
  this.initFromObject = function (obj) {
    this.xform = obj['xform']
    this.dt = obj['dt']
  }

  // Returns 1D array of length 2
  this.trans = function () {
    return [this.xform[0][2], this.xform[1][2]]
  }

  this.setXform = function (xform) {
    if (xform.length !== 2) { throw new Error('Array length !== 2') }
    if (xform[0].length !== 3) { throw new Error('point[0] length !== 3') }
    if (xform[1].length !== 3) { throw new Error('point[1] length !== 3') }
    this.xform[0][0] = xform[0][0]
    this.xform[0][1] = xform[0][1]
    this.xform[0][2] = xform[0][2]
    this.xform[1][0] = xform[1][0]
    this.xform[1][1] = xform[1][1]
    this.xform[1][2] = xform[1][2]
  }

  this.setdt = function () { this.dt_ = this.xform[0][0] * this.xform[1][1] - this.xform[0][1] * this.xform[1][0] }
  this.setXformScaleRotation = function (xRotInDegrees, yRotInDegrees, xScale, yScale, trans) {
    if (trans.length !== 2) { throw new Error('trans length !== 2') }
    var xRot = xRotInDegrees * Math.PI / 180.0
    var yRot = yRotInDegrees * Math.PI / 180.0
    this.xform[0][0] = xScale * Math.cos(xRot)
    this.xform[0][1] = -yScale * Math.sin(yRot)
    this.xform[1][0] = xScale * Math.sin(xRot)
    this.xform[1][1] = yScale * Math.cos(yRot)
    this.xform[0][2] = trans[0]
    this.xform[1][2] = trans[1]
    this.setdt()
  }

  this.setXformShearAspect = function (scale, aspect, shearInDegrees, rotationInDegrees, trans) {
    var rotation = rotationInDegrees * Math.PI / 180.0
    var shear = shearInDegrees * Math.PI / 180.0
    var cosR = Math.cos(rotation)
    var sinR = Math.sin(rotation)
    var tanK = Math.tan(shear)
    this.xform[0][0] = scale * cosR
    this.xform[0][1] = aspect * scale * (-sinR - cosR * tanK)
    this.xform[1][0] = scale * sinR
    this.xform[1][1] = aspect * scale * (cosR - sinR * tanK)
    this.xform[0][2] = trans[0]
    this.xform[1][2] = trans[1]
    this.setdt()
  }

  this.compose = function (xform) {
    var res = new cc2XformLinear()
    res.xform = [[
      this.xform[0][0] * xform.xform[0][0] + this.xform[0][1] * xform.xform[1][0],
      this.xform[0][0] * xform.xform[0][1] + this.xform[0][1] * xform.xform[1][1],
      this.xform[0][0] * xform.xform[0][2] + this.xform[0][1] * xform.xform[1][2] + this.xform[0][2]
    ],
    [
      this.xform[1][0] * xform.xform[0][0] + this.xform[1][1] * xform.xform[1][0],
      this.xform[1][0] * xform.xform[0][1] + this.xform[1][1] * xform.xform[1][1],
      this.xform[1][0] * xform.xform[0][2] + this.xform[1][1] * xform.xform[1][2] + this.xform[1][2]
    ]]
    return res
  }

  this.mapPoint = function (point) {
    if (point.length !== 2) { throw new Error('Array length !== 2') }
    var mappedX = this.xform[0][0] * point[0] + this.xform[0][1] * point[1] + this.xform[0][2]
    var mappedY = this.xform[1][0] * point[0] + this.xform[1][1] * point[1] + this.xform[1][2]
    return [mappedX, mappedY]
  }

  function determinant (xform) {
    return xform[0][0] * xform[1][1] - xform[1][0] * xform[0][1]
  };

  this.inverse = function () {
    var det = determinant(this.xform)
    if (det === 0) { throw Error('Cannot be inverted') }
    var inv = new cc2XformLinear()
    inv.setXform([[this.xform[1][1] / det, -this.xform[0][1] / det, (this.xform[0][1] * this.xform[1][2] - this.xform[0][2] * this.xform[1][1]) / det],
      [-this.xform[1][0] / det, this.xform[0][0] / det, (this.xform[0][2] * this.xform[1][0] - this.xform[0][0] * this.xform[1][2]) / det]])
    return inv
  }

  function cfSqr (val) {
    return val * val
  }
  // Decomposition methods
  // All check for singularity throw ccMathError::Singular if so
  // Note xRot,yRot,xScale,yScale are a group.
  this.xRotInDegrees = function () {
    if (isSingular(this.xform)) { throw new Error('Singular') }
    // Full angle so use arctan(y,x) = atan2(y,x)
    return Math.atan2(this.xform[1][0], this.xform[0][0]) * 180.0 / Math.PI
  }

  this.yRotInDegrees = function () {
    if (isSingular(this.xform)) { throw new Error('Singular') }
    // Full angle so use arctan(y,x) = atan2(y,x)
    return Math.atan2(-this.xform[0][1], this.xform[1][1]) * 180.0 / Math.PI
  }

  this.xScale = function () {
    if (isSingular(this.xform)) { throw new Error('Singular') }
    return Math.sqrt(cfSqr(this.xform[0][0]) + cfSqr(this.xform[1][0]))
  }

  this.yScale = function () {
    if (isSingular(this.xform)) { throw new Error('Singular') }
    return Math.sqrt(cfSqr(this.xform[0][1]) + cfSqr(this.xform[1][1]))
  }

  // Note scale,aspect,shear,rotation are a group.
  this.scale = function () {
    if (isSingular(this.xform)) { throw new Error('Singular') }
    return Math.sqrt(cfSqr(this.xform[0][0]) + cfSqr(this.xform[1][0]))
  }

  this.aspect = function () {
    if (isSingular(this.xform)) { throw new Error('Singular') }
    return determinant(this.xform) / (cfSqr(this.xform[0][0]) + cfSqr(this.xform[1][0]))
  }

  this.shearInDegrees = function () {
    if (isSingular(this.xform)) { throw new Error('Singular') }
    // Shear is limited to (-PI/2, PI/2) so use arctan(x) = atan(x)
    return (-Math.atan((this.xform[0][0] * this.xform[0][1] + this.xform[1][0] * this.xform[1][1]) / determinant(this.xform))) * 180.0 / Math.PI
  }

  this.rotationInDegrees = function () {
    if (isSingular(this.xform)) { throw new Error('Singular') }
    // Full angle so use arctan(y,x) = atan2(y,x)
    return Math.atan2(this.xform[1][0], this.xform[0][0]) * 180.0 / Math.PI
  }

  this.cosAngle = function () {
    var rot = Math.atan2(this.xform[1][0], this.xform[0][0])
    return Math.cos(rot)
  }

  this.sinAngle = function () {
    var rot = Math.atan2(this.xform[1][0], this.xform[0][0])
    return Math.sin(rot)
  }

  function isSingular (xform) {
    return determinant(xform) === 0
  }
}

function xya2Rigid (xyaStr) {
  var tmpRigid = new cc2Rigid()
  var xya = JSON.parse(xyaStr)
  var x = 0.0
  var y = 0.0
  var a = 0.0

  x = xya['X']
  y = xya['Y']
  a = xya['Angle']

  var trans = [x * 1.0, y * 1.0]

  tmpRigid.setXform(a, trans)
  return tmpRigid
}

function xya2XformLinear (xyaStr) {
  var tmpRigid = new cc2Rigid()
  var xya = JSON.parse(xyaStr)
  var x = 0.0
  var y = 0.0
  var a = 0.0

  x = xya['X']
  y = xya['Y']
  a = xya['Angle']

  var trans = [x * 1.0, y * 1.0]

  tmpRigid.setXform(a, trans)
  var xfLin = new cc2XformLinear()
  xfLin.setXform(tmpRigid.xform)
  return xfLin
}

function lin2Rigid (xfLinear) {
  var xfRigid = new cc2Rigid()
  xfRigid.setXform(xfLinear.xRotInDegrees(), xfLinear.trans())
  return xfRigid
}

function convertToCorrected (xforms, uncorrectedHomeFromHand) {
  var hfm = new cc2XformLinear()
  // hfm.setXform(JSON.parse(xforms["Home2DFromMotion2D"]).xform);
  hfm.setXform(xforms.Home2DFromMotion2D.xform)

  var transOnly = new cc2Rigid()
  transOnly.setXform(0, uncorrectedHomeFromHand.trans())
  var transHome2D = hfm.compose(transOnly)

  var home2DFromStage2D = new cc2Rigid()
  home2DFromStage2D.setXform(uncorrectedHomeFromHand.angleInDegrees(), transHome2D.trans())
  return home2DFromStage2D
}

function convertToUncorrected (xforms, correctedHomeFromHand) {
  var hfm = new cc2XformLinear()
  // hfm.setXform(JSON.parse(xforms["Home2DFromMotion2D"]).xform);
  hfm.setXform(xforms.Home2DFromMotion2D.xform)

  var transMotion2D = new cc2XformLinear()
  var transOnly = new cc2Rigid()
  transOnly.setXform(0, correctedHomeFromHand.trans())
  transMotion2D = hfm.inverse().compose(transOnly)

  var uncorrectedHome2DFromStage2D = new cc2Rigid()
  uncorrectedHome2DFromStage2D.setXform(correctedHomeFromHand.angleInDegrees(), transMotion2D.trans())
  return uncorrectedHome2DFromStage2D
}

function ccMultiViewPlanarMotionCalibResult () {
  this.isComputed_ = false
  this.isCameraMoving_ = false
  this.motionXAxisHome2D_ = [1, 0]
  this.motionYAxisHome2D_ = [0, 1]
  this.image2DFromCamera2D_ = new cc2XformLinear()
  this.home2DFromStationaryCamera2D_ = new cc2XformLinear()
  this.stage2DFromMovingCamera2D_ = new cc2XformLinear()
  this.home2DFromStationaryPlate2D_ = new cc2XformLinear()
  this.stage2DFromMovingPlate2D_ = new cc2XformLinear()
  this.home2DFromMotion2D_ = new cc2XformLinear()
  this.overallResidualsImage2D_ = new ccResiduals(0, 0)
  this.overallResidualsHome2D_ = new ccResiduals(0, 0)
  this.hasEstimatedHome2DFromStage2DPoses_ = []
  this.estimatedHome2DFromStage2DPoses_ = []
  this.isSingleViewResidualsValid_ = []
  this.singleViewResidualsImage2D_ = []
  this.singleViewResidualsPlate2D_ = []
  this.initFromObject = function (resultObject) {
    this.isComputed_ = resultObject['isComputed_']
    this.isCameraMoving_ = resultObject['isCameraMoving_']
    this.motionXAxisHome2D_ = resultObject['motionXAxisHome2D_']
    this.motionYAxisHome2D = resultObject['motionYAxisHome2D_']
    this.image2DFromCamera2D_.initFromObject(resultObject['image2DFromCamera2D_'])
    this.home2DFromStationaryCamera2D_.initFromObject(resultObject['home2DFromStationaryCamera2D_'])
    this.stage2DFromMovingCamera2D_.initFromObject(resultObject['stage2DFromMovingCamera2D_'])
    this.home2DFromStationaryPlate2D_.initFromObject(resultObject['home2DFromStationaryPlate2D_'])
    this.stage2DFromMovingPlate2D_.initFromObject(resultObject['stage2DFromMovingPlate2D_'])
    this.home2DFromMotion2D_.initFromObject(resultObject['home2DFromMotion2D_'])
    this.overallResidualsImage2D_.initFromObject(resultObject['overallResidualsImage2D_'])
    this.overallResidualsHome2D_.initFromObject(resultObject['overallResidualsHome2D_'])
    // this.hasEstimatedHome2DFromStage2DPoses_.initFromObject(resultObject["hasEstimatedHome2DFromStage2DPoses_"]);
    // this.estimatedHome2DFromStage2DPoses_.initFromObject(resultObject["estimatedHome2DFromStage2DPoses_"]);
    // this.isSingleViewResidualsValid_.initFromObject(resultObject["isSingleViewResidualsValid_"]);
    // this.singleViewResidualsImage2D_.initFromObject(resultObject["singleViewResidualsImage2D_"]);
    // this.singleViewResidualsPlate2D_.initFromObject(resultObject["singleViewResidualsPlate2D_"]);
  }

  // getters
  this.isCameraMoving = function () {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    return this.isCameraMoving_
  }

  this.image2DFromCamera2D = function () {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    return this.image2DFromCamera2D_
  }

  this.image2DFromHome2D = function (uncorrectedHome2DFromStage2D) {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    var home2DFromCamera2D
    if (this.isCameraMoving) { home2DFromCamera2D = (this.convertUncorrectedHome2DFromStage2DToHome2DFromStage2D(uncorrectedHome2DFromStage2D).compose(this.stage2DFromMovingCamera2D_)) } else { home2DFromCamera2D = this.home2DFromStationaryCamera2D_ }
    return this.image2DFromCamera2D_.compose(home2DFromCamera2D.inverse())
  }

  this.home2DFromStationaryCamera2D = function () {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    if (this.isCameraMoving_) { throw new Error('The camera must be stationary.') }
    return this.home2DFromStationaryCamera2D_
  }

  this.stage2DFromMovingCamera2D = function () {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    if (!this.isCameraMoving_) { throw new Error('The camera must be moving.') }
    return this.stage2DFromMovingCamera2D_
  }

  this.home2DFromStationaryPlate2D = function () {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    if (!this.isCameraMoving_) { throw new Error('The calibration plate must be stationary.') }
    return this.home2DFromStationaryPlate2D_
  }

  this.stage2DFromMovingPlate2D = function () {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    if (this.isCameraMoving) { throw new Error('The calibration plate must be moving.') }
    return this.stage2DFromMovingPlate2D_
  }

  this.stage2DFromMovingPlate2D = function () {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    return this.motionXAxisHome2D_
  }

  this.motionYAxisHome2D = function () {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    return this.motionYAxisHome2D_
  }

  // -----------------------------------------------------------------------------
  // NOTE:
  // For the conversion functions, assume that the theta axis of the stage
  // is independant of the X and Y axes. So any motion distortion will only
  // affect the translation component of a pose, the rotation remains unchanged.
  // This model of the stage is different from the current implementation of the
  // calibration tool solver which models the stage as having a coupled x,y and
  // theta axis, and any motion distortion is distributed across all axes.
  this.convertUncorrectedHome2DFromStage2DToHome2DFromStage2D = function (uncorrectedHome2DFromStage2D) {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    // return home2DFromMotion2D_ * uncorrectedHome2DFromStage2D * home2DFromMotion2D_.inverse();
    // assume independent theta axis that doesnt suffer any motion distortion
    var transHome2D = this.home2DFromMotion2D_.mapPoint(uncorrectedHome2DFromStage2D.trans())
    var home2DFromStage2D = new cc2Rigid()
    home2DFromStage2D.setXform(uncorrectedHome2DFromStage2D.angleInDegrees(), transHome2D)
    return home2DFromStage2D
  }

  this.convertHome2DFromStage2DToUncorrectedHome2DFromStage2D = function (home2DFromStage2D) {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    // return home2DFromMotion2D_.inverse() * home2DFromStage2D * home2DFromMotion2D_;
    // assume independent theta axis that doesnt suffer any motion distortion
    var transMotion2D = this.home2DFromMotion2D_.inverse().compose(home2DFromStage2D.trans())
    var uncorrectedHome2DFromStage2D = new cc2Rigid()
    uncorrectedHome2DFromStage2D.setXform(home2DFromStage2D.angleInDegrees(), transMotion2D)
    return uncorrectedHome2DFromStage2D
  }

  // -----------------------------------------------------------------------------
  this.isSingleViewResidualsValid = function () {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    return this.isSingleViewResidualsValid_
  }

  this.singleViewResidualsImage2D = function () {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    return this.singleViewResidualsImage2D_
  }

  this.singleViewResidualsPlate2D = function () {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    return this.singleViewResidualsPlate2D_
  }

  this.overallResidualsImage2D = function () {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    return this.overallResidualsImage2D_
  }

  this.overallResidualsHome2D = function () {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    return this.overallResidualsHome2D_
  }

  this.hasEstimatedHome2DFromStage2DPose = function (index) {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    throw new Error('Not implemented')
  }

  this.estimatedHome2DFromStage2DPose = function (index) {
    if (!this.isComputed_) { throw new Error('Not Computed') }
    throw new Error('Not implemented')
  }

  this.set = function (isCameraMoving, // bool
    image2DFromCamera2D, // const cc2XformLinear&
    cameraPlacementPose, // const cc2XformLinear&
    platePlacementPose, // const cc2XformLinear&
    motionXAxisHome2D, // Array of length 2
    motionYAxisHome2D, // Array of length 2
    hasEstHome2DFromStage2DPoses, // Array of bool
    estHome2DFromStage2DPoses, // Array of cc2Rigid
    singleViewResidualsImage2D, // Array of ccResiduals
    singleViewResidualsPlate2D, // Array of bool
    overallResidualsImage2D, // ccResiduals
    overallResidualsHome2D, // ccResiduals
    ifwXform // const cc2XformLinear&
  ) {
    /*
		gssUtils.assert(estHome2DFromStage2DPoses.length !== hasEstHome2DFromStage2DPoses.length);
		gssUtils.assert(estHome2DFromStage2DPoses.length === singleViewResidualsImage2D.length
			|| singleViewResidualsImage2D.length===0);
		gssUtils.assert(estHome2DFromStage2DPoses.length === singleViewResidualsPlate2D.length
			|| singleViewResidualsPlate2D.length === 0);
		*/
    this.isComputed_ = true
    this.isCameraMoving_ = isCameraMoving
    this.image2DFromCamera2D_ = image2DFromCamera2D
    if (this.isCameraMoving_) {
      this.stage2DFromMovingCamera2D_ = cameraPlacementPose
      this.home2DFromStationaryPlate2D_ = platePlacementPose
    } else {
      this.home2DFromStationaryCamera2D_ = cameraPlacementPose
      this.stage2DFromMovingPlate2D_ = platePlacementPose
    }
    this.hasEstimatedHome2DFromStage2DPoses_ = hasEstHome2DFromStage2DPoses
    this.estimatedHome2DFromStage2DPoses_ = estHome2DFromStage2DPoses
    this.motionXAxisHome2D_ = motionXAxisHome2D
    this.motionYAxisHome2D_ = motionYAxisHome2D
    this.singleViewResidualsImage2D_ = singleViewResidualsImage2D
    this.singleViewResidualsPlate2D_ = singleViewResidualsPlate2D
    this.overallResidualsImage2D_ = overallResidualsImage2D
    this.overallResidualsHome2D_ = overallResidualsHome2D
    // cache home2DFromMotion2D
    this.home2DFromMotion2D_ = new cc2XformLinear()
    this.home2DFromMotion2D_.setXform([[this.motionXAxisHome2D_[0], this.motionYAxisHome2D_[0], 0],
      [this.motionXAxisHome2D_[1], this.motionYAxisHome2D_[1], 0]])
    var HUGE_VAL = 10000000000
    this.isSingleViewResidualsValid_ = []
    for (var i = 0; i < this.singleViewResidualsImage2D_.length; i++) {
      if (this.singleViewResidualsPlate2D_[i].maximum() >= HUGE_VAL / 2) { this.isSingleViewResidualsValid_[i] = false} else { this.isSingleViewResidualsValid_[i] = true}
    }
    // cache of deprecated fields
    this.ifwXform_ = ifwXform
  }
}
module.exports.Vector2 = Vector2
module.exports.setPythag = setPythag
module.exports.setMax = setMax
module.exports.sign = sign
module.exports.getDistance = getDistance
module.exports.cfNormalizePoints = cfNormalizePoints
module.exports.ccNmMatrixOp = ccNmMatrixOp
module.exports.cc2Rigid = cc2Rigid
module.exports.cc2XformLinear = cc2XformLinear
module.exports.ccResiduals = ccResiduals
module.exports.xya2Rigid = xya2Rigid
module.exports.xya2XformLinear = xya2XformLinear
module.exports.lin2Rigid = lin2Rigid
module.exports.convertToCorrected = convertToCorrected
module.exports.convertToUncorrected = convertToUncorrected
module.exports.ccMultiViewPlanarMotionCalibResult = ccMultiViewPlanarMotionCalibResult
module.exports.ccNmMatrixOp = ccNmMatrixOp
module.exports.version = version

console.log('Loading CogMath done')
