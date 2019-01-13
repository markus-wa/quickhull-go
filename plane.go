package quickhull

import (
	"github.com/golang/geo/r3"
)

type plane struct {
	n          r3.Vector
	d          float64 // Signed distance (if normal is of length 1) to the plane from origin
	sqrNLength float64 // Normal length squared
}

func (p plane) isPointOnPositiveSide(q r3.Vector) bool {
	return p.n.Dot(q)+p.d >= 0
}

func newPlane(n r3.Vector, p r3.Vector) plane {
	return plane{n: n, d: -n.Dot(p), sqrNLength: n.Dot(n)}
}
