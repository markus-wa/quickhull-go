package quickhull

import "github.com/golang/geo/r3"

func triangleNormal(a r3.Vector, b r3.Vector, c r3.Vector) r3.Vector {
	x := a.X - c.X
	y := a.Y - c.Y
	z := a.Z - c.Z
	rhsX := b.X - c.X
	rhsY := b.Y - c.Y
	rhsZ := b.Z - c.Z
	return r3.Vector{
		X: y*rhsZ - z*rhsY,
		Y: z*rhsX - x*rhsZ,
		Z: x*rhsY - y*rhsX,
	}
}

func signedDistanceToPlane(v r3.Vector, p plane) float64 {
	return p.n.Dot(v) + p.d
}
