package quickhull

import (
	r3 "github.com/golang/geo/r3"
)

type ray struct {
	s                 r3.Vector
	v                 r3.Vector
	vInvLengthSquared float64
}

func newRay(s, v r3.Vector) ray {
	return ray{s: s, v: v, vInvLengthSquared: 1 / v.Dot(v)}
}
