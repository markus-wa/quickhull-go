package quickhull

import (
	r3 "github.com/golang/geo/r3"
)

type halfEdgeMesh struct {
	vertices  []r3.Vector
	faces     []face
	halfEdges []halfEdge
}

type halfEdge struct {
	endVertex uint
	opp       uint
	face      uint
	next      uint
}

const maxUint = ^uint(0)

func (he *halfEdge) disable() {
	he.endVertex = maxUint
}

func (he halfEdge) isDisabled() bool {
	return he.endVertex == maxUint
}

type face struct {
	halfEdgeIndex uint
}
