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
	endVertex int
	opp       int
	face      int
	next      int
}

const disabledInt = ^int(0)

func (he *halfEdge) disable() {
	he.endVertex = disabledInt
}

func (he halfEdge) isDisabled() bool {
	return he.endVertex == disabledInt
}

type face struct {
	halfEdgeIndex int
}

func newHalfEdgeMesh(builder meshBuilder, vertices []r3.Vector) halfEdgeMesh {
	var heMesh halfEdgeMesh

	faceMapping := make(map[int]int)
	halfEdgeMapping := make(map[int]int)
	vertexMapping := make(map[int]int)

	for i, f := range builder.faces {
		if f.isDisabled() {
			continue
		}

		heMesh.faces = append(heMesh.faces, face{halfEdgeIndex: f.halfEdgeIndex})
		faceMapping[i] = len(heMesh.faces) - 1

		heIndicies := builder.halfEdgeIndicesOfFace(f)
		for _, heIndex := range heIndicies {
			vertexIndex := builder.halfEdges[heIndex].endVertex
			if _, contains := vertexMapping[vertexIndex]; contains {
				heMesh.vertices = append(heMesh.vertices, vertices[vertexIndex])
				vertexMapping[vertexIndex] = len(heMesh.vertices) - 1
			}
		}
	}

	for i, he := range builder.halfEdges {
		if he.isDisabled() {
			continue
		}

		heMesh.halfEdges = append(heMesh.halfEdges, he)
		halfEdgeMapping[i] = len(heMesh.halfEdges) - 1
	}

	for _, f := range heMesh.faces {
		_, contains := halfEdgeMapping[f.halfEdgeIndex]
		assertB(contains)
		f.halfEdgeIndex = halfEdgeMapping[f.halfEdgeIndex]
	}

	for _, he := range heMesh.halfEdges {
		he.face = faceMapping[he.face]
		he.opp = halfEdgeMapping[he.opp]
		he.next = halfEdgeMapping[he.next]
		he.endVertex = vertexMapping[he.endVertex]
	}

	return heMesh
}
