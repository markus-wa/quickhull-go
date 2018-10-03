package quickhull

type meshBuilder struct {
	// Mesh data
	faces     []meshBuilderFace
	halfEdges []halfEdge

	// When the mesh is modified and faces and half edges are removed from it, we do not actually remove them from the container vectors.
	// Insted, they are marked as disabled which means that the indices can be reused when we need to add new faces and half edges to the mesh.
	// We store the free indices in the following vectors.
	disabledFaces     []uint
	disabledHalfEdges []uint
}

func (mb *meshBuilder) addFace() int {
	if nDisabled := len(mb.disabledFaces); nDisabled > 0 {
		index := nDisabled - 1
		f := mb.faces[index]
		assertB(f.isDisabled())
		assertB(f.pointsOnPositiveSide == nil)
		f.mostDistantPointDist = 0
		mb.disabledFaces = mb.disabledFaces[:index]

		return index
	}

	mb.faces = append(mb.faces, meshBuilderFace{})
	return len(mb.faces) - 1
}

func (mb *meshBuilder) addHalfEdge() int {
	if nDisabled := len(mb.disabledHalfEdges); nDisabled > 0 {
		index := nDisabled - 1
		mb.disabledHalfEdges = mb.disabledHalfEdges[:index]

		return index
	}

	mb.halfEdges = append(mb.halfEdges, halfEdge{})
	return len(mb.halfEdges) - 1
}

func assertB(b bool) {
	// TODO: Delete assertion
	if !b {
		panic("Assertion failed")
	}
}

type meshBuilderFace struct {
	halfEdgeIndex                   uint
	plane                           plane
	mostDistantPointDist            float64
	visibilityCheckedOnIteration    uint
	isVisibleFaceOnCurrentIteration bool
	inFaceStack                     bool
	horizonEdgesOnCurrentIteration  byte // Bit for each half edge assigned to this face, each being 0 or 1 depending on whether the edge belongs to horizon edge
	pointsOnPositiveSide            []uint
}

func (mbf *meshBuilderFace) disable() {
	mbf.halfEdgeIndex = maxUint
}

func (mbf *meshBuilderFace) isDisabled() bool {
	return mbf.halfEdgeIndex == maxUint
}

func newMeshBuilderFace() meshBuilderFace {
	return meshBuilderFace{halfEdgeIndex: maxUint}
}
