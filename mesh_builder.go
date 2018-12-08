package quickhull

type meshBuilder struct {
	// Mesh data
	faces     []meshBuilderFace
	halfEdges []halfEdge

	// When the mesh is modified and faces and half edges are removed from it, we do not actually remove them from the container vectors.
	// Insted, they are marked as disabled which means that the indices can be reused when we need to add new faces and half edges to the mesh.
	// We store the free indices in the following vectors.
	disabledFaces     []int
	disabledHalfEdges []int
}

func (mb *meshBuilder) addFace() int {
	if nDisabled := len(mb.disabledFaces); nDisabled > 0 {
		lastDisabledIndex := nDisabled - 1
		index := mb.disabledFaces[lastDisabledIndex]
		f := mb.faces[index]
		assertB(f.isDisabled())
		assertB(f.pointsOnPositiveSide == nil)
		f.mostDistantPointDist = 0
		mb.disabledFaces = mb.disabledFaces[:lastDisabledIndex]

		return index
	}

	mb.faces = append(mb.faces, meshBuilderFace{})
	return len(mb.faces) - 1
}

func (mb *meshBuilder) addHalfEdge() int {
	if nDisabled := len(mb.disabledHalfEdges); nDisabled > 0 {
		lastDisabledIndex := nDisabled - 1
		index := mb.disabledHalfEdges[lastDisabledIndex]
		mb.disabledHalfEdges = mb.disabledHalfEdges[:lastDisabledIndex]

		return index
	}

	mb.halfEdges = append(mb.halfEdges, halfEdge{})
	return len(mb.halfEdges) - 1
}

func (mb *meshBuilder) disableFace(faceIndex int) []int {
	f := mb.faces[faceIndex]
	f.disable()
	mb.disabledFaces = append(mb.disabledFaces, faceIndex)
	return f.pointsOnPositiveSide
}

func (mb *meshBuilder) disableHalfEdge(heIndex int) {
	he := mb.halfEdges[heIndex]
	he.disable()
	mb.disabledHalfEdges = append(mb.disabledHalfEdges, heIndex)
}

func (mb *meshBuilder) vertexIndicesOfFace(f meshBuilderFace) [3]int {
	var v [3]int
	he := mb.halfEdges[f.halfEdgeIndex]
	v[0] = he.endVertex
	he = mb.halfEdges[he.next]
	v[1] = he.endVertex
	he = mb.halfEdges[he.next]
	v[2] = he.endVertex
	return v
}

func (mb *meshBuilder) vertexIndicesOfHalfEdge(he halfEdge) [2]int {
	return [2]int{mb.halfEdges[he.opp].endVertex, he.endVertex}
}
func (mb *meshBuilder) halfEdgeIndicesOfFace(f meshBuilderFace) [3]int {
	second := mb.halfEdges[f.halfEdgeIndex].next
	return [3]int{f.halfEdgeIndex, second, mb.halfEdges[second].next}
}

// Create a mesh with initial tetrahedron ABCD. Dot product of AB with the normal of triangle ABC should be negative.
func newMeshBuilder(a, b, c, d int) meshBuilder {
	// Create halfedges
	var mb meshBuilder

	ab := halfEdge{
		endVertex: b,
		opp:       6,
		face:      0,
		next:      1,
	}
	mb.halfEdges = append(mb.halfEdges, ab)

	bc := halfEdge{
		endVertex: c,
		opp:       9,
		face:      0,
		next:      2,
	}
	mb.halfEdges = append(mb.halfEdges, bc)

	ca := halfEdge{
		endVertex: a,
		opp:       3,
		face:      0,
		next:      0,
	}
	mb.halfEdges = append(mb.halfEdges, ca)

	ac := halfEdge{
		endVertex: c,
		opp:       2,
		face:      1,
		next:      4,
	}
	mb.halfEdges = append(mb.halfEdges, ac)

	cd := halfEdge{
		endVertex: d,
		opp:       11,
		face:      1,
		next:      5,
	}
	mb.halfEdges = append(mb.halfEdges, cd)

	da := halfEdge{
		endVertex: a,
		opp:       7,
		face:      1,
		next:      3,
	}
	mb.halfEdges = append(mb.halfEdges, da)

	ba := halfEdge{
		endVertex: a,
		opp:       0,
		face:      2,
		next:      7,
	}
	mb.halfEdges = append(mb.halfEdges, ba)

	ad := halfEdge{
		endVertex: d,
		opp:       5,
		face:      2,
		next:      8,
	}
	mb.halfEdges = append(mb.halfEdges, ad)

	db := halfEdge{
		endVertex: b,
		opp:       10,
		face:      2,
		next:      6,
	}
	mb.halfEdges = append(mb.halfEdges, db)

	cb := halfEdge{
		endVertex: b,
		opp:       1,
		face:      3,
		next:      10,
	}
	mb.halfEdges = append(mb.halfEdges, cb)

	bd := halfEdge{
		endVertex: d,
		opp:       8,
		face:      3,
		next:      11,
	}
	mb.halfEdges = append(mb.halfEdges, bd)

	dc := halfEdge{
		endVertex: c,
		opp:       4,
		face:      3,
		next:      9,
	}
	mb.halfEdges = append(mb.halfEdges, dc)

	return mb
}

func assertB(b bool) {
	// TODO: delete assertion?
	if !b {
		panic("Assertion failed")
	}
}

type meshBuilderFace struct {
	halfEdgeIndex                   int
	plane                           plane
	mostDistantPoint                int
	mostDistantPointDist            float64
	visibilityCheckedOnIteration    int
	isVisibleFaceOnCurrentIteration bool
	inFaceStack                     bool
	horizonEdgesOnCurrentIteration  byte // Bit for each half edge assigned to this face, each being 0 or 1 depending on whether the edge belongs to horizon edge
	pointsOnPositiveSide            []int
}

func (mbf *meshBuilderFace) disable() {
	mbf.halfEdgeIndex = disabledInt
}

func (mbf *meshBuilderFace) isDisabled() bool {
	return mbf.halfEdgeIndex == disabledInt
}

func newMeshBuilderFace() meshBuilderFace {
	return meshBuilderFace{halfEdgeIndex: disabledInt}
}
