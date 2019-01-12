package quickhull

import (
	"fmt"
	"log"
	"math"

	"github.com/golang/geo/r3"
)

type diagnostics struct {
	failedHorizonEdges int // How many times QuickHull failed to solve the horizon edge. Failures lead to degenerated convex hulls.
}

type quickHull struct {
	epsilon        float64
	epsilonSquared float64

	planar               bool
	planarPointCloudTemp []r3.Vector
	vertexData           []r3.Vector
	mesh                 meshBuilder
	extremeValueIndices  [6]int
	diagnostics          diagnostics

	newFaceIndices           []int
	newHalfEdgeIndices       []int
	disabledFacePointVectors [][]int
}

// Find indices of extreme values (max x, min x, max y, min y, max z, min z) for the given point cloud
func extremeValues(vertexData []r3.Vector) (extremeValueIndices [6]int) {
	vd0 := vertexData[0]
	extremeVals := [6]float64{vd0.X, vd0.X, vd0.Y, vd0.Y, vd0.Z, vd0.Z}

	xv := func(i, i2 int, v float64, idx int) {
		if v > extremeVals[i] {
			extremeVals[i] = v
			extremeValueIndices[i] = idx
		} else if v < extremeVals[i2] {
			extremeVals[i2] = v
			extremeValueIndices[i2] = idx
		}
	}

	for i, pos := range vertexData[1:] {
		idx := i + 1
		xv(0, 1, pos.X, idx)
		xv(2, 3, pos.Y, idx)
		xv(4, 5, pos.Z, idx)
	}

	return
}

type vectorField int

const (
	vfX vectorField = iota
	vfY
	vfZ
)

// Compute scale of the vertex data.
func scale(vertexData []r3.Vector, extremeValueIndices [6]int) (s float64) {
	scl := func(i int, valType vectorField) {
		v := vertexData[extremeValueIndices[i]]
		var a float64
		switch valType {
		case vfX:
			a = v.X
		case vfY:
			a = v.Y
		case vfZ:
			a = v.Z
		}
		s = math.Max(s, math.Abs(a))
	}

	scl(0, vfX)
	scl(1, vfX)
	scl(2, vfY)
	scl(3, vfY)
	scl(4, vfZ)
	scl(5, vfZ)

	return
}

// Create a half edge mesh representing the base tetrahedron from which the QuickHull iteration proceeds. m_extremeValues must be properly set up when this is called.
func (qh *quickHull) initialTetrahedron() meshBuilder {
	nVertices := len(qh.vertexData)

	// If we have at most 4 points, just return p1 degenerate tetrahedron:
	if nVertices <= 4 {
		v := [4]int{
			0,
			int(math.Min(1, float64(nVertices-1))),
			int(math.Min(2, float64(nVertices-1))),
			int(math.Min(3, float64(nVertices-1))),
		}

		n := triangleNormal(qh.vertexData[v[0]], qh.vertexData[v[1]], qh.vertexData[v[2]])

		trianglePlane := newPlane(n, qh.vertexData[v[0]])
		if trianglePlane.isPointOnPositiveSide(qh.vertexData[v[3]]) {
			v[0], v[1] = v[1], v[0]
		}
		return newMeshBuilder(v[0], v[1], v[2], v[3])
	}

	// Find two most distant extreme points.
	maxD := qh.epsilonSquared
	var p1, p2 int
	for i := 0; i < 6; i++ {
		for j := i + 1; j < 6; j++ {
			dv := qh.vertexData[qh.extremeValueIndices[i]].Sub(qh.vertexData[qh.extremeValueIndices[j]])
			dSq := dv.X*dv.X + dv.Y*dv.Y + dv.Z*dv.Z
			if dSq > maxD {
				maxD = dSq
				p1 = qh.extremeValueIndices[i]
				p2 = qh.extremeValueIndices[j]
			}
		}
	}

	if maxD == qh.epsilonSquared {
		// A degenerate case: the point cloud seems to consists of p1 single point
		return newMeshBuilder(0, int(math.Min(1, float64(nVertices-1))), int(math.Min(2, float64(nVertices-1))), int(math.Min(3, float64(nVertices-1))))
	}
	assertB(p1 != p2)

	// Find the most distant point to the line between the two chosen extreme points.
	r := newRay(qh.vertexData[p1], qh.vertexData[p2].Sub(qh.vertexData[p1]))
	maxD = qh.epsilonSquared
	maxI := maxInt
	for i, v := range qh.vertexData {
		distToRay := squaredDistanceBetweenPointAndRay(v, r)
		if distToRay > maxD {
			maxD = distToRay
			maxI = i
		}
	}
	if maxD == qh.epsilonSquared {
		// It appears that the point cloud belongs to a 1 dimensional subspace of R^3: convex hull has no volume => return a thin triangle
		// Pick any point other than selectedPoints.first and selectedPoints.second as the third point of the triangle
		var it r3.Vector
		var p3 int
		for i, v := range qh.vertexData {
			if v != qh.vertexData[p1] && v != qh.vertexData[p2] {
				it = v
				p3 = i
			}
		}
		if it == qh.vertexData[len(qh.vertexData)-1] {
			p3 = p1
		}

		var p4 int
		for i, v := range qh.vertexData {
			if v != qh.vertexData[p1] && v != qh.vertexData[p2] && v != qh.vertexData[p3] {
				it = v
				p4 = i
			}
		}
		if it == qh.vertexData[len(qh.vertexData)-1] {
			p4 = p1
		}
		return newMeshBuilder(p1, p2, p3, p4)
	}

	// These three points form the base triangle for our tetrahedron.
	assertB(p1 != maxI && p2 != maxI)
	baseTriangle := [3]int{p1, p2, maxI}
	baseTriangleVertices := [3]r3.Vector{qh.vertexData[baseTriangle[0]], qh.vertexData[baseTriangle[1]], qh.vertexData[baseTriangle[2]]}

	// Next step is to find the 4th vertex of the tetrahedron.
	// We naturally choose the point farthest away from the triangle plane.
	maxD = qh.epsilon
	maxI = 0
	{
		n := triangleNormal(baseTriangleVertices[0], baseTriangleVertices[1], baseTriangleVertices[2])
		{
			trianglePlane := newPlane(n, baseTriangleVertices[0])
			for i := 0; i < nVertices; i++ {
				d := math.Abs(signedDistanceToPlane(qh.vertexData[i], trianglePlane))
				if d > maxD {
					maxD = d
					maxI = i
				}
			}
		}
		if maxD == qh.epsilon {
			// All the points seem to lie on a 2D subspace of R^3. How to handle this?
			// Well, let's add one extra point to the point cloud so that the convex hull will have volume.
			qh.planar = true
			n := triangleNormal(baseTriangleVertices[1], baseTriangleVertices[2], baseTriangleVertices[0])
			qh.planarPointCloudTemp = qh.planarPointCloudTemp[:0]
			// TODO: is this append correct?
			qh.planarPointCloudTemp = append(qh.vertexData, qh.planarPointCloudTemp...)
			extraPoint := n.Add(qh.vertexData[0])
			qh.planarPointCloudTemp = append(qh.planarPointCloudTemp, extraPoint)
			maxI = len(qh.planarPointCloudTemp) - 1
			qh.vertexData = qh.planarPointCloudTemp
		}

		// Enforce CCW orientation (if user prefers clockwise orientation, swap two vertices in each triangle when final mesh is created)
		triPlane := newPlane(n, baseTriangleVertices[0])
		if triPlane.isPointOnPositiveSide(qh.vertexData[maxI]) {
			baseTriangle[0], baseTriangle[1] = baseTriangle[1], baseTriangle[0]
		}
	}

	// Create a tetrahedron half edge mesh and compute planes defined by each triangle
	mesh := newMeshBuilder(baseTriangle[0], baseTriangle[1], baseTriangle[2], maxI)
	for i := range mesh.faces {
		v := mesh.vertexIndicesOfFace(mesh.faces[i])
		va := qh.vertexData[v[0]]
		vb := qh.vertexData[v[1]]
		vc := qh.vertexData[v[2]]
		n := triangleNormal(va, vb, vc)
		mesh.faces[i].plane = newPlane(n, va)
	}

	// Finally we assign a face for each vertex outside the tetrahedron (vertices inside the tetrahedron have no role anymore)
	for i := 0; i < nVertices; i++ {
		for j := range mesh.faces {
			if qh.addPointToFace(&mesh.faces[j], i) {
				break
			}
		}
	}

	return mesh
}

// Given a list of half edges, try to rearrange them so that they form a loop. Return true on success.
func (qh quickHull) reorderHorizontalEdges(horizontalEdges []int) bool {
	nEdges := len(horizontalEdges)
	for i := 0; i < nEdges-1; i++ {
		endVertex := qh.mesh.halfEdges[horizontalEdges[i]].endVertex
		var foundNext bool
		for j := i + 1; j < nEdges; j++ {
			beginVertex := qh.mesh.halfEdges[qh.mesh.halfEdges[horizontalEdges[j]].opp].endVertex
			if beginVertex == endVertex {
				horizontalEdges[i+1], horizontalEdges[j] = horizontalEdges[j], horizontalEdges[i+1]
				foundNext = true
				break
			}
		}
		if !foundNext {
			return false
		}
	}
	return true
}

// Associates a point with a face if the point resides on the positive side of the plane. Returns true if the points was on the positive side.
func (qh *quickHull) addPointToFace(face *meshBuilderFace, pointIndex int) bool {
	d := signedDistanceToPlane(qh.vertexData[pointIndex], face.plane)
	if d > 0 && d*d > qh.epsilonSquared*face.plane.sqrNLength {
		/* TODO: optimize
		if face.pointsOnPositiveSide == nil {
			f.m_pointsOnPositiveSide = std::move(getIndexVectorFromPool());
		}
		*/
		face.pointsOnPositiveSide = append(face.pointsOnPositiveSide, pointIndex)
		if d > face.mostDistantPointDist {
			face.mostDistantPointDist = d
			face.mostDistantPoint = pointIndex
		}
		return true
	}
	return false
}

const (
	minUint = uint(0)
	maxUint = ^minUint
	//minInt  = -maxInt - 1
	maxInt = int(maxUint >> 1)
)

// This will update m_mesh from which we create the ConvexHull object that getConvexHull function returns
func (qh *quickHull) createConvexHalfEdgeMesh() {
	var visibleFaces []int
	var horizontalEdges []int

	type faceData struct {
		faceIndex           int
		enteredFromHalfEdge int // If the face turns out not to be visible, this half edge will be marked as horizon edge
	}

	var possiblyVisibleFaces []faceData

	// Compute base tetrahedron
	qh.mesh = qh.initialTetrahedron()
	assertB(len(qh.mesh.faces) == 4)

	var faceList []int
	for i := 0; i < 4; i++ {
		f := &qh.mesh.faces[i]
		if len(f.pointsOnPositiveSide) > 0 {
			faceList = append(faceList, i)
			f.inFaceStack = true
		}
	}

	// Process faces until the face list is empty.
	iter := 0
	for len(faceList) > 0 {
		iter++
		if iter == maxInt {
			// Visible face traversal marks visited faces with iteration counter (to mark that the face has been visited on this iteration) and the max value represents unvisited faces. At this point we have to reset iteration counter. This shouldn't be an
			// issue on 64 bit machines.
			iter = 0
		}

		var topFaceIndex int
		topFaceIndex, faceList = faceList[0], faceList[1:]

		tf := &qh.mesh.faces[topFaceIndex]
		tf.inFaceStack = false

		assertB(tf.pointsOnPositiveSide == nil || len(tf.pointsOnPositiveSide) > 0)
		if tf.pointsOnPositiveSide == nil || tf.isDisabled() {
			continue
		}

		// Pick the most distant point to this triangle plane as the point to which we extrude
		activePoint := qh.vertexData[tf.mostDistantPoint]
		activePointIndex := tf.mostDistantPoint

		// Clear outer vars
		horizontalEdges = horizontalEdges[:0]
		visibleFaces = visibleFaces[:0]
		possiblyVisibleFaces = possiblyVisibleFaces[:0]

		// Find out the faces that have our active point on their positive side (these are the "visible faces").
		// The face on top of the stack of course is one of them. At the same time, we create a list of horizon edges.
		possiblyVisibleFaces = append(possiblyVisibleFaces, faceData{faceIndex: topFaceIndex, enteredFromHalfEdge: maxInt})
		for len(possiblyVisibleFaces) > 0 {
			fd := possiblyVisibleFaces[len(possiblyVisibleFaces)-1]
			possiblyVisibleFaces = possiblyVisibleFaces[:len(possiblyVisibleFaces)-1]
			pvf := &qh.mesh.faces[fd.faceIndex]
			assertB(!pvf.isDisabled())

			if pvf.visibilityCheckedOnIteration == iter {
				if pvf.isVisibleFaceOnCurrentIteration {
					continue
				}
			} else {
				p := pvf.plane
				pvf.visibilityCheckedOnIteration = iter
				d := p.n.Dot(activePoint) + p.d
				if d > 0 {
					pvf.isVisibleFaceOnCurrentIteration = true
					pvf.horizonEdgesOnCurrentIteration = 0
					visibleFaces = append(visibleFaces, fd.faceIndex)

					for _, heIndex := range qh.mesh.halfEdgeIndicesOfFace(*pvf) {
						opp := qh.mesh.halfEdges[heIndex].opp
						if opp != fd.enteredFromHalfEdge {
							possiblyVisibleFaces = append(possiblyVisibleFaces, faceData{faceIndex: qh.mesh.halfEdges[opp].face, enteredFromHalfEdge: heIndex})
						}
					}
					continue
				}
				assertB(fd.faceIndex != topFaceIndex)
			}

			// The face is not visible. Therefore, the halfedge we came from is part of the horizon edge.
			pvf.isVisibleFaceOnCurrentIteration = false
			horizontalEdges = append(horizontalEdges, fd.enteredFromHalfEdge)

			// Store which half edge is the horizon edge. The other half edges of the face will not be part of the final mesh so their data slots can by recycled.
			halfEdges := qh.mesh.halfEdgeIndicesOfFace(qh.mesh.faces[qh.mesh.halfEdges[fd.enteredFromHalfEdge].face])
			var ind byte
			if halfEdges[0] != fd.enteredFromHalfEdge {
				if halfEdges[1] == fd.enteredFromHalfEdge {
					ind = 1
				} else {
					ind = 2
				}
			}
			qh.mesh.faces[qh.mesh.halfEdges[fd.enteredFromHalfEdge].face].horizonEdgesOnCurrentIteration |= 1 << ind
		}

		nHorizontalEdges := len(horizontalEdges)

		// Order horizon edges so that they form a loop. This may fail due to numerical instability in which case we give up trying to solve horizon edge for this point and accept a minor degeneration in the convex hull.
		if !qh.reorderHorizontalEdges(horizontalEdges) {
			qh.diagnostics.failedHorizonEdges++
			log.Println("Failed to solve horizon edge")

			for i := range tf.pointsOnPositiveSide {
				if tf.pointsOnPositiveSide[i] == activePointIndex {
					tf.pointsOnPositiveSide = append(tf.pointsOnPositiveSide[:i], tf.pointsOnPositiveSide[i+1:]...)
					break
				}
			}

			/*
				TODO: optimize
				if len(tf.pointsOnPositiveSide) == 0 {
					reclaimToIndexVectorPool(tf.m_pointsOnPositiveSide);
				}
			*/
			continue
		}

		// Except for the horizon edges, all half edges of the visible faces can be marked as disabled. Their data slots will be reused.
		// The faces will be disabled as well, but we need to remember the points that were on the positive side of them - therefore
		// we save pointers to them.
		qh.newFaceIndices = qh.newFaceIndices[:0]
		qh.newHalfEdgeIndices = qh.newHalfEdgeIndices[:0]
		qh.disabledFacePointVectors = qh.disabledFacePointVectors[:0]

		var nDisabled int
		for _, faceIdx := range visibleFaces {
			disabledFace := qh.mesh.faces[faceIdx]
			halfEdges := qh.mesh.halfEdgeIndicesOfFace(disabledFace)
			for i := uint(0); i < 3; i++ {
				if disabledFace.horizonEdgesOnCurrentIteration&(1<<i) == 0 {
					if nDisabled < nHorizontalEdges*2 {
						// Use on this iteration
						qh.newHalfEdgeIndices = append(qh.newHalfEdgeIndices, halfEdges[i])
						nDisabled++
					} else {
						// Mark for reusal on later iteration step
						qh.mesh.disableHalfEdge(halfEdges[i])
					}
				}
			}
			// Disable the face, but retain pointer to the points that were on the positive side of it. We need to assign those points
			// to the new faces we create shortly.
			t := qh.mesh.disableFace(faceIdx)
			if t != nil {
				assertB(len(t) > 0)
				// TODO: is this equivalent to std::move???
				qh.disabledFacePointVectors = append(qh.disabledFacePointVectors, t)
			}
		}
		if nDisabled < nHorizontalEdges*2 {
			nNewHalfEdgesNeeded := nHorizontalEdges*2 - nDisabled
			for i := 0; i < nNewHalfEdgesNeeded; i++ {
				qh.newHalfEdgeIndices = append(qh.newHalfEdgeIndices, qh.mesh.addHalfEdge())
			}
		}
		// Create new faces using the edgeloop
		for i := 0; i < nHorizontalEdges; i++ {
			ab := horizontalEdges[i]

			horizonEdgeVertexIndices := qh.mesh.vertexIndicesOfHalfEdge(qh.mesh.halfEdges[ab])
			a, b, c := horizonEdgeVertexIndices[0], horizonEdgeVertexIndices[1], activePointIndex

			newFaceIdx := qh.mesh.addFace()
			qh.newFaceIndices = append(qh.newFaceIndices, newFaceIdx)

			ca, bc := qh.newHalfEdgeIndices[2*i+0], qh.newHalfEdgeIndices[2*i+1]

			qh.mesh.halfEdges[ab].next = bc
			qh.mesh.halfEdges[bc].next = ca
			qh.mesh.halfEdges[ca].next = ab

			qh.mesh.halfEdges[bc].face = newFaceIdx
			qh.mesh.halfEdges[ca].face = newFaceIdx
			qh.mesh.halfEdges[ab].face = newFaceIdx

			qh.mesh.halfEdges[ca].endVertex = a
			qh.mesh.halfEdges[bc].endVertex = c

			newFace := &qh.mesh.faces[newFaceIdx]

			planeNormal := triangleNormal(qh.vertexData[a], qh.vertexData[b], activePoint)
			newFace.plane = newPlane(planeNormal, activePoint)
			newFace.halfEdgeIndex = ab

			var idx int
			if i > 0 {
				idx = i*2 - 1
			} else {
				idx = 2*nHorizontalEdges - 1
			}
			qh.mesh.halfEdges[ca].opp = qh.newHalfEdgeIndices[idx]
			qh.mesh.halfEdges[bc].opp = qh.newHalfEdgeIndices[((i+1)*2)%(nHorizontalEdges*2)]
		}

		for _, disabledPoints := range qh.disabledFacePointVectors {
			assertB(disabledPoints != nil)
			for _, pointIdx := range disabledPoints {
				if pointIdx == activePointIndex {
					continue
				}
				for i := 0; i < nHorizontalEdges; i++ {
					if qh.addPointToFace(&qh.mesh.faces[qh.newFaceIndices[i]], pointIdx) {
						break
					}
				}
			}
			/* TODO: optimize
			// The points are no longer needed: we can move them to the vector pool for reuse.
			reclaimToIndexVectorPool(disabledPoints);
			*/
		}
		// Increase face stack size if needed
		for _, newFaceIdx := range qh.newFaceIndices {
			newFace := &qh.mesh.faces[newFaceIdx]
			if newFace.pointsOnPositiveSide != nil {
				assertB(len(newFace.pointsOnPositiveSide) > 0)
				if !newFace.inFaceStack {
					faceList = append(faceList, newFaceIdx)
					newFace.inFaceStack = true
				}
			}
		}
	}

	fmt.Println()

	/* TODO: optimize
	// Cleanup
	m_indexVectorPool.clear();
	*/
}

func (qh *quickHull) buildMesh(pointCloud []r3.Vector, epsilon float64) {
	if len(pointCloud) == 0 {
		// TODO: is ths correct?
		return
	}
	qh.vertexData = pointCloud

	// Very first: find extreme values and use them to compute the scale of the point cloud.
	qh.extremeValueIndices = extremeValues(qh.vertexData)
	scale := scale(qh.vertexData, qh.extremeValueIndices) // TODO: maybe pass extreme values

	// Epsilon we use depends on the scale
	qh.epsilon = epsilon * scale
	qh.epsilonSquared = qh.epsilon * qh.epsilon

	// Reset diagnostics
	qh.diagnostics = diagnostics{}

	qh.planar = false // The planar case happens when all the points appear to lie on a two dimensional subspace of R^3.
	qh.createConvexHalfEdgeMesh()

	if qh.planar {
		extraPointIdx := len(qh.planarPointCloudTemp) - 1
		for i := range qh.mesh.halfEdges {
			if qh.mesh.halfEdges[i].endVertex == extraPointIdx {
				qh.mesh.halfEdges[i].endVertex = 0
			}
		}
		qh.vertexData = pointCloud
		qh.planarPointCloudTemp = qh.planarPointCloudTemp[:0]
	}
}

func (qh *quickHull) convexHull(pointCloud []r3.Vector, ccw bool, useOriginalIndices bool, epsilon float64) convexHull {
	qh.buildMesh(pointCloud, epsilon)
	return newConvexHull(qh.mesh, qh.vertexData, ccw, useOriginalIndices)
}

const (
	epsilonF = 0.0001
	//epsilonD = 0.0000001
)

// ConvexHull calculates the convex hull of the given point cloud using the Quickhull algorithm.
// See: https://en.wikipedia.org/wiki/Quickhull
func ConvexHull(pointCloud []r3.Vector) []r3.Vector {
	qh := new(quickHull)
	hull := qh.convexHull(pointCloud, true, false, epsilonF)
	return hull.vertices
}

// ConvexHull calculates the convex hull and returns it as a half-edge-mesh.
// See also: ConvexHull()
func ConvexHullAsMesh(pointCloud []r3.Vector) HalfEdgeMesh {
	qh := new(quickHull)
	qh.buildMesh(pointCloud, epsilonF)
	return newHalfEdgeMesh(qh.mesh, qh.vertexData)
}
