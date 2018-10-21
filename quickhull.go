package quickhull

import (
	"log"

	r3 "github.com/golang/geo/r3"
)

type diagnostics struct {
	failedHorizonEdges int // How many times QuickHull failed to solve the horizon edge. Failures lead to degenerated convex hulls.
}

type quickHull struct {
	epsilon        float64
	epsilonSquared float64
	scale          float64

	planar               bool
	planarPointCloudTemp []r3.Vector
	vertexData           []r3.Vector
	mesh                 meshBuilder
	extremeValues        [6]int
	diagnostics          diagnostics

	newFaceIndices           []int
	newHalfEdgeIndices       []int
	disabledFacePointVectors [][]int
}

// Find indices of extreme values (max x, min x, max y, min y, max z, min z) for the given point cloud
func (qh *quickHull) calculateExtremeValues() {
	// TODO: quickHull.calculateExtremeValues()
	qh.extremeValues = [6]int{0, 0, 0, 0, 0, 0}
}

// Compute scale of the vertex data.§
func (qh *quickHull) calculateScale() {
	// TODO: quickHull.scale()
	qh.scale = 0.0
}

// Create a half edge mesh representing the base tetrahedron from which the QuickHull iteration proceeds. m_extremeValues must be properly set up when this is called.
func (qh quickHull) initialTetrahedron() meshBuilder {
	// TODO: quickHull.initialTetrahedron()
	return meshBuilder{}
}

// Given a list of half edges, try to rearrange them so that they form a loop. Return true on success.
func (qh quickHull) reorderHorizontalEdges() bool {
	// TODO: quickHull.reorderHorizontalEdges()
	return false
}

// Associates a point with a face if the point resides on the positive side of the plane. Returns true if the points was on the positive side.
func (qh *quickHull) addPointToFace(face meshBuilderFace, pointIndex int) {
	// TODO: quickHull.addPointToFace()
}

const maxUint = ^uint(0)
const minUint = 0
const maxInt = int(maxUint >> 1)
const minInt = -maxInt - 1

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
		f := qh.mesh.faces[i]
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

		topFaceIndex, faceList := faceList[0], faceList[1:]

		tf := qh.mesh.faces[topFaceIndex]
		tf.inFaceStack = false

		assertB(tf.pointsOnPositiveSide == nil || len(tf.pointsOnPositiveSide) > 0)
		if tf.pointsOnPositiveSide == nil || tf.isDisabled() {
			continue
		}

		// Pick the most distant point to this triangle plane as the point to which we extrude
		activePoint := qh.vertexData[tf.mostDistantPoint]
		activePointIndex := tf.mostDistantPoint

		// Find out the faces that have our active point on their positive side (these are the "visible faces"). The face on top of the stack of course is one of them. At the same time, we create a list of horizon edges.
		horizontalEdges = make([]int)
		possiblyVisibleFaces = make([]int)
		visibleFaces = make([]int)
		possiblyVisibleFaces = append(possiblyVisibleFaces, faceData{faceIndex: topFaceIndex, enteredFromHalfEdge: maxInt})
		for len(possiblyVisibleFaces) > 0 {
			faceData := possiblyVisibleFaces[len(possiblyVisibleFaces)-1]
			possiblyVisibleFaces := possiblyVisibleFaces[:len(possiblyVisibleFaces)-1]
			pvf := qh.mesh.faces[faceData.faceIndex]
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
					visibleFaces = append(visibleFaces, faceData.faceIndex)

					for _, heIndex := range qh.mesh.halfEdgeIndicesOfFace(pvf) {
						opp := qh.mesh.halfEdges[heIndex].opp
						if opp != faceData.enteredFromHalfEdge {
							possiblyVisibleFaces = append(possiblyVisibleFaces, faceData{faceIndex: qh.mesh.halfEdges[opp].face, enteredFromHalfEdge: heIndex})
						}
					}
					continue
				}
				assertB(faceData.faceIndex != topFaceIndex)
			}

			// The face is not visible. Therefore, the halfedge we came from is part of the horizon edge.
			pvf.isVisibleFaceOnCurrentIteration = false
			horizontalEdges = append(horizontalEdges, faceData.enteredFromHalfEdge)

			// Store which half edge is the horizon edge. The other half edges of the face will not be part of the final mesh so their data slots can by recycled.
			halfEdges := qh.mesh.halfEdgeIndicesOfFace(qh.mesh.faces[qh.mesh.halfEdges[faceData.enteredFromHalfEdge].face])
			var ind byte
			if halfEdges[0] != faceData.enteredFromHalfEdge {
				if halfEdges[1] == faceData.enteredFromHalfEdge {
					ind = 1
				} else {
					ind = 2
				}
			}
			qh.mesh.faces[qh.mesh.halfEdges[faceData.enteredFromHalfEdge].face].horizonEdgesOnCurrentIteration |= (1 << ind)
		}

		horizontalEdgeCount := len(horizontalEdges)

		// Order horizon edges so that they form a loop. This may fail due to numerical instability in which case we give up trying to solve horizon edge for this point and accept a minor degeneration in the convex hull.
		if !qh.reorderHorizontalEdges(horizontalEdges) {
			qh.diagnostics.failedHorizonEdges++
			log.Println("Failed to solve horizon edge")

			for i := range tf.pointsOnPositiveSide {
				if tf.pointsOnPositiveSide[i] == activePointIndex {
					tf.pointsOnPositiveSide = append(tf.pointsOnPositiveSide[:i], tf.pointsOnPositiveSide[i+1:])
					break
				}
			}

			if len(tf.pointsOnPositiveSide) == 0 {
				// TODO: Optimize
				//reclaimToIndexVectorPool(tf.m_pointsOnPositiveSide);
			}
			continue
		}

		// Except for the horizon edges, all half edges of the visible faces can be marked as disabled. Their data slots will be reused.
		// The faces will be disabled as well, but we need to remember the points that were on the positive side of them - therefore
		// we save pointers to them.
	}

	// TODO: quickHull.createConvexHalfEdgeMesh()

}

func (qh *quickHull) buildMesh(pointCloud []r3.Vector, ccw bool, useOriginalIndices bool, epsilon float64) {
	// TODO: quickHull.buildMesh()
}

func (qh *quickHull) convexHull(pointCloud []r3.Vector, ccw bool, useOriginalIndices bool, epsilon float64) convexHull {
	// TODO: quickHull.convexHull()
	return convexHull{}
}

// ConvexHull calculates the convex hull of the given point cloud using the Quickhull algorithm.
// See: https://en.wikipedia.org/wiki/Quickhull
func ConvexHull(pointCloud []r3.Vector) []r3.Vector {
	return []r3.Vector{}
}
