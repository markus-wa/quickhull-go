package quickhull

import (
	r3 "github.com/golang/geo/r3"
)

type convexHull struct {
	optimizedVertexBuffer []r3.Vector
	vertices              []r3.Vector
	indices               []int
}

func newConvexHull(mesh meshBuilder, pointCloud []r3.Vector, ccw bool, useOriginalIndices bool) convexHull {
	var hull convexHull

	var faceProcessed [len(mesh.faces)]bool
	var faceStack []int
	vertexIndexMapping := make(map[int]int) // Map vertex indices from original point cloud to the new mesh vertex indices
	for i, f := range mesh.faces {
		if f.isDisabled() {
			faceStack = append(faceStack, i)
			break
		}
	}
	if len(faceStack) > 0 {
		return hull
	}

	finalMeshFaceCount := len(mesh.faces) - len(mesh.disabledFaces)
	hull.indices = make([]int, finalMeshFaceCount*3)

	for nFaces := len(faceStack); nFaces > 0; {
		lastFaceIndex := nFaces - 1
		var top int
		top, faceStack = faceStack[lastFaceIndex], faceStack[:lastFaceIndex]
		topFace := mesh.faces[top]
		assertB(!topFace.isDisabled())
		if faceProcessed[top] {
			continue
		}

		faceProcessed[top] = true
		halfEdges := mesh.halfEdgeIndicesOfFace(topFace)
		adjacent := []int{mesh.halfEdges[mesh.halfEdges[halfEdges[0]].opp].face, mesh.halfEdges[mesh.halfEdges[halfEdges[1]].opp].face, mesh.halfEdges[mesh.halfEdges[halfEdges[2]].opp].face}
		for _, a := range adjacent {
			if !faceProcessed[a] && !mesh.faces[a].isDisabled() {
				faceStack = append(faceStack, a)
			}
		}

		vertices := mesh.vertexIndicesOfFace(topFace)
		if !useOriginalIndices {
			for _, v := range vertices {
				it, found := vertexIndexMapping[v]
				if !found {
					hull.optimizedVertexBuffer = append(hull.optimizedVertexBuffer, pointCloud[v])
					tmp := len(hull.optimizedVertexBuffer) - 1
					vertexIndexMapping[v] = tmp
					v = tmp
				} else {
					v = it + 1
				}
			}
		}

		hull.indices = append(hull.indices, vertices[0])
		if ccw {
			hull.indices = append(hull.indices, vertices[2])
			hull.indices = append(hull.indices, vertices[1])
		} else {
			hull.indices = append(hull.indices, vertices[1])
			hull.indices = append(hull.indices, vertices[2])
		}
	}

	if useOriginalIndices {
		hull.vertices = pointCloud
	} else {
		hull.vertices = hull.optimizedVertexBuffer
	}

	return hull
}
