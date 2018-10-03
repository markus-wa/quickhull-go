package quickhull

import (
	"testing"

	r3 "github.com/golang/geo/r3"
	assert "github.com/stretchr/testify/assert"
)

// Simple 2D test (all points on a plane)
func TestConvexHull2D(t *testing.T) {
	// Construct a point cloud that looks roughly like this.
	// All points are on a single plane (Z=0).
	// D should be inside the Convex Hull.
	//
	//         C
	//       /   \
	//     /   D   \
	//   /           \
	// A - - - - - - - B
	//
	pointCloud := []r3.Vector{
		r3.Vector{X: 1, Y: 2, Z: 0},
		r3.Vector{X: 4, Y: 7, Z: 0},
		r3.Vector{X: 7, Y: 2, Z: 0},
		r3.Vector{X: 4, Y: 4, Z: 0}, // This point is inside the hull
	}

	expectedHull := []r3.Vector{
		r3.Vector{X: 1, Y: 2, Z: 0},
		r3.Vector{X: 4, Y: 7, Z: 0},
		r3.Vector{X: 7, Y: 2, Z: 0},
	}

	assert.ElementsMatch(t, expectedHull, ConvexHull(pointCloud), "ConvexHull should be as expected")
}
