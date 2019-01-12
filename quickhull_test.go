package quickhull

import (
	"math/rand"
	"testing"

	"github.com/golang/geo/r3"
	"github.com/stretchr/testify/assert"
)

// Simple 2D test (all points on a plane)
func TestConvexHull2D(t *testing.T) {
	// Construct a square as 'point cloud' that looks roughly like this.
	// All points are on a single plane (Z=0).
	// E should be inside the Convex Hull.
	//
	// C - - - - - - D
	// |             |
	// |             |
	// |      E      |
	// |             |
	// |             |
	// A - - - - - - B
	//
	pointCloud := []r3.Vector{
		{X: 0, Y: 0, Z: 1},
		{X: 0, Y: 10, Z: 1},
		{X: 10, Y: 0, Z: 1},
		{X: 10, Y: 10, Z: 1},
		{X: 5, Y: 5, Z: 1}, // This point is inside the hull
	}

	expectedHull := []r3.Vector{
		{X: 0, Y: 0, Z: 1},
		{X: 0, Y: 10, Z: 1},
		{X: 10, Y: 0, Z: 1},
		{X: 10, Y: 10, Z: 1},
	}

	assert.ElementsMatch(t, expectedHull, ConvexHull(pointCloud), "ConvexHull should be as expected")
}

// Simple 3D test (one point in a box)
func TestConvexHull3D(t *testing.T) {
	// Construct a point cloud that looks roughly like this.
	// A should be inside the Convex Hull.
	/*
		@ + + + + + + + + + + + @
		+\                      +\
		+ \                     + \
		+  \                    +  \
		+   \                   +   \
		+    @ + + + + + + + + +++ + @
		+    +                  +    +
		+    +                  +    +
		+    +                  +    +
		+    +         A        +    +
		+    +                  +    +
		+    +                  +    +
		@ + +++ + + + + + + + + @    +
		 \   +                   \   +
		  \  +                    \  +
		   \ +                     \ +
		    \+                      \+
		     @ + + + + + + + + + + + @
	*/
	pointCloud := []r3.Vector{
		{X: 0, Y: 0, Z: 0},
		{X: 0, Y: 0, Z: 10},
		{X: 0, Y: 10, Z: 0},
		{X: 0, Y: 10, Z: 10},
		{X: 10, Y: 0, Z: 0},
		{X: 10, Y: 0, Z: 10},
		{X: 10, Y: 10, Z: 0},
		{X: 10, Y: 10, Z: 10},
		{X: 5, Y: 5, Z: 5},
	}

	actual := ConvexHull(pointCloud)

	expected := []r3.Vector{
		{X: 0, Y: 0, Z: 0},
		{X: 0, Y: 0, Z: 10},
		{X: 0, Y: 10, Z: 0},
		{X: 0, Y: 10, Z: 10},
		{X: 10, Y: 0, Z: 0},
		{X: 10, Y: 0, Z: 10},
		{X: 10, Y: 10, Z: 0},
		{X: 10, Y: 10, Z: 10},
	}

	assert.Equal(t, 8, len(actual))
	assert.ElementsMatch(t, expected, actual, "ConvexHull should be as expected")
}

func TestHalfEdgeOutput(t *testing.T) {
	var pointCloud []r3.Vector

	for i := 0; i < 1000; i++ {
		pointCloud = append(pointCloud, r3.Vector{
			X: randF64(-1, 1),
			Y: randF64(-1, 1),
			Z: randF64(-1, 1),
		})
	}

	twoOrNegativeTwo := func(i, and int) float64 {
		if i&and > 0 {
			return -2
		}
		return 2
	}

	for i := 0; i < 8; i++ {
		pointCloud = append(pointCloud, r3.Vector{
			X: twoOrNegativeTwo(i, 1),
			Y: twoOrNegativeTwo(i, 2),
			Z: twoOrNegativeTwo(i, 4),
		})
	}

	mesh := ConvexHullAsMesh(pointCloud)

	assert.Equal(t, 12, len(mesh.faces))
	assert.Equal(t, 36, len(mesh.halfEdges))
	assert.Equal(t, 8, len(mesh.vertices))
}

func randF64(min, max float64) float64 {
	return (max-min)*rand.Float64() + min
}

func TestPlanes(t *testing.T) {
	m := r3.Vector{X: 1, Y: 0, Z: 0}
	n := r3.Vector{X: 2, Y: 0, Z: 0}
	p := newPlane(m, n)

	dist := signedDistanceToPlane(r3.Vector{X: 3, Y: 0, Z: 0}, p)
	assert.Equal(t, 1.0, dist)

	dist = signedDistanceToPlane(r3.Vector{X: 1, Y: 0, Z: 0}, p)
	assert.Equal(t, -1.0, dist)

	m = r3.Vector{X: 2, Y: 0, Z: 0}
	p = newPlane(m, n)

	dist = signedDistanceToPlane(r3.Vector{X: 6, Y: 0, Z: 0}, p)

	assert.Equal(t, 8.0, dist)
}
