package quickhull

import (
	"testing"

	"github.com/golang/geo/r3"
	"github.com/stretchr/testify/assert"
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
		{X: 1, Y: 2, Z: 1},
		{X: 4, Y: 7, Z: 1},
		{X: 7, Y: 2, Z: 1},
		{X: 4, Y: 4, Z: 1}, // This point is inside the hull
	}

	expectedHull := []r3.Vector{
		{X: 1, Y: 2, Z: 0},
		{X: 4, Y: 7, Z: 0},
		{X: 7, Y: 2, Z: 0},
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

	expectedHull := []r3.Vector{
		{X: 0, Y: 0, Z: 0},
		{X: 0, Y: 0, Z: 10},
		{X: 0, Y: 10, Z: 0},
		{X: 0, Y: 10, Z: 10},
		{X: 10, Y: 0, Z: 0},
		{X: 10, Y: 0, Z: 10},
		{X: 10, Y: 10, Z: 0},
		{X: 10, Y: 10, Z: 10},
	}

	assert.ElementsMatch(t, expectedHull, ConvexHull(pointCloud), "ConvexHull should be as expected")
}

func TestPlanes(t *testing.T) {
	m := r3.Vector{X: 1, Y: 0, Z: 0}
	n := r3.Vector{X: 2, Y: 0, Z: 0}
	p := newPlane(m, n)

	var dist float64
	dist = signedDistanceToPlane(r3.Vector{X: 3, Y: 0, Z: 0}, p)
	assert.Equal(t, 1.0, dist)

	dist = signedDistanceToPlane(r3.Vector{X: 1, Y: 0, Z: 0}, p)
	assert.Equal(t, -1.0, dist)

	m = r3.Vector{X: 2, Y: 0, Z: 0}
	p = newPlane(m, n)

	dist = signedDistanceToPlane(r3.Vector{X: 6, Y: 0, Z: 0}, p)

	assert.Equal(t, 8.0, dist)
}
