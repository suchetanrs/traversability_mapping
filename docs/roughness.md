# Roughness Calculation

## Overview

Roughness measures how much points in a vicinity window scatter above and below the best-fit plane. It is computed from the minimum eigenvalue of the fused point cloud covariance matrix.

## Steps

### 1. Fuse the vicinity moments

All occupied cells in a window around the query cell have their moments shifted to a common origin (the query cell centre) and summed:

$$N_{total},\ \mathbf{S} = \sum_i \mathbf{s}_i,\ \mathbf{Q} = \sum_i \mathbf{Q}_i$$

### 2. Compute the covariance matrix

$$\boldsymbol{\mu} = \frac{\mathbf{S}}{N}, \qquad \boldsymbol{\Sigma} = \frac{\mathbf{Q}}{N} - \boldsymbol{\mu}\boldsymbol{\mu}^T$$

This is the 3×3 covariance of all points in the vicinity, computed without storing any individual points.

### 3. Eigendecompose

$$\boldsymbol{\Sigma} = \mathbf{V} \boldsymbol{\Lambda} \mathbf{V}^T, \qquad \lambda_0 \leq \lambda_1 \leq \lambda_2$$

Eigenvalues are sorted ascending. The smallest eigenvalue λ₀ corresponds to the eigenvector perpendicular to the best-fit plane (the surface normal).

### 4. Interpret λ₀

λ₀ is the mean squared point-to-plane distance:

$$\lambda_0 = \frac{1}{N} \sum_i \left( (\mathbf{p}_i - \boldsymbol{\mu}) \cdot \hat{\mathbf{n}} \right)^2$$

- Flat surface → all points lie on the plane → λ₀ = 0
- Rough surface → points scatter above and below the plane → λ₀ > 0

### 5. RMS point-to-plane distance

$$\text{roughness} = \sqrt{\lambda_0}$$

This is the standard deviation of point heights relative to the fitted plane, i.e. the RMS scatter perpendicular to the surface.

### 6. Normalise to a hazard score

$$\text{HAZ\_ROUGHNESS} = \min\!\left(\frac{\text{roughness}}{\text{ground\_clearance}},\ 1.0\right)$$

`ground_clearance` is the robot's minimum clearance (default 0.15 m). The hazard reaches 1.0 when the RMS scatter equals the clearance — the terrain is considered impassable.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `ground_clearance` | 0.15 m | RMS scatter at which roughness hazard = 1.0 |
| `min_vicinity_points` | 15 | Minimum total points to attempt a plane fit |
| `min_occupied_fraction` | 0.5 | Minimum fraction of vicinity cells that must be observed |
