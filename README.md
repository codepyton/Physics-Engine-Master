Physics Engine Master: A Computational Realization of Discontinuous Hamiltonian Dynamics on a Bounded Euclidean Manifold



1\. Abstract



This compendium elucidates the algorithmic instantiation of a stochastic ensemble of $N$ discrete, mass-endowed entities evolving within a constrained Riemannian metric space ($\\mathcal{M} \\cong \\mathbb{R}^3$). The system $\\mathcal{S}$ is governed by a time-invariant Hamiltonian $\\mathcal{H}(\\mathbf{q}, \\mathbf{p})$ subject to a homogeneous gravitational potential field $\\Phi\_g$. The temporal propagation of the phase-space trajectory $\\Gamma(t)$ is approximated via a first-order Symplectic Euler discretization scheme, ensuring the preservation of the symplectic 2-form $\\omega = d\\mathbf{q} \\wedge d\\mathbf{p}$. Discontinuous state transitions arising from boundary constraints and inter-agent scattering events are resolved via an impulse-based kinematic renormalization predicated on the conservation of linear momentum $\\mathbf{P}$ and the restitution of kinetic energy $T$.



2\. Nomenclature \& Phase Space Definition



Let the simulation domain $\\Omega$ be defined as a compact, convex subset of Euclidean 3-space, specifically an axis-aligned hexahedron:





$$\\Omega = \\{ \\mathbf{x} \\in \\mathbb{R}^3 \\mid \\|\\mathbf{x}\\|\_\\infty \\le \\frac{L}{2} \\}$$



The system state at discrete time step $n \\in \\mathbb{N}$ is the Cartesian product of the states of $N$ particles. For an arbitrary particle $i$, the state vector $\\mathbf{\\psi}\_i$ resides in the tangent bundle $T\\mathbb{R}^3$:



$$\\mathbf{\\psi}\_i\[n] = \\begin{bmatrix} \\mathbf{q}\_i\[n] \\\\ \\mathbf{v}\_i\[n] \\end{bmatrix} \\in \\mathbb{R}^6$$



Where:



$\\mathbf{q}\_i \\in \\mathbb{R}^3$: The generalized position coordinates.



$\\mathbf{v}\_i \\equiv \\dot{\\mathbf{q}}\_i \\in \\mathbb{R}^3$: The generalized velocity coordinates.



$m\_i \\in \\mathbb{R}^+$: The inertial mass scalar.



3\. Discretized Equations of Motion



The continuous evolution of the system is governed by Newton's Second Law, $\\mathbf{F}\_{net} = \\dot{\\mathbf{p}}$, where $\\mathbf{p} = m\\mathbf{v}$. We approximate the integral curve using a semi-implicit symplectic integrator.



Let $\\Delta t \\in \\mathbb{R}^+$ be the temporal quantization step. The mapping $\\mathcal{T}: \\mathbb{R}^6 \\to \\mathbb{R}^6$ transforms the state from $t$ to $t + \\Delta t$:



$$\\begin{aligned} \\forall i \\in \\{1, \\dots, N\\}: \\\\ \\mathbf{v}\_i\[n+1] \&= \\mathbf{v}\_i\[n] + \\Delta t \\cdot \\mathcal{A}\_i(\\mathbf{q}\_i\[n]) \\\\ \\mathbf{q}\_i\[n+1] \&= \\mathbf{q}\_i\[n] + \\Delta t \\cdot \\mathbf{v}\_i\[n+1] \\end{aligned}$$



Lemma 3.1 (Symplecticity):

The Jacobian $J\_{\\mathcal{T}}$ of the transformation has a determinant of unity ($\\det(J\_{\\mathcal{T}}) = 1$), implying that the phase-space volume is conserved (Liouville's Theorem is satisfied discretely), unlike in explicit Runge-Kutta methods where energy drift is $\\mathcal{O}(\\Delta t^k)$.



4\. Collision Manifold Resolution \& Impulse Derivation



The system is non-smooth due to hard-sphere constraints. We define the inter-penetration set $\\mathcal{C}\_{ij}$ for any pair $(i, j)$ as:





$$\\mathcal{C}\_{ij} = \\{ (\\mathbf{q}\_i, \\mathbf{q}\_j) \\in \\mathbb{R}^3 \\times \\mathbb{R}^3 \\mid \\|\\mathbf{q}\_i - \\mathbf{q}\_j\\|\_2 < (r\_i + r\_j) \\}$$



Upon detection of $\\mathbf{q} \\in \\mathcal{C}\_{ij}$, we invoke an instantaneous velocity projection.



4.1. Derivation of the Impulse Tensor $\\mathbf{J}$



Let $\\mathbf{v}\_{rel} = \\mathbf{v}\_i - \\mathbf{v}\_j$ be the relative velocity.

Let $\\hat{\\mathbf{n}} = \\frac{\\mathbf{q}\_i - \\mathbf{q}\_j}{\\|\\mathbf{q}\_i - \\mathbf{q}\_j\\|}$ be the normalized collision normal.



We seek an impulse scalar $\\lambda$ such that the post-collision relative velocity $\\mathbf{v}'\_{rel}$ satisfies Newton's Law of Restitution with coefficient $\\varepsilon \\in \[0, 1]$:





$$\\mathbf{v}'\_{rel} \\cdot \\hat{\\mathbf{n}} = -\\varepsilon (\\mathbf{v}\_{rel} \\cdot \\hat{\\mathbf{n}})$$



From the conservation of momentum $\\Delta \\mathbf{p}\_i = -\\Delta \\mathbf{p}\_j = \\lambda \\hat{\\mathbf{n}}$:





$$\\mathbf{v}'\_i = \\mathbf{v}\_i + \\frac{\\lambda}{m\_i}\\hat{\\mathbf{n}}, \\quad \\mathbf{v}'\_j = \\mathbf{v}\_j - \\frac{\\lambda}{m\_j}\\hat{\\mathbf{n}}$$



Substituting into the relative velocity equation:





$$(\\mathbf{v}'\_i - \\mathbf{v}'\_j) \\cdot \\hat{\\mathbf{n}} = \\left( (\\mathbf{v}\_i - \\mathbf{v}\_j) + \\lambda \\left(\\frac{1}{m\_i} + \\frac{1}{m\_j}\\right)\\hat{\\mathbf{n}} \\right) \\cdot \\hat{\\mathbf{n}}$$



Since $\\hat{\\mathbf{n}} \\cdot \\hat{\\mathbf{n}} = 1$:





$$-\\varepsilon (\\mathbf{v}\_{rel} \\cdot \\hat{\\mathbf{n}}) = (\\mathbf{v}\_{rel} \\cdot \\hat{\\mathbf{n}}) + \\lambda \\left(\\frac{1}{m\_i} + \\frac{1}{m\_j}\\right)$$



Theorem 4.1 (Impulse Magnitude):

Solving for $\\lambda$ yields the governing equation for the elastic scattering event:





$$\\lambda = \\frac{-(1+\\varepsilon)(\\mathbf{v}\_{rel} \\cdot \\hat{\\mathbf{n}})}{m\_i^{-1} + m\_j^{-1}}$$



$$\\therefore \\mathbf{J} = \\lambda \\hat{\\mathbf{n}}$$



This impulse $\\mathbf{J}$ is applied strictly when $\\mathbf{v}\_{rel} \\cdot \\hat{\\mathbf{n}} < 0$ (approaching phase).



5\. Algorithmic Orthogonalization



The implementation architecture is bifurcated into strictly orthogonalized modules to maximize cohesion and minimize coupling $\\chi$.



vec3: Implements the vector space axioms for $\\mathbb{V}^3$.



physics: Encapsulates the Lagrangian state integration.



collision: Computes the intersection of convex sets and resolves the Linear Complementarity Problem (LCP) for non-penetration constraints.



scene: Manages the $\\mathcal{O}(N)$ entity list and spatial queries.



renderer: Maps $\\mathbb{R}^3 \\to \\mathbb{Z}^2$ via perspective projection matrices $\\mathbf{P}\_{proj}$.

