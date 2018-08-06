using ParticleFilters

const N_PARTICLES = 1000

function initial_particle_belief(pomdp::OCPOMDP)
    d0 = initial_state_distribution(pomdp)
    p = Vector{OCState}(N_PARTICLES)
    for i=1:N_PARTICLES
        p[i] = rand(rng, d0)
    end
    return ParticleCollection{OCState}(p)
end

function init_updater(pomdp::OCPOMDP, N::Int64 = N_PARTICLES)
    filter = SIRParticleFilter(pomdp, N)
    return filter
end

function POMDPs.action(policy::QMDPPolicy, b::ParticleCollection{OCState})
end
