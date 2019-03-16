using Documenter
using AutomotivePOMDPs

makedocs(
	 modules = [AutomotivePOMDPs],
	 sitename="AutomotivePOMDPs.jl"
	 )

deploydocs(
    repo = "github.com/sisl/AutomotivePOMDPs.jl.git",
)