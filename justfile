# Delegates to the `e-axle/justfile`
mod e-axle
# Delegates to the `fluids/justfile`
mod fluids

@_default:
    just --list

# Runs both `e-axle::all` and `fluids::all`
@all: e-axle::all fluids::all
