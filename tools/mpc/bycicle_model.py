import casadi
from casadi import interpolant, vertcat, MX, types, tanh, cos, sin, Function
from tracks.readDataFcn import getTrack
import numpy as np


def bycicle_model(track="LMS_Track.txt"):
    # define structs
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    # load track parameters
    [s0, _, _, _, kapparef] = getTrack(track)
    length = len(s0)
    pathlength = s0[-1]
    # copy loop to beginning and end
    s0 = np.append(s0, [s0[length - 1] + s0[1:length]])
    kapparef = np.append(kapparef, kapparef[1:length])
    s0 = np.append([-s0[length - 2] + s0[length - 81 : length - 2]], s0)
    kapparef = np.append(kapparef[length - 80 : length - 1], kapparef)

    # compute spline interpolations
    kapparef_s = interpolant("kapparef_s", "bspline", [s0], kapparef)

    ## Race car parameters
    m = 0.043
    C1 = 0.5
    C2 = 15.5
    Cm1 = 0.28
    Cm2 = 0.05
    Cr0 = 0.011
    Cr2 = 0.006

    ## CasADi Model
    # set up states & controls
    s = MX.sym("s")  # longitudinal distance
    n = MX.sym("n")  # lateral distance
    alpha = MX.sym("alpha")  # heading ???
    v = MX.sym("v")  # velocity
    D = MX.sym("D")  # acceleration ???
    delta = MX.sym("delta")  # steering angle
    x = vertcat(s, n, alpha, v, D, delta)

    # controls
    derD = MX.sym("derD")
    derDelta = MX.sym("derDelta")
    u = vertcat(derD, derDelta)

    # xdot
    sdot = MX.sym("sdot")
    ndot = MX.sym("ndot")
    alphadot = MX.sym("alphadot")
    vdot = MX.sym("vdot")
    Ddot = MX.sym("Ddot")
    deltadot = MX.sym("deltadot")
    xdot = vertcat(sdot, ndot, alphadot, vdot, Ddot, deltadot)

    # algebraic variables
    z = vertcat([])

    # parameters
    p = vertcat([])

    # dynamics
    Fxd = (Cm1 - Cm2 * v) * D - Cr2 * v * v - Cr0 * tanh(5 * v)
    sdota = (v * cos(alpha + C1 * delta)) / (1 - kapparef_s(s) * n)
    f_expl = vertcat(
        sdota,  # s_dot
        v * sin(alpha + C1 * delta),  # n_dot
        v * C2 * delta - kapparef_s(s) * sdota,  # alpha_dot
        Fxd / m * cos(C1 * delta),  # v_dot, aka accel
        derD,  # D_dot
        derDelta,  # delta_dot
    )

    # constraint on forces
    a_lat = C2 * v * v * delta + Fxd * sin(C1 * delta) / m
    a_long = Fxd / m

    # Model bounds
    model.n_min = -0.12  # width of the track [m]
    model.n_max = 0.12  # width of the track [m]

    # state bounds
    model.throttle_min = -1.0
    model.throttle_max = 1.0

    model.delta_min = -0.40  # minimum steering angle [rad]
    model.delta_max = 0.40  # maximum steering angle [rad]

    # input bounds
    model.ddelta_min = -2.0  # minimum change rate of stering angle [rad/s]
    model.ddelta_max = 2.0  # maximum change rate of steering angle [rad/s]
    model.dthrottle_min = -10  # -10.0  # minimum throttle change rate
    model.dthrottle_max = 10  # 10.0  # maximum throttle change rate

    # nonlinear constraint
    constraint.alat_min = -4  # maximum lateral force [m/s^2]
    constraint.alat_max = 4  # maximum lateral force [m/s^1]

    constraint.along_min = -4  # maximum lateral force [m/s^2]
    constraint.along_max = 4  # maximum lateral force [m/s^2]

    # Define initial conditions
    model.x0 = np.array([-2, 0, 0, 0, 0, 0])

    # define constraints struct
    constraint.alat = Function("a_lat", [x, u], [a_lat])
    constraint.pathlength = pathlength
    constraint.expr = vertcat(a_long, a_lat, n, D, delta)

    # Define model struct
    params = types.SimpleNamespace()
    params.C1 = C1
    params.C2 = C2
    params.Cm1 = Cm1
    params.Cm2 = Cm2
    params.Cr0 = Cr0
    params.Cr2 = Cr2
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = z
    model.p = p
    model.name = "Spatialbycicle_model"
    model.params = params
    return model, constraint