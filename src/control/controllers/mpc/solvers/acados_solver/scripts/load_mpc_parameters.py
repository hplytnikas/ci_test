from os.path import dirname, join, abspath
import yaml


def load_mpc_yaml_params():
    """
    Load the parameters from the yaml files using relative paths.

    Returns:
        tuple: Tuple containing the constraint parameters, cost parameters, model parameters and solver parameters.

    """
    constraint_param_path = join(
        dirname(abspath(__file__)),
        "../../mpc_solver_common/config/constraint_parameters.yaml",
    )
    cost_param_path = join(
        dirname(abspath(__file__)),
        "../../mpc_solver_common/config/cost_parameters.yaml",
    )
    model_param_path = join(
        dirname(abspath(__file__)),
        "../../mpc_solver_common/config/model_parameters.yaml",
    )
    acados_solver_param_path = join(
        dirname(abspath(__file__)), "../config/acados_solver_parameters.yaml"
    )
    common_solver_param_path = join(
        dirname(abspath(__file__)),
        "../../mpc_solver_common/config/solver_parameters.yaml",
    )

    with open(constraint_param_path, "r") as file:
        constraint_params = yaml.safe_load(file)

    with open(cost_param_path, "r") as file:
        cost_params = yaml.safe_load(file)

    with open(model_param_path, "r") as file:
        model_params = yaml.safe_load(file)

    with open(acados_solver_param_path, "r") as file:
        acados_solver_params = yaml.safe_load(file)

    with open(common_solver_param_path, "r") as file:
        common_solver_params = yaml.safe_load(file)
    with open(acados_solver_param_path, "r") as file:
        acados_solver_params = yaml.safe_load(file)

    with open(common_solver_param_path, "r") as file:
        common_solver_params = yaml.safe_load(file)

    return (
        constraint_params,
        cost_params,
        model_params,
        acados_solver_params,
        common_solver_params,
    )
