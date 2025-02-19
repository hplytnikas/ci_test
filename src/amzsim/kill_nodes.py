import subprocess
import time


def get_all_running_nodes():
    # Start a subprocess
    node_fetch_process = subprocess.Popen(
        ["ros2", "node", "list"], stdout=subprocess.PIPE
    )

    process_output, _ = node_fetch_process.communicate()
    running_nodes = process_output.decode("utf-8").split("\n")[:-1]

    node_fetch_process.terminate()

    return running_nodes


def get_node_pids(node):
    pid_fetch_subprocess = subprocess.Popen(
        ["pgrep", "-f", node[1:].split("/")[0]], stdout=subprocess.PIPE
    )
    process_output, _ = pid_fetch_subprocess.communicate()
    running_pid = process_output.decode("utf-8").split("\n")[:-1]
    pid_fetch_subprocess.terminate()

    return running_pid


def kill_active_nodes():
    pids_running = set()
    pid_kill_subprocesses = list()

    running_nodes = get_all_running_nodes()

    if not running_nodes:
        print("No active nodes.")
        return

    # while len(running_nodes) != 0:
    for node in running_nodes:
        running_pid = get_node_pids(node)

        for pid in running_pid:
            if pid:
                pids_running.add(pid)

    for active_pid in pids_running:
        print(f"Killing active process with PID {active_pid}")
        kill_node_subprocess = subprocess.Popen(["kill", active_pid])
        pid_kill_subprocesses.append(kill_node_subprocess)

    while get_all_running_nodes():
        time.sleep(3.0)

    # for pid_kill in pid_kill_subprocesses:
    #     pid_kill.terminate()

    print(f"All active nodes sucessfully terminated.")


if __name__ == "__main__":
    kill_active_nodes()
