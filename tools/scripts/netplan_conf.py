#!/usr/bin/env python3
"""
This script makes user choose netplan yaml file,
    makes user choose the right interface,
    updates the interface in the file to static ip while also creating a bak (backup file),
    can restore from the backup file,
    launches sudo netplan apply
"""

import os
import shutil
import sys
import re
import yaml
import subprocess
from pathlib import Path

"""List all network interfaces excluding 'lo'."""


def list_network_interfaces():
    output = subprocess.check_output(["ip", "-o", "link", "show"]).decode()
    interfaces = re.findall(r"\d+: ([^:]+):", output)
    return [interface for interface in interfaces if interface != "lo"]


"""Let the user choose an item from a list."""


def choose_from_list(items, prompt):
    for idx, item in enumerate(items, start=1):
        print(f"{idx}) {item}")
    choice = int(input(prompt))
    return items[choice - 1]


"""Find all YAML files in the specified directory."""


def find_yaml_files(directory):
    return list(Path(directory).rglob("*.yaml"))


"""Update the specified YAML file by replacing the block of the chosen_interface
    with the content from the template file."""


def update_yaml_file(yaml_file, chosen_interface, template_file):
    with open(template_file, "r") as file:
        template_content = yaml.safe_load(file)
        if "$network_interface" in template_content:
            interface_content = template_content["$network_interface"]
        else:
            interface_content = template_content

    with open(yaml_file, "r") as file:
        data = yaml.safe_load(file)
        if not data:
            data = {}

    # Ensure "network" key exists and is a dictionary
    if ("network" not in data) or (data["network"] is None):
        data["network"] = {"ethernets": {chosen_interface: {}}}

    # Ensure "ethernets" key exists within "network" and is a dictionary
    if ("ethernets" not in data["network"]) or (data["network"]["ethernets"] is None):
        data["network"]["ethernets"] = {chosen_interface: {}}

    # Ensure chosen_interface key exists within "ethernets" and is a dictionary
    if (chosen_interface not in data["network"]["ethernets"]) or (
        data["network"]["ethernets"] is None
    ):
        data["network"]["ethernets"][chosen_interface] = {}

    data["network"]["ethernets"][chosen_interface] = interface_content

    with open(yaml_file, "w") as file:
        yaml.safe_dump(data, file, sort_keys=False, default_flow_style=False)
    print(f"Updated {yaml_file}.")


"""Restore the specified YAML file from its backup."""


def restore_from_backup(yaml_file):
    bak_file = str(yaml_file) + ".bak"
    if os.path.exists(bak_file):
        shutil.move(bak_file, yaml_file)
        print(f"Restored {yaml_file} from backup.")
    else:
        print("Backup file not found.")


def run_commands():
    os.system("sudo netplan apply")


def main():
    # Check if the script is running as root. If not, re-run it with sudo.
    if os.geteuid() != 0:
        os.execvp("sudo", ["sudo", "python3"] + sys.argv)

    action = "static"

    if len(sys.argv) >= 2:
        action = sys.argv[1].lower()

    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    template_file = os.path.join(
        scripts_dir, ".templates", "netplan_conf_template.yaml"
    )
    template_new_file = os.path.join(
        scripts_dir, ".templates", "netplan_new_conf_template.yaml"
    )
    netplan_dir = "/etc/netplan/"

    yaml_files = find_yaml_files(netplan_dir)
    no_yaml = False
    if not yaml_files:
        print(f"No .yaml files found in {netplan_dir}.")
        os.makedirs(os.path.dirname(netplan_dir), exist_ok=True)
        shutil.copy(template_new_file, netplan_dir + "default.yaml")
        print(f"Created new file now updating {netplan_dir} default.yaml.")
        no_yaml = True
    yaml_files = find_yaml_files(netplan_dir)

    if action == "static" or action == "":
        template_file = os.path.join(
            scripts_dir, ".templates", "netplan_conf_template.yaml"
        )
        chosen_yaml_file = (
            yaml_files[0]
            if len(yaml_files) == 1
            else choose_from_list(
                yaml_files,
                "Please enter the number of the YAML file you want to update: ",
            )
        )
        interfaces = list_network_interfaces()
        if not interfaces:
            print("No network interfaces found.")
            return

        chosen_interface = choose_from_list(
            interfaces, "Please enter the number of the interface you want to choose: "
        )
        print(f"You have chosen the interface: {chosen_interface}")

        if not os.path.exists(f"{chosen_yaml_file}.bak") and not no_yaml:
            shutil.copy(chosen_yaml_file, f"{chosen_yaml_file}.bak")
            print(f"Backup created for {chosen_yaml_file}.")

        update_yaml_file(chosen_yaml_file, chosen_interface, template_file)

        run_commands()
    elif action == "restore":
        chosen_yaml_file = (
            yaml_files[0]
            if len(yaml_files) == 1
            else choose_from_list(
                yaml_files,
                "Please enter the number of the YAML file you want to restore: ",
            )
        )
        restore_from_backup(chosen_yaml_file)

        run_commands()
    elif action == "dhcp":
        template_file = os.path.join(
            scripts_dir, ".templates", "netplan_conf_dhcp_template.yaml"
        )
        chosen_yaml_file = (
            yaml_files[0]
            if len(yaml_files) == 1
            else choose_from_list(
                yaml_files,
                "Please enter the number of the YAML file you want to update: ",
            )
        )
        interfaces = list_network_interfaces()
        if not interfaces:
            print("No network interfaces found.")
            return

        chosen_interface = choose_from_list(
            interfaces, "Please enter the number of the interface you want to choose: "
        )
        print(f"You have chosen the interface: {chosen_interface}")

        if not os.path.exists(f"{chosen_yaml_file}.bak"):
            shutil.copy(chosen_yaml_file, f"{chosen_yaml_file}.bak")
            print(f"Backup created for {chosen_yaml_file}.")

        update_yaml_file(chosen_yaml_file, chosen_interface, template_file)

        run_commands()
    elif action == "--help":
        print(
            """
            ./netplan_conf | ./netplan_conf static
                    Run without argument or with static to update the netplan file
            ./netplan_conf dhcp
                    Run with dhcp to update the netplan file to dhcp
            ./netplan_conf restore
                    Run with restore to restore the previous config
        """
        )
    else:
        print("Invalid action. Use 'static' or 'restore'.")
        sys.exit(1)


if __name__ == "__main__":
    main()
