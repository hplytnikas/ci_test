import ftplib
import json
import paramiko


def modify_json_on_server(
    hostname, username, password, params_to_change, path, filepath
):

    try:
        # Connect to the FTP server
        print(f"Connecting to {hostname}...")
        ftp = ftplib.FTP(hostname)
        ftp.login(username, password)

        # Change to the target directory
        print(f"Changing to {path} directory...")
        ftp.cwd(path)

        # Download the JSON file
        print(f"Downloading {filepath}...")
        with open("paramInfo.json", "wb") as file:
            ftp.retrbinary(f"RETR {filepath}", file.write)

        # Modify the JSON file
        print("Modifying parameters...")
        with open("paramInfo.json", "rb") as file:
            data = json.load(file)
            for param_name, new_value in params_to_change.items():
                changed = False
                for item in data["parameters"]:
                    if item.get("Name") == param_name:
                        item["Value"] = new_value
                        changed = True
                if not changed:
                    print(f"Haven't found parameter {param_name}")

        # Modify the local JSON file
        print("Modifying local file")
        with open("paramInfo.json", "w") as file:
            json.dump(data, file)

        # Delete file in server to avoid problems
        print("Deleting file in server")
        ftp.delete(filepath)

        # Upload the modified JSON file
        print("Uploading modified parameters...")
        with open("paramInfo.json", "rb") as file:
            file.seek(0)
            ftp.storbinary(f"STOR {filepath}", file)

        print("Successfully modified parameters on the server")

        # Close the FTP connection
        ftp.quit()

        # Connect via SSH
        print("Connecting via SSH...")
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(hostname, username=username, password=password)

        # Execute slrealtime command
        print("Executing slrealtime command...")
        stdin, stdout, stderr = ssh.exec_command("slrealtime loadParamSet -F paramInfo")

        # Print command output
        print(stdout.read().decode())

        # Close SSH connection
        print("Closing SSH connection...")
        ssh.close()

        print("Successfully changed parameters on the target hardware")

    except Exception as e:
        print(f"An error occurred: {e}")


# Replace these with your FTP server credentials
ftp_hostname = "192.168.1.111"
ftp_username = "slrt"
ftp_password = "slrt"
model_name = "dufour"

# Path to the JSON file on the server
path = f"/home/slrt/applications/{model_name}/paramSet"
filepath = "paramInfo.json"

# Read parameters to change from local JSON file
with open("params_to_change.json", "r") as params_file:
    params_to_change = json.load(params_file)

# Call the function to modify the JSON file on the server and execute SSH command
modify_json_on_server(
    ftp_hostname, ftp_username, ftp_password, params_to_change, path, filepath
)
