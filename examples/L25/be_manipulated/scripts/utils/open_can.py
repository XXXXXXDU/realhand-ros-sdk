import sys,os,time,subprocess
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from color_msg import ColorMsg
class OpenCan:
    def __init__(self):
        pass

    def open_can0(self):
        try:
            # Check if the can0 interface already exists and is in the 'up' state
            result = subprocess.run(
                ["ip", "link", "show", "can0"],
                check=True,
                text=True,
                capture_output=True
            )
            if "state UP" in result.stdout:
                return 
            # If not in UP state, configure the interface
            subprocess.run(
                ["sudo", "-S", "ip", "link", "set", "can0", "up", "type", "can", "bitrate", "1000000"],
                input=f"{self.password}\n",
                check=True,
                text=True,
                capture_output=True
            )
            
        except subprocess.CalledProcessError as e:
            pass
        except Exception as e:
            pass
            

    def is_can_up_sysfs(self, interface="can0"):
    # Check if the interface directory exists
        if not os.path.exists(f"/sys/class/net/{interface}"):
            return False
        # Read interface status
        try:
            with open(f"/sys/class/net/{interface}/operstate", "r") as f:
                state = f.read().strip()
            if state == "up":
                return True
        except Exception as e:
            print(f"Error reading CAN interface state: {e}")
            return False