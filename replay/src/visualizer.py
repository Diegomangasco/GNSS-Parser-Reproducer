import socket, os

class Visualizer:

    def __init__(self):
        pass

    def open_map_gui(lat, lon, server_ip, server_port):
        """
        Opens the map GUI.
        """
        message = f"map,{lat},{lon}"
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(message.encode(), (server_ip, server_port))
        except Exception as e:
            print(f"Error sending UDP message: {e}")
            raise e

    def start_nodejs_server(httpport, ip, port, fifo_path):
        """
        Starts the nodejs server for the vehicle visualizer.
        """
        try:
            os.system(f"node ./vehicle_visualizer/server.js {httpport} {ip} {port} {fifo_path} &")
        except Exception as e:
            print(f"Error starting nodejs server: {e}")
            raise e

    def send_object_udp_message(lat, lon, heading, server_ip, server_port):
        """
        Sends a UDP message with the latitude, longitude, and heading to the specified server.
        """
        id = 1
        station_type = 5
        if not heading:
            heading = 361
        message = f"object,{id},{lat},{lon},{station_type},{heading}"
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(message.encode(), (server_ip, server_port))
        except Exception as e:
            print(f"Error sending UDP message: {e}")
            raise e

    def stop_server(server_ip, server_port):
        """
        Stops the nodejs server for the vehicle visualizer.
        """
        message = f"terminate"
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(message.encode(), (server_ip, server_port))
        except Exception as e:
            print(f"Error stopping nodejs server: {e}")
            raise e