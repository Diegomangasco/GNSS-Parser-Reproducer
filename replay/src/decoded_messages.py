class DecodedMessage:
    NMEA_MESSAGES = {
        "DTM": {
            "lat": 3,
            "lon": 5,
            "heading": None
        },
        "GGA": {
            "lat": 2,
            "lon": 4,
            "heading": None
        },
        "GLL": {
            "lat": 1,
            "lon": 3,
            "heading": None
        },
        "GNS": {
            "lat": 2,
            "lon": 4,
            "heading": None
        },
        "RMC": {
            "lat": 3,
            "lon": 5,
            "heading": None
        },
        "THS": {
            "lat": None,
            "lon": None,
            "heading": 1
        },
    }

    UBX_MESSAGES = {
        b'\x01\x05': {
            "lat": None,
            "lon": None,
            "heading": 16
        },
        b'\x01\x07': {
            "lat": 28,
            "lon": 24,
            "heading": 64
        },
        b'\x01\x12': {
            "lat": None,
            "lon": None,
            "heading": 24
        }
    }

    def __init__(self):
        pass

    def extract_data(self, content, message_type):
        """
        Extracts the latitude, longitude, and heading.
        """
        lat = None
        lon = None
        heading = None
        if message_type == "NMEA":
            values = content.split(",")
            dict_key = values[0][3:]
            if dict_key not in self.NMEA_MESSAGES.keys():
                return None, None, None
            if self.NMEA_MESSAGES[dict_key]["lat"]:
                lat = float(values[self.NMEA_MESSAGES[dict_key]["lat"]])
                
            else:
                lat = None
            if self.NMEA_MESSAGES[dict_key]["lon"]:
                lon = float(values[self.NMEA_MESSAGES[dict_key]["lon"]])
            else:
                lon = None
            if self.NMEA_MESSAGES[dict_key]["heading"]:
                heading = float(values[self.NMEA_MESSAGES[dict_key]["heading"]])
            else:
                heading = None
        elif message_type == "UBX":
            dict_key = content[2:4]
            if dict_key not in self.UBX_MESSAGES.keys():
                return None, None, None
            lat_offset = self.UBX_MESSAGES[dict_key]["lat"]
            lon_offset = self.UBX_MESSAGES[dict_key]["lon"]
            heading_offset = self.UBX_MESSAGES[dict_key]["heading"]

            lat = None if lat_offset is None else int.from_bytes(content[lat_offset+6:lat_offset+10], byteorder='little', signed=True) / 1e7
            lon = None if lon_offset is None else int.from_bytes(content[lon_offset+6:lon_offset+10], byteorder='little', signed=True) / 1e7
            heading = None if heading_offset is None else int.from_bytes(content[heading_offset+6:heading_offset+10], byteorder='little', signed=True) / 1e5
        return lat, lon, heading