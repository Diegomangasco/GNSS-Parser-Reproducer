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
        b'\01\07': {
            "lat": 24,
            "lon": 28,
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

    def extract_data(self, content, type):
        """
        Extracts the latitude, longitude, and heading.
        """
        if type == "NMEA":
            values = content.split(",")
            if self.NMEA_MESSAGES[self.message_type]["lat"]:
                lat = float(values[self.NMEA_MESSAGES[self.message_type]["lat"]])
            else:
                lat = None
            if self.NMEA_MESSAGES[self.message_type]["lon"]:
                lon = float(values[self.NMEA_MESSAGES[self.message_type]["lon"]])
            else:
                lon = None
            if self.NMEA_MESSAGES[self.message_type]["heading"]:
                heading = float(values[self.NMEA_MESSAGES[self.message_type]["heading"]])
            else:
                heading = None
        elif type == "UBX":
            lat_offset = self.UBX_MESSAGES[self.message_type]["lat"]
            lon_offset = self.UBX_MESSAGES[self.message_type]["lon"]
            heading_offset = self.UBX_MESSAGES[self.message_type]["heading"]

            lat = None if lat_offset is None else int.from_bytes(content[lat_offset+6:lat_offset+10], byteorder='little', signed=True) / 1e7
            lon = None if lon_offset is None else int.from_bytes(content[lon_offset+6:lon_offset+10], byteorder='little', signed=True) / 1e7
            heading = None if heading_offset is None else int.from_bytes(content[heading_offset+6:heading_offset+10], byteorder='little', signed=True) / 1e5

        return lat, lon, heading