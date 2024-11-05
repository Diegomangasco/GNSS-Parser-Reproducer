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

    def extract_data(self, content, message_type, direction=None):
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
                # For the DTM messages the latitude is expressed in DDDMM.MMMM
                if dict_key != "DTM":
                    lat_split = values[self.NMEA_MESSAGES[dict_key]["lat"]].split(".")
                    degrees = int(lat_split[0][:-2])
                    minutes = float(lat_split[0][-2:] + "." + lat_split[1])
                    lat = degrees + minutes/60
                    if direction is not None and direction ['S', 'W']:
                        lat = - lat
                else:
                    lat = float(values[self.NMEA_MESSAGES[dict_key]["lat"]])
            else:
                lat = None
            if self.NMEA_MESSAGES[dict_key]["lon"]:
                # For the DTM messages the longitude is expressed in DDDMM.MMMM
                if dict_key != "DTM":
                    lon_split = values[self.NMEA_MESSAGES[dict_key]["lon"]].split(".")
                    degrees = int(lon_split[0][:-2])
                    minutes = float(lon_split[0][-2:] + "." + lon_split[1])
                    lon = degrees + minutes/60
                    if direction is not None and direction in ['S', 'W']:
                        lon = - lon
                else:
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

            # UBX uses little endian and magnitude of 1e7 for lat and lon and 1e5 for heading
            lat = None if lat_offset is None else int.from_bytes(content[lat_offset+6:lat_offset+10], byteorder='little', signed=True) / 1e7
            lon = None if lon_offset is None else int.from_bytes(content[lon_offset+6:lon_offset+10], byteorder='little', signed=True) / 1e7
            heading = None if heading_offset is None else int.from_bytes(content[heading_offset+6:heading_offset+10], byteorder='little', signed=True) / 1e5
        
        return lat, lon, heading
    
    def get_ubx_message_type(self, content):
        """
        Returns the UBX message type.
        """
        if(len(content)>=4):
            if(content[2]==0x01 and content[3]==0x05):
                return "NAV-PVT"
            elif(content[2]==0x01 and content[3]==0x07):
                return "NAV-ATT"
            elif(content[2]==0x10 and content[3]==0x15):
                return "ESF-INS"
            elif(content[2]==0x10 and content[3]==0x03):
                return "ESF-RAW"
        else:
            return None