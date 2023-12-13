import json
import os

PROFILE_PATH = "/home/user/pidar_app/lens_profiles.json"

class LensProfile:
    def __init__(self, name):
        self.name = name
        self.calibration_data = []
        self.lens_position_data = []

    def add_calibration_data(self, focus, position):
        self.calibration_data.append({"focus": focus, "position": position})
    
    def add_lens_position_data(self, position):
        self.lens_position_data.append({"position": position})

    def get_calibration_data(self):
        return self.calibration_data
    
    def get_lens_position_data(self):
        return self.lens_position_data

    def to_dict(self):
        return {
            "name": self.name,
            "calibration_data": self.calibration_data,
            "lens_position_data": self.lens_position_data  # Add this line
        }

    @classmethod
    def from_dict(cls, name, data_dict):
        profile = cls(name)
        profile.calibration_data = data_dict.get("calibration_data", [])
        profile.lens_position_data = data_dict.get("lens_position_data", [])
        return profile

class LensProfileManager:
    def __init__(self):
        self.profiles = {}
        self.load_profiles()

    def load_profiles(self):
        if os.path.exists(PROFILE_PATH):
            with open(PROFILE_PATH, 'r') as file:
                raw_profiles = json.load(file)
                self.profiles = {name: LensProfile.from_dict(name, data) for name, data in raw_profiles.items()}

    def save_profiles(self):
        raw_profiles = {name: profile.to_dict() for name, profile in self.profiles.items()}
        with open(PROFILE_PATH, 'w') as file:
            json.dump(raw_profiles, file, indent=4)

    def add_profile(self, profile):
        self.profiles[profile.name] = profile
        self.save_profiles()

    def get_profile(self, name):
        return self.profiles.get(name)

    def delete_profile(self, name):
        if name in self.profiles:
            del self.profiles[name]
            self.save_profiles()
