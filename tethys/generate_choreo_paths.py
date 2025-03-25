import json
import pathlib
from jinja2 import Template


def load_choreo_variables(choreo_file: pathlib.Path):
    data = json.loads(choreo_file.read_text())
    variables = {}
    for var_name, var in data["variables"]["poses"].items():
        print(var_name)
        variables[var_name] = dict(
            name=var_name, x=var["x"]["val"], y=var["y"]["val"], heading=var["heading"]["val"]
        )

    return variables


def create_path(choreo_dir, variables, first_variable, second_variable, keepout=False):
    filename = f"{first_variable}To{second_variable}"

    template = TRAJECTORY_TEMPLATE
    if keepout:
        template = TRAJECTORY_KEEPOUT_TEMPLATE
        print("Keepout zone")

    contents = Template(template).render(
        name=filename,
        first_pose=variables[first_variable],
        second_pose=variables[second_variable],
    )

    path_to_write = choreo_dir / f"{filename}.traj"
    path_to_write.write_text(contents)


def main():
    root_dir = pathlib.Path(r".")
    choreo_dir = root_dir / r"src\main\deploy\choreo"
    print(choreo_dir / r"ChoreoAutos.chor")
    variables = load_choreo_variables(choreo_dir / r"ChoreoAutos.chor")

    for reef_position in ["A", "B", "L", "K", "J", "I", "H"]:
        create_path(choreo_dir, variables, reef_position, "HumanPlayerLeftFar", True)
        create_path(choreo_dir, variables, "HumanPlayerLeftFar", reef_position)
        create_path(choreo_dir, variables, reef_position, "HumanPlayerLeftClose", True)
        create_path(choreo_dir, variables, "HumanPlayerLeftClose", reef_position)

    for reef_position in ["A", "B", "C", "D", "E", "F", "G"]:
        create_path(choreo_dir, variables, reef_position, "HumanPlayerRightFar", True)
        create_path(choreo_dir, variables, "HumanPlayerRightFar", reef_position)
        create_path(choreo_dir, variables, reef_position, "HumanPlayerRightClose", True)
        create_path(choreo_dir, variables, "HumanPlayerRightClose", reef_position)

    for reef_position in ["D", "E", "F", "G"]:
        create_path(choreo_dir, variables, "StartingPosRight", reef_position)

    for reef_position in ["H", "I", "J", "K"]:
        create_path(choreo_dir, variables, "StartingPosLeft", reef_position)

    for reef_position in ["H", "G"]:
        create_path(choreo_dir, variables, "StartingPosCenter", reef_position)

    for algae_position in ["GH", "EF", "IJ"]:
        create_path(choreo_dir, variables, algae_position, "Processor")
        create_path(choreo_dir, variables, "Processor", algae_position)


TRAJECTORY_TEMPLATE = """{
 "name":"{{ name }}",
 "version":1,
 "snapshot":{
  "waypoints":[],
  "constraints":[],
  "targetDt":0.05
 },
 "params":{
  "waypoints":[
    {"x":{"exp":"{{ first_pose.name }}.x", "val":{{ first_pose.x }}}, "y":{"exp":"{{ first_pose.name }}.y", "val":{{ first_pose.y }}}, "heading":{"exp":"{{ first_pose.name }}.heading", "val":{{ first_pose.heading }}}, "intervals":40, "split":false, "fixTranslation":true, "fixHeading":true, "overrideIntervals":false},
    {"x":{"exp":"{{ second_pose.name }}.x", "val":{{ second_pose.x }}}, "y":{"exp":"{{ second_pose.name }}.y", "val":{{ second_pose.y }}}, "heading":{"exp":"{{ second_pose.name }}.heading", "val":{{ second_pose.heading }}}, "intervals":40, "split":false, "fixTranslation":true, "fixHeading":true, "overrideIntervals":false}],
  "constraints":[
    {"from":"first", "to":null, "data":{"type":"StopPoint", "props":{}}, "enabled":true},
    {"from":"last", "to":null, "data":{"type":"StopPoint", "props":{}}, "enabled":true},
    {"from":0, "to":1, "data":{"type":"MaxVelocity", "props":{"max":{"exp":"DefaultMaxVelocity", "val":1.524}}}, "enabled":true}],
  "targetDt":{
   "exp":"0.05 s",
   "val":0.05
  }
 },
 "trajectory":{
  "sampleType":null,
  "waypoints":[],
  "samples":[],
  "splits":[]
 },
 "events":[]
}

"""

TRAJECTORY_KEEPOUT_TEMPLATE = """{
 "name":"{{ name }}",
 "version":1,
 "snapshot":{
  "waypoints":[],
  "constraints":[],
  "targetDt":0.05
 },
 "params":{
  "waypoints":[
    {"x":{"exp":"{{ first_pose.name }}.x", "val":{{ first_pose.x }}}, "y":{"exp":"{{ first_pose.name }}.y", "val":{{ first_pose.y }}}, "heading":{"exp":"{{ first_pose.name }}.heading", "val":{{ first_pose.heading }}}, "intervals":40, "split":false, "fixTranslation":true, "fixHeading":true, "overrideIntervals":false},
    {"x":{"exp":"{{ second_pose.name }}.x", "val":{{ second_pose.x }}}, "y":{"exp":"{{ second_pose.name }}.y", "val":{{ second_pose.y }}}, "heading":{"exp":"{{ second_pose.name }}.heading", "val":{{ second_pose.heading }}}, "intervals":40, "split":false, "fixTranslation":true, "fixHeading":true, "overrideIntervals":false}],
  "constraints":[
    {"from":"first", "to":null, "data":{"type":"StopPoint", "props":{}}, "enabled":true},
    {"from":"last", "to":null, "data":{"type":"StopPoint", "props":{}}, "enabled":true},
    {"from":0, "to":1, "data":{"type":"MaxVelocity", "props":{"max":{"exp":"DefaultMaxVelocity", "val":1.524}}}, "enabled":true},
    {"from":"first", "to":"last", "data":{"type":"KeepOutCircle", "props":{"x":{"exp":"4.500539489090443 m", "val":4.500539489090443}, "y":{"exp":"4.011054694652557 m", "val":4.011054694652557}, "r":{"exp":"0.9088200308047361 m", "val":0.908820030804736}}}, "enabled":true},
    {"from":"first", "to":"last", "data":{"type":"KeepOutCircle", "props":{"x":{"exp":"4.485787129029632 m", "val":4.485787129029632}, "y":{"exp":"3.1479875072836876 m", "val":3.1479875072836876}, "r":{"exp":"0.1033116783456731 m", "val":0.1033116783456731}}}, "enabled":true},
    {"from":"first", "to":"last", "data":{"type":"KeepOutCircle", "props":{"x":{"exp":"5.252504387870431 m", "val":5.252504387870431}, "y":{"exp":"3.5905269272625446 m", "val":3.5905269272625446}, "r":{"exp":"0.1033116783456731 m", "val":0.1033116783456731}}}, "enabled":true},
    {"from":"first", "to":"last", "data":{"type":"KeepOutCircle", "props":{"x":{"exp":"3.732968019321561 m", "val":3.732968019321561}, "y":{"exp":"3.5768594816327095 m", "val":3.5768594816327095}, "r":{"exp":"0.1033116783456731 m", "val":0.1033116783456731}}}, "enabled":true},
    {"from":"first", "to":"last", "data":{"type":"KeepOutCircle", "props":{"x":{"exp":"3.729197174310684 m", "val":3.729197174310684}, "y":{"exp":"4.468318946659565 m", "val":4.468318946659565}, "r":{"exp":"0.1033116783456731 m", "val":0.1033116783456731}}}, "enabled":true},
    {"from":"first", "to":"last", "data":{"type":"KeepOutCircle", "props":{"x":{"exp":"4.487039112253115 m", "val":4.487039112253115}, "y":{"exp":"4.906476908363402 m", "val":4.906476908363402}, "r":{"exp":"0.1033116783456731 m", "val":0.1033116783456731}}}, "enabled":true},
    {"from":"first", "to":"last", "data":{"type":"KeepOutCircle", "props":{"x":{"exp":"5.245879590511322 m", "val":5.245879590511322}, "y":{"exp":"4.463507607579231 m", "val":4.463507607579231}, "r":{"exp":"0.1033116783456731 m", "val":0.1033116783456731}}}, "enabled":true}],
  "targetDt":{
   "exp":"0.05 s",
   "val":0.05
  }
 },
 "trajectory":{
  "sampleType":null,
  "waypoints":[],
  "samples":[],
  "splits":[]
 },
 "events":[]
}

"""

if __name__ == "__main__":
    # py -m y2025.Reefscape.generate_choreo_mini_paths
    main()