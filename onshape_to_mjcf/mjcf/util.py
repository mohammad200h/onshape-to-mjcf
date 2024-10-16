from .tree import Part
from ..onshape_api.config import config, configFile
from colorama import Fore, Back, Style
import json
import math
import numpy as np
import requests

from .components import MujocoGraphState

####### This is copied or modified code ########
partNames = {}
def extractPartName(name, configuration):
        parts = name.split(' ')
        del parts[-1]
        basePartName = '_'.join(parts).lower()

        # only add configuration to name if its not default and not a very long configuration (which happens for library parts like screws)
        if configuration != 'default' and len(configuration) < 40:
            parts += ['_' + configuration.replace('=', '_').replace(' ', '_')]

        return basePartName, '_'.join(parts).lower()


def partIsIgnore(name):
        if config['whitelist'] is None:
            return name in config['ignore']
        else:
            return name not in config['whitelist']

# This should probably be added to part_trees_to_node
# so as the XML is being generated the parts get saved
def addPart(client,partData:Part):
        # print(f"addPart::partData::type::{type(partData)}")
        occurrence = partData.occurence
        matrix = partData.transform
        part = occurrence['instance']

        if part['suppressed']:
            return

        if part['partId'] == '':
            print(Fore.YELLOW + 'WARNING: Part '+part['name']+' has no partId'+Style.RESET_ALL)
            return

        # Importing STL file for this part
        justPart, prefix = extractPartName(part['name'], part['configuration'])

        extra = ''
        if occurrence['instance']['configuration'] != 'default':
            extra = Style.DIM + ' (configuration: ' + \
                occurrence['instance']['configuration']+')'
        symbol = '+'
        if partIsIgnore(justPart):
            symbol = '-'
            extra += Style.DIM + ' / ignoring visual and collision'

        print(Fore.GREEN + symbol+' Adding part ' +
            occurrence['instance']['name']+extra + Style.RESET_ALL)

        if partIsIgnore(justPart):
            stlFile = None
        else:
            stlFile = prefix.replace('/', '_')+'.stl'
            # shorten the configuration to a maximum number of chars to prevent errors. Necessary for standard parts like screws
            if len(part['configuration']) > 40:
                shortend_configuration = hashlib.md5(
                    part['configuration'].encode('utf-8')).hexdigest()
            else:
                shortend_configuration = part['configuration']
            stl = client.part_studio_stl_m(part['documentId'], part['documentMicroversion'], part['elementId'],
                                        part['partId'], shortend_configuration)
            with open(config['outputDirectory']+'/'+stlFile, 'wb') as stream:
                stream.write(stl)

            stlMetadata = prefix.replace('/', '_')+'.part'
            with open(config['outputDirectory']+'/'+stlMetadata, 'w', encoding="utf-8") as stream:
                json.dump(part, stream, indent=4, sort_keys=True)

            stlFile = config['outputDirectory']+'/'+stlFile

        # Import the SCAD files pure shapes
        shapes = None
        if config['useScads']:
            scadFile = prefix+'.scad'
            if os.path.exists(config['outputDirectory']+'/'+scadFile):
                shapes = csg.process(
                    config['outputDirectory']+'/'+scadFile, config['pureShapeDilatation'])

def get_assembly(client):
    assembly = None

    # If a versionId is provided, it will be used,
    # else the main workspace is retrieved
    if config["versionId"] != "":
        print(
            "\n"
            + Style.BRIGHT
            + "* Using configuration version ID "
            + config["versionId"]
            + " ..."
            + Style.RESET_ALL
        )
    elif config["workspaceId"] != "":
        print(
            "\n"
            + Style.BRIGHT
            + "* Using configuration workspace ID "
            + config["workspaceId"]
            + " ..."
            + Style.RESET_ALL
        )
        workspaceId = config["workspaceId"]
    else:
        print("\n" + Style.BRIGHT + "* Retrieving workspace ID ..." + Style.RESET_ALL)
        document = client.get_document(config["documentId"]).json()
        workspaceId = document["defaultWorkspace"]["id"]
        print(Fore.GREEN + "+ Using workspace id: " + workspaceId + Style.RESET_ALL)

    # Now, finding the assembly, according to given name in configuration,
    # or else the first possible one
    print(
        "\n"
        + Style.BRIGHT
        + "* Retrieving elements in the document, searching for the assembly..."
        + Style.RESET_ALL
    )
    if config["versionId"] != "":
        elements = client.list_elements(
            config["documentId"], config["versionId"], "v"
        ).json()
    else:
        elements = client.list_elements(config["documentId"], workspaceId).json()
    assemblyId = None
    assemblyName = ""
    for element in elements:
        if element["type"] == "Assembly" and (
            config["assemblyName"] is False or element["name"] == config["assemblyName"]
        ):
            print(
                Fore.GREEN
                + "+ Found assembly, id: "
                + element["id"]
                + ', name: "'
                + element["name"]
                + '"'
                + Style.RESET_ALL
            )
            assemblyName = element["name"]
            assemblyId = element["id"]

    if assemblyId == None:
        print(
            Fore.RED + "ERROR: Unable to find assembly in this document" + Style.RESET_ALL
        )
        exit(1)

    # Retrieving the assembly
    print(
        "\n"
        + Style.BRIGHT
        + '* Retrieving assembly "'
        + assemblyName
        + '" with id '
        + assemblyId
        + Style.RESET_ALL
    )
    if config["versionId"] != "":
        assembly = client.get_assembly(
            config["documentId"],
            config["versionId"],
            assemblyId,
            "v",
            configuration=config["configuration"],
        )
    else:
        assembly = client.get_assembly(
            config["documentId"],
            workspaceId,
            assemblyId,
            configuration=config["configuration"],
        )

    return assembly

def findInstance(assembly,path, instances=None):

    if instances is None:
        instances = assembly["rootAssembly"]["instances"]

    for instance in instances:
        if instance["id"] == path[0]:
            if len(path) == 1:
                # If the length of remaining path is 1, the part is in the current assembly/subassembly
                return instance
            else:
                # Else, we need to find the matching sub assembly to find the proper part (recursively)
                d = instance["documentId"]
                m = instance["documentMicroversion"]
                e = instance["elementId"]
                for asm in assembly["subAssemblies"]:
                    if (
                        asm["documentId"] == d
                        and asm["documentMicroversion"] == m
                        and asm["elementId"] == e
                    ):
                        return findInstance(assembly,path[1:], asm["instances"])

    print(Fore.RED + "Could not find instance for " + str(path) + Style.RESET_ALL)

def get_color(client,part):
    # Obtain metadatas about part to retrieve color
    if config['color'] is not None:
        color = config['color']
    else:
        # print(f"get_color::part::keys::{part.keys()}")
        # print(f"get_color::part::{part}")
        metadata = client.part_get_metadata(
            part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'], part['configuration'])
        color = [0.5, 0.5, 0.5]
        # XXX: There must be a better way to retrieve the part color
        for entry in metadata['properties']:
            if 'value' in entry and type(entry['value']) is dict and 'color' in entry['value']:
                rgb = entry['value']['color']
                color = np.array(
                    [rgb['red'], rgb['green'], rgb['blue']])/255.0

    return color.tolist()+[1]

def getMeshName(occurrence):
    part = occurrence['instance']
    justPart, prefix = extractPartName(part['name'], part['configuration'])
    # print(f"getMeshName::part::{part}")
    # print(f"getMeshName::justPart::{justPart}")
    # print(f"getMeshName::prefix::{prefix}")

    return justPart,prefix,part

def processPartName(name, configuration, overrideName=None):
      if overrideName is None:
          global partNames
          _, name = extractPartName(name, configuration)
          if name in partNames:
              partNames[name] += 1
          else:
              partNames[name] = 1
          if partNames[name] == 1:
              return name
          else:
              return name+'_'+str(partNames[name])
      else:
          return overrideName


def get_inetia_prop(client,prefix,part):
    # Obtain mass properties about that part
    if config['noDynamics']:
        mass = 0
        com = [0]*3
        inertia = [0]*12
    else:
        if prefix in config['dynamicsOverride']:
            entry = config['dynamicsOverride'][prefix]
            mass = entry['mass']
            com = entry['com']
            inertia = entry['inertia']
        else:
            if part['isStandardContent']:
                massProperties = client.standard_cont_mass_properties(
                    part['documentId'], part['documentVersion'], part['elementId'], part['partId'],config['documentId'], part['configuration'])
            else:
                massProperties = client.part_mass_properties(
                    part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'], part['configuration'])
            if part['partId'] not in massProperties['bodies']:
                print(Fore.YELLOW + 'WARNING: part ' +
                    part['name']+' has no dynamics (maybe it is a surface)' + Style.RESET_ALL)
                return
            massProperties = massProperties['bodies'][part['partId']]
            mass = massProperties['mass'][0]
            com = massProperties['centroid']
            inertia = massProperties['inertia']
            if abs(mass) < 1e-9:
                print(Fore.YELLOW + 'WARNING: part ' +
                    part['name']+' has no mass, maybe you should assign a material to it ?' + Style.RESET_ALL)



    return mass,inertia,com

def compute_inertia(matrix,mass,com,inertia):
    # this is taken from addLinkDynamics
    # Inertia
    I = np.matrix(np.reshape(inertia[:9], (3, 3)))
    R = matrix[:3, :3]
    # Expressing COM in the link frame
    com = np.array(
        (matrix*np.matrix([com[0], com[1], com[2], 1]).T).T)[0][:3]

    # Expressing inertia in the link frame
    inertia = R*I*R.T

    return {
        'com': com,
        'inertia': inertia,
        'mass':mass
    }


def get_T_part_mate(matedEntity: dict):
    T_part_mate = np.eye(4)
    T_part_mate[:3, :3] = np.stack(
        (
            np.array(matedEntity["matedCS"]["xAxis"]),
            np.array(matedEntity["matedCS"]["yAxis"]),
            np.array(matedEntity["matedCS"]["zAxis"]),
        )
    ).T
    T_part_mate[:3, 3] = matedEntity["matedCS"]["origin"]

    return T_part_mate

def feature_init(client,fullConfiguration, workspaceId, assemblyId):
    # Load joint features to get limits later
    if config['versionId'] == '':
        joint_features = client.get_features(
            config['documentId'], workspaceId, assemblyId)
    else:
        joint_features = client.get_features(
            config['documentId'], config['versionId'], assemblyId, type='v')

    # Retrieving root configuration parameters
    configuration_parameters = {}
    parts = fullConfiguration.split(';')
    # print(f"init::parts::{parts}")
    for part in parts:
        kv = part.split('=')
        if len(kv) == 2:
            configuration_parameters[kv[0]] = kv[1].replace('+', ' ')
    return configuration_parameters, joint_features

def getLimits(jointType, name,joint_features):

    # print(f"joint_features::{joint_features}")
    # print(f"getLimits::jointType::{jointType}")
    enabled = False
    minimum, maximum = 0, 0
    for feature in joint_features['features']:
        # Find coresponding joint
        if name == feature['message']['name']:
            # Find min and max values
            for parameter in feature['message']['parameters']:
                if parameter['message']['parameterId'] == "limitsEnabled":
                    enabled = parameter['message']['value']

                if jointType == 'revolute':
                    if parameter['message']['parameterId'] == 'limitAxialZMin':
                        minimum = readParameterValue(parameter, name)
                    if parameter['message']['parameterId'] == 'limitAxialZMax':
                        maximum = readParameterValue(parameter, name)
                elif jointType == 'prismatic':
                    if parameter['message']['parameterId'] == 'limitZMin':
                        minimum = readParameterValue(parameter, name)
                    if parameter['message']['parameterId'] == 'limitZMax':
                        maximum = readParameterValue(parameter, name)
    if enabled:
        return (minimum, maximum)
    else:
        if jointType != 'continuous':
            print(Fore.YELLOW + 'WARNING: joint ' + name + ' of type ' +
                jointType + ' has no limits ' + Style.RESET_ALL)
        return None

def readExpression(expression):
    # Expression can itself be a variable from configuration
    # XXX: This doesn't handle all expression, only values and variables
    if expression[0] == '#':
        expression = configuration_parameters[expression[1:]]
    if expression[0:2] == '-#':
        expression = '-'+configuration_parameters[expression[2:]]

    parts = expression.split(' ')

    # Checking the unit, returning only radians and meters
    if parts[1] == 'deg':
        return math.radians(float(parts[0]))
    elif parts[1] in ['radian', 'rad']:
        # looking for PI
        if isinstance(parts[0], str):
            if parts[0] == '(PI)':
                value = math.pi
            else:
                raise ValueError(f"{parts[0]} variable isn't supported")
        else:
            value = parts[0]
        return float(value)
    elif parts[1] == 'mm':
        return float(parts[0])/1000.0
    elif parts[1] == 'cm':
        return float(parts[0])/100.0
    elif parts[1] == 'm':
        return float(parts[0])
    elif parts[1] == 'in':
        return float(parts[0])*0.0254
    else:
        print(Fore.RED + 'Unknown unit: '+parts[1] + Style.RESET_ALL)
        exit()


def readParameterValue(parameter, name):
    # This is an expression
    if parameter['typeName'] == 'BTMParameterNullableQuantity':
        return readExpression(parameter['message']['expression'])
    if parameter['typeName'] == 'BTMParameterConfigured':
        message = parameter['message']
        parameterValue = configuration_parameters[message['configurationParameterId']]

        for value in message['values']:
            if value['typeName'] == 'BTMConfiguredValueByBoolean':
                booleanValue = (parameterValue == 'true')
                if value['message']['booleanValue'] == booleanValue:
                    return readExpression(value['message']['value']['message']['expression'])
            elif value['typeName'] == 'BTMConfiguredValueByEnum':
                if value['message']['enumValue'] == parameterValue:
                    return readExpression(value['message']['value']['message']['expression'])
            else:
                print(Fore.RED+"Can't read value of parameter "+name+" configured with "+value['typeName']+Style.RESET_ALL)
                exit()

        print(Fore.RED+"Could not find the value for "+name+Style.RESET_ALL)
    else:
        print(Fore.RED+'Unknown feature type for '+name+': ' +
              parameter['typeName']+Style.RESET_ALL)
        exit()


####### End: This is copied or modified code ########
def translate_joint_type_to_mjcf(j_type):
    mj_j_type = {
        "revolute":"hinge"
    }
    return mj_j_type[j_type]

def get_joint_limit2(client,joint):
    assembly_info = joint.assemblyInfo
    document = client.get_document(assembly_info['documentId']).json()
    workspaceId = document["defaultWorkspace"]["id"]

    configuration_parameters, joint_features = feature_init(
        client,
        assembly_info['fullConfiguration'],
        workspaceId,
        assembly_info['assemblyId']
        )
    # print(f"get_joint_limit2::joint_features::{joint_features}")

    limit = getLimits(joint.j_type.lower(),joint.name,joint_features)
    return limit


def get_joint_name(joint_name,graph_state:MujocoGraphState):
    if joint_name in graph_state.joint_names.keys():
        graph_state.joint_names[joint_name] +=1
    else:
        graph_state.joint_names[joint_name] = 1
    return joint_name + str(graph_state.joint_names[joint_name])

def get_worldAxisFrame2(part):
    T_world_part = np.matrix(part.transform).reshape(4,4)
    # print(f"MJCF::T_world_part::type::{type(T_world_part)}")
    # print(f"MJCF::T_world_part::\n{T_world_part}")

    # print(f"get_worldAxisFrame2::part.joint.feature['featureData']::{part.joint.feature['featureData']}")
    T_part_mate = get_T_part_mate(part.joint.feature['featureData']['matedEntities'][0])
    # T_world_part = transform
    # print(f"MJCF::T_part_mate::\n{T_part_mate}")
    # print(f"MJCF::T_world_part::{T_world_part}")
    # The problem is T_world_part which is different for URDF
    T_world_mate = T_world_part * T_part_mate
    # T_world_mate = T_world_part * T_part_mate
    worldAxisFrame = T_world_mate

    return worldAxisFrame


def convert_to_snake_case(s):
    s = s.lower()
    s = s.replace(" ", "_")
    return s

def get_color_name(rgb):
    url = f"https://www.thecolorapi.com/id?rgb=rgb({rgb[0]},{rgb[1]},{rgb[2]})"
    response = requests.get(url)
    data = response.json()
    color_name = data['name']['value']
    return convert_to_snake_case(color_name)

def find_occurence(assembly,occurences,instanceId:str):
    # I need to rewrite this so that it considers both part and sub-assembly instance.
    for occ in occurences:
      instance = findInstance(assembly,occ["path"])
      if instance["id"]== instanceId:
        return occ

def find_occurrence(occurences,occurence_path):
  # assuming that that we are dealing with a part that belong to sub assemblies:
  # occurence_path: [part_instance, assembly_instance]
  # assuming that we are in the root assembly
  # occurence path: [part_instance]

  # print(f"occurence_path::{occurence_path}")
  for occ in occurences:
    # print(f"occ['path']::{occ['path']}")
    if occ["path"] == occurence_path:
      return occ


def get_part_relations(relations,instance_id,assemblyInstance):
    children = []
    # print(f"instance_id::{instance_id}")
    # print(f"assemblyInstance::{assemblyInstance}")

    for r in relations:
        # print("\n")
        # print(f"get_part_relations::r['parent'][0]::{r['parent'][0]}")
        # print(f"get_part_relations::r::assemblyInstanceId::{r['assemblyInstanceId']}")
        # print("\n")
        if r['parent'][0] == instance_id :
          if assemblyInstance == None:
            children.append(r)
          elif r['assemblyInstanceId'] == assemblyInstance:
            children.append(r)
    return children

def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def rotationMatrixToQuatAngles(transform):
     # Extract rotation matrix
    R = transform[:3, :3]

    # Compute the trace of the rotation matrix
    trace = np.trace(R)

    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2  # S=4*q_w
        q_w = 0.25 * S
        q_x = (R[2, 1] - R[1, 2]) / S
        q_y = (R[0, 2] - R[2, 0]) / S
        q_z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S=4*q_x
        q_w = (R[2, 1] - R[1, 2]) / S
        q_x = 0.25 * S
        q_y = (R[0, 1] + R[1, 0]) / S
        q_z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S=4*q_y
        q_w = (R[0, 2] - R[2, 0]) / S
        q_x = (R[0, 1] + R[1, 0]) / S
        q_y = 0.25 * S
        q_z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S=4*q_z
        q_w = (R[1, 0] - R[0, 1]) / S
        q_x = (R[0, 2] + R[2, 0]) / S
        q_y = (R[1, 2] + R[2, 1]) / S
        q_z = 0.25 * S

    quaternion = np.array([q_w, q_x, q_y, q_z])

    return quaternion

def pos_form_trasform(transform):
    x = transform[0, 3]
    y = transform[1, 3]
    z = transform[2, 3]

    # print(f"pos_form_trasform::transform::shape::{transform.shape}")

    return [x,y,z]

def transform_to_pos_and_euler(transform):
    rpy = rotationMatrixToEulerAngles(transform)
    xyz = pos_form_trasform(transform)
    quat = rotationMatrixToQuatAngles(transform)
    # print(f"quat::{quat}")
    return xyz,rpy,quat


