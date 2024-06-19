import os
import rospkg

package_name = 'mik_ros_utils'
package_path = os.path.join(os.path.dirname(os.path.abspath(__file__)).split(f'/{package_name}')[0], package_name)
meshes_path = os.path.join(package_path, 'meshes')
config_path = os.path.join(package_path, 'config')
data_path = os.path.join(package_path, 'data')

XML_PATH = os.path.join(package_path, 'xmls')


def find_package_path(package_name):
    rospack = rospkg.RosPack()
    package_path = rospack.get_path(package_name)
    return package_path


def get_package_config_path(package_name):
    package_path = find_package_path(package_name)
    config_path = os.path.join(package_path, 'config')
    return config_path


def get_mesh_path(mesh_name):
    mesh_path = os.path.join(meshes_path, mesh_name)
    return mesh_path


def get_xml_path(xml_name):
    xml_path = os.path.join(XML_PATH, xml_name)
    return xml_path


def get_saved_solution_path(solution_name):
    solution_path = os.path.join(data_path, 'solution', '{}.npy'.format(solution_name))
    return solution_path



