#!/usr/bin/env python

import sys, os, math
from xml.dom.minidom import parse, parseString
import xml.dom
import yaml

import tf

reload(sys)
sys.setdefaultencoding('utf-8')

#### >>> copied from xacro/src/xacro.py
# Better pretty printing of xml
# Taken from http://ronrothman.com/public/leftbraned/xml-dom-minidom-toprettyxml-and-silly-whitespace/
def fixed_writexml(self, writer, indent="", addindent="", newl=""):
    # indent = current indentation
    # addindent = indentation to add to higher levels
    # newl = newline string
    writer.write(indent+"<" + self.tagName)

    attrs = self._get_attributes()
    a_names = attrs.keys()
    a_names.sort()

    for a_name in a_names:
        writer.write(" %s=\"" % a_name)
        xml.dom.minidom._write_data(writer, attrs[a_name].value)
        writer.write("\"")
    if self.childNodes:
        if len(self.childNodes) == 1 \
          and self.childNodes[0].nodeType == xml.dom.minidom.Node.TEXT_NODE:
            writer.write(">")
            self.childNodes[0].writexml(writer, "", "", "")
            writer.write("</%s>%s" % (self.tagName, newl))
            return
        writer.write(">%s"%(newl))
        for node in self.childNodes:
            if node.nodeType is not xml.dom.minidom.Node.TEXT_NODE: # 3:
                node.writexml(writer,indent+addindent,addindent,newl) 
                #node.writexml(writer,indent+addindent,addindent,newl)
        writer.write("%s</%s>%s" % (indent,self.tagName,newl))
    else:
        writer.write("/>%s"%(newl))
# replace minidom's function with ours
xml.dom.minidom.Element.writexml = fixed_writexml
#### <<< copied from xacro/src/xacro.py

def search_tagname_id (el, tagname, idname, attr = 'id'):
    ret = None
    for ee in el.getElementsByTagName(tagname):
        if ee.hasAttribute(attr):
            if ee.getAttribute(attr) == idname:
                ret = ee
                break
    return ret

class parseXmlBase:
    doc = None

    def readXmlFile(self, filename):
        self.doc = xml.dom.minidom.parse(filename)
        ret = self.doc.getElementsByTagName('COLLADA')
        if ret:
            ret = parseColladaSensor()
            ret.doc = self.doc
            return ret

        ret = self.doc.getElementsByTagName('robot')
        if ret:
            ret = parseURDFSensor()
            ret.doc = self.doc
            return ret

    def getDocumentString(self, indent = '\t'):
        if self.doc:
            return self.doc.toprettyxml(indent = indent)

    def writeDocument(self, strm, indent = '\t'):
        dstr = self.getDocumentString(indent = indent)
        if dstr:
            strm.write(dstr)
        else:
            sys.stderr.write('no document to write!\n')

    def ListToString (self, lst):
        ret = '%s'%lst.pop(0)
        for l in lst:
            ret = ret + ' %s'%l
        return ret

    def StringToList (self, strin):
        ret = []
        for r in strin.split():
            ret.append(float(r))
        return ret

#
# URDF
#
class parseURDFBase(parseXmlBase):
    objtype = 'urdf'

    def searchRootLink (self):
        return

    def parseTranslate (self, translate):
        if not translate:
            return '0 0 0'
        if isinstance(translate, list):
            translate = self.ListToString(translate)
        return translate

    def parseRotate (self, rotate):
        if not rotate:
            return '0 0 1 0'
        if isinstance(rotate, basestring):
            rotate = self.StringToList(rotate)
        q = tf.transformations.quaternion_about_axis(math.radians(rotate[3]), rotate[:3])
        rpy = tf.transformations.euler_from_quaternion(q)
        return '%f %f %f' % (rpy[0], rpy[1], rpy[2])

class parseURDFSensor(parseURDFBase):
    def init(self):
        return True

    def addLink(self, pos, rpy, parent_link, link_name):
        robot = self.doc.getElementsByTagName("robot")[0]

        pos = self.parseTranslate(pos)
        rpy = self.parseRotate(rpy)

        # creating child joint
        child_joint = self.doc.createElement("joint")
        child_joint.setAttribute("name", link_name + "_joint")
        child_joint.setAttribute("type", "fixed")
        origin_element = self.doc.createElement("origin")
        origin_element.setAttribute("xyz", pos)
        origin_element.setAttribute("rpy", rpy)
        child_joint.appendChild(origin_element)
        parent_element = self.doc.createElement("parent")
        parent_element.setAttribute("link", parent_link)
        child_joint.appendChild(parent_element)
        child_element = self.doc.createElement("child")
        child_element.setAttribute("link", link_name)
        child_joint.appendChild(child_element)

        child_link = self.doc.createElement("link")
        child_link.setAttribute("name", link_name)
        robot.appendChild(child_joint)
        robot.appendChild(child_link)

    def add_sensor (self, name, parent_link, sensor_type, translate = None, rotate = None):
        if not translate:
            translate = '0 0 0'
        if not rotate:
            rotate = '0 0 1 0'
        self.addLink(translate, rotate, parent_link, name)

    def add_manipulator (self, name, origin, tip, translate = None, rotate = None):
        if not translate:
            translate = '0 0 0'
        if not rotate:
            rotate = '0 0 1 0'
        self.addLink(translate, rotate, tip, name)

#
# Collada
#
class parseColladaBase(parseXmlBase):
    objtype = 'collada'

    def searchLinkid (self, linkname):
        tmp_link = ''
        for f in self.doc.getElementsByTagName('library_kinematics_models')[0].getElementsByTagName("technique_common")[0].getElementsByTagName("link"):
            if f.getAttribute("name") == linkname:
                tmp_link = f.getAttribute("sid")
        kmodel_id = self.doc.getElementsByTagName('library_kinematics_models')[0].getElementsByTagName("kinematics_model")[0].getAttribute("id")
        if tmp_link == '':
            sys.stderr.write('link name: %s was not found!\n'%linkname)
        return '%s/%s'%(kmodel_id,tmp_link)

    def parseTranslate (self, translate):
        if isinstance(translate, list):
            translate = self.ListToString(translate)
        return translate

    def parseRotate (self, rotate):
        if isinstance(rotate, list):
            rotate = self.ListToString(rotate)
        return rotate

    def searchRootLink (self):
        return

class parseColladaSensor(parseColladaBase):
    library_sensors_node = None
    target_articulated_system = None

    sensor_id = 1
    force_sensor_id = 0
    gyro_sensor_id = 0
    acc_sensor_id = 0
    cam_sensor_id = 0

    def init (self):
        ##self.readXmlFile(filename)

        plst = self.doc.getElementsByTagName('extra')
        for p in plst:
            if p.hasAttribute('id') and p.getAttribute('id') == 'sensors' and p.hasAttribute('type') and p.getAttribute('type') == 'library_sensors':
                self.library_sensors_node = p

        plst = self.doc.getElementsByTagName('articulated_system')
        for p in plst:
            if p.hasAttribute('id') and p.getAttribute('id').find('_motion') != -1:
                self.target_articulated_system = p

        return self.library_sensors_node != None and self.target_articulated_system != None

    def add_manipulator (self, name, origin, tip, translate = None, rotate = None):
        if self.target_articulated_system == None:
            return

        translate = self.parseTranslate(translate)
        rotate = self.parseRotate(rotate)

        ### add manipulator to articulated system
        ex = self.doc.createElement('extra')
        ex.setAttribute('name', name)
        ex.setAttribute('type', 'manipulator')

        tec = self.doc.createElement('technique')
        tec.setAttribute('profile', 'OpenRAVE')

        ##
        tmp = self.doc.createElement('frame_origin')
        tmp.setAttribute('link', self.searchLinkid(origin))
        tec.appendChild(tmp)

        tmp = self.doc.createElement('frame_tip')
        tmp.setAttribute('link', self.searchLinkid(tip))
        #
        tl = self.doc.createElement('translate')
        if translate == None:
            tl.appendChild(self.doc.createTextNode('0 0 0'))
        else:
            tl.appendChild(self.doc.createTextNode(translate))

        ro = self.doc.createElement('rotate')
        if rotate == None:
            ro.appendChild(self.doc.createTextNode('0 0 1 0'))
        else:
            ro.appendChild(self.doc.createTextNode(rotate))
        tmp.appendChild(tl)
        tmp.appendChild(ro)
        #
        tec.appendChild(tmp)
        ex.appendChild(tec)
        self.target_articulated_system.appendChild(ex)

    def add_sensor (self, name, parent_link, sensor_type, translate = None, rotate = None):
        if self.library_sensors_node == None or self.target_articulated_system == None:
            return

        translate = self.parseTranslate(translate)
        rotate = self.parseRotate(rotate)

        ### add sensor to articulated system
        ex = self.doc.createElement('extra')
        ex.setAttribute('name', name)
        ex.setAttribute('type', 'attach_sensor')

        tec = self.doc.createElement('technique')
        tec.setAttribute('profile', 'OpenRAVE')

        tmp = self.doc.createElement('instance_sensor')
        tmp.setAttribute('url', '#sensor%d' % self.sensor_id)
        self.sensor_id = self.sensor_id + 1
        tec.appendChild(tmp)

        tmp = self.doc.createElement('frame_origin')
        tmp_parent_link = ''
        for f in self.doc.getElementsByTagName('library_kinematics_models')[0].getElementsByTagName("technique_common")[0].getElementsByTagName("link"):
            if f.getAttribute("name") == parent_link:
                tmp_parent_link = f.getAttribute("sid")
        kmodel_id = self.doc.getElementsByTagName('library_kinematics_models')[0].getElementsByTagName("kinematics_model")[0].getAttribute("id")
        if tmp_parent_link == '':
            sys.stderr.write('parent_link: %s was not found!\n'%parent_link)

        tmp.setAttribute('link', '%s/%s'%(kmodel_id,tmp_parent_link))

        tl = self.doc.createElement('translate')
        if translate == None:
            tl.appendChild(self.doc.createTextNode('0 0 0'))
        else:
            tl.appendChild(self.doc.createTextNode(translate))

        ro = self.doc.createElement('rotate')
        if rotate == None:
            ro.appendChild(self.doc.createTextNode('0 0 1 0'))
        else:
            ro.appendChild(self.doc.createTextNode(rotate))

        tmp.appendChild(tl)
        tmp.appendChild(ro)

        tec.appendChild(tmp)
        ex.appendChild(tec)
        self.target_articulated_system.appendChild(ex)

        ### add to library_sensors
        plst = self.library_sensors_node.getElementsByTagName('technique')
        targetNode = None
        for p in plst:
            if p.hasAttribute('profile') and p.getAttribute('profile') == 'OpenRAVE':
                targetNode = p

        if targetNode != None:
            sen = self.doc.createElement('sensor')
            sen.setAttribute('id', 'sensor%d' % (self.sensor_id - 1))

            if sensor_type == 'force' or sensor_type == 'base_force6d':
                sen.setAttribute('type', 'base_force6d')
                sen.setAttribute('sid', '%d'%self.force_sensor_id)
                self.force_sensor_id = self.force_sensor_id + 1
                lf = self.doc.createElement('load_range_force')
                lf.appendChild(self.doc.createTextNode('-1.0 -1.0 -1.0'))
                lt = self.doc.createElement('load_range_torque')
                lt.appendChild(self.doc.createTextNode('-1.0 -1.0 -1.0'))
                sen.appendChild(lf)
                sen.appendChild(lt)
            elif sensor_type == 'acceleration' or sensor_type == 'base_imu':
                sen.setAttribute('type', 'base_imu')
                sen.setAttribute('sid', '%d'%self.acc_sensor_id)
                self.acc_sensor_id = self.acc_sensor_id + 1
                max_acc = self.doc.createElement('max_acceleration')
                max_acc.appendChild(self.doc.createTextNode('-1.0 -1.0 -1.0'))
                sen.appendChild(max_acc)
            elif sensor_type == 'gyro':
                sen.setAttribute('type', 'base_imu')
                sen.setAttribute('sid', '%d'%self.gyro_sensor_id)
                self.gyro_sensor_id = self.gyro_sensor_id + 1
                max_ang = self.doc.createElement('max_angular_velocity')
                max_ang.appendChild(self.doc.createTextNode('-1.0 -1.0 -1.0'))
                sen.appendChild(max_ang)
            elif sensor_type == 'camera' or sensor_type == 'base_pinhole_camera':
                sen.setAttribute('type', 'base_pinhole_camera')
                sen.setAttribute('sid', '%d'%self.cam_sensor_id)
                self.cam_sensor_id = self.cam_sensor_id + 1
                #img_dim = self.doc.createElement('image_dimensions')
                #img_dim.appendChild()
                #sen.appendChild(img_dim)
                #child_elem = self.doc.createElement('format')
                #child_elem = self.doc.createElement('measurement_time')
                #child_elem = self.doc.createElement('intrinsic')
                #child_elem = self.doc.createElement('focal_length')
                #child_elem = self.doc.createElement('distortion_model')

            targetNode.appendChild(sen)

class replaceLibraryNode(parseColladaBase):

    def init(self, filename):
        self.readXmlFile(filename)

        llst = self.doc.getElementsByTagName('library_nodes')
        if len(llst) == 0:
            sys.stderr.write('This file does not have library_nodes. Do nothing.\n')
            return False

        nlst = self.doc.getElementsByTagName('instance_node')
        if len(nlst) > 1:
            sys.stderr.write('instance node is more than 1.\n')
            return False

        i_node = nlst[0]
        a_url = None
        if(i_node.hasAttribute('url')):
            a_url = i_node.getAttribute('url')

        if not(a_url):
            sys.stderr.write('instance node does not have url\n')
            return False

        a_url = a_url[1:] ## remove first #

        nd = None
        for ll in llst:
            nd = search_tagname_id (ll, 'node', a_url)
            if nd:
                break

        if not(nd):
            sys.stderr.write('node which has id as %s not found\n'%a_url)
            return False

        p_node = i_node.parentNode
        p_node.replaceChild (nd, i_node)
        #p_node.removeChild (i_node)
        #p_node.appendChild(nd.cloneNode(True))

        return True

class yamlParser:
    yaml_data = None

    def load(self, fname):
        self.yaml_data = yaml.load(open(fname).read())

    def add_sensor(self, xml_obj):
        if 'sensors' in self.yaml_data and self.yaml_data['sensors']:
            for sensor in self.yaml_data['sensors']:
                translate = sensor['translate'] if sensor.has_key('translate') else None
                rotate = sensor['rotate'] if sensor.has_key('rotate') else None
                xml_obj.add_sensor(sensor['sensor_name'],
                                   sensor['parent_link'],
                                   sensor['sensor_type'],
                                   translate = translate, rotate = rotate)

    def add_eef(self, xml_obj):
        for limb in ['rleg', 'lleg', 'rarm', 'larm', 'head', 'torso']:
            eff_name = '%s-end-coords'%limb
            eff = self.yaml_data[eff_name] if eff_name in self.yaml_data else None
            if eff:
                translate = eff['translate'] if 'translate' in eff else None
                rotate = eff['rotate'] if 'rotate' in eff else None
                parent  = eff['parent'] if 'parent' in eff else None
                root  = eff['root'] if 'root' in eff else 'BODY'
                if not parent:
                    if not self.yaml_data.has_key(limb):
                        print >>sys.stderr, "cannot find limb: %s" %(limb)
                        return
                    limb_lst = self.yaml_data[limb]
                    parent = limb_lst[-1].keys()[0].replace("JOINT", "LINK") # not goood!
                xml_obj.add_manipulator('%s_end_coords'%limb, root, parent,
                                        translate = translate, rotate = rotate)

    def add_links(self, xml_obj):
        if xml_obj.objtype == 'urdf' and 'links' in self.yaml_data:
            for sensor in self.yaml_data['links']:
                translate = sensor['translate'] if sensor.has_key('translate') else None
                rotate = sensor['rotate'] if sensor.has_key('rotate') else None
                xml_obj.add_sensor(sensor['name'],
                                   sensor['parent_link'],
                                   None,
                                   translate = translate, rotate = rotate)
    def replace_xmls(self, xml_obj):
        if xml_obj.objtype == 'urdf' and 'replace_xmls' in self.yaml_data:
            for replace_xml in self.yaml_data['replace_xmls']:
                match_rule = replace_xml['match_rule']
                target_tags = []
                if match_rule.has_key('tag'):
                    tags = xml_obj.doc.getElementsByTagName(match_rule['tag'])
                    if match_rule.has_key('attribute_name'):
                        attribute_name = str(match_rule['attribute_name'])
                        attribute_value = str(match_rule['attribute_value'])
                        matched_tags = [tag for tag in tags if tag.getAttribute(attribute_name) == attribute_value]
                        target_tags = target_tags + matched_tags
                    elif match_rule.has_key('sub_attribute_name'):
                        sub_attribute_name = str(match_rule['sub_attribute_name'])
                        sub_attribute_value = str(match_rule['sub_attribute_value'])
                        for tag in tags:
                            for sub_tag in tag.getElementsByTagName(match_rule['sub_tag']):
                                if sub_tag.getAttribute(sub_attribute_name) == sub_attribute_value:
                                    target_tags.append(tag)
                    elif match_rule.has_key('parent_attribute_name'):
                        parent_attribute_name = str(match_rule['parent_attribute_name'])
                        parent_attribute_value = str(match_rule['parent_attribute_value'])
                        parent_depth = 1
                        if match_rule.has_key("parent_depth"):
                            parent_depth = match_rule["parent_depth"]
                        
                        target_tags.extend([tag for tag in tags
                                            if getParentNode(tag, parent_depth).getAttribute(parent_attribute_name) == parent_attribute_value])
                else:
                    raise Exception("yaml does not have tag section")
                if len(target_tags) > 0:
                    for tag in target_tags:
                        parent = tag.parentNode
                        # remove the tag
                        if replace_xml.has_key('replaced_xml'):
                            parent.removeChild(tag)
                            parent.appendChild(parseString(str(replace_xml['replaced_xml'])).documentElement)
                        elif replace_xml.has_key('replaced_attribute_value'):
                            tag.setAttribute(str(match_rule['attribute_name']),
                                             str(replace_xml['replaced_attribute_value']))
                        else:
                            raise Exception("No rule to replacement is specified")
def getParentNode(node, depth):
    if depth == 0:
        return node
    else:
        return getParentNode(node.parentNode, depth - 1)
            
