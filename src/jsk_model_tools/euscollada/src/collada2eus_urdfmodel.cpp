/* Author: Yohei Kakiuchi */
#include <ros/ros.h>
#include "collada_parser/collada_parser.h"
#include "urdf/model.h"
#include "urdf_parser/urdf_parser.h"

#include <sys/utsname.h>
#include <math.h>

// assimp_devel
#include <Importer.hpp>
#include <scene.h>
#include <postprocess.h>
#include <IOStream.hpp>
#include <IOSystem.hpp>

#include <iostream>
#include <fstream>
#include <algorithm>


#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/foreach.hpp>

#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>

#include <resource_retriever/retriever.h>

#include "yaml-cpp/yaml.h"

extern "C" {
#ifdef __APPLE__
#include <libqhull/qhull_a.h>
#else
#include <qhull/qhull_a.h>
#endif
}

#include "rospack/rospack.h"

// using collada just for parsing sensors
#include "dae.h"
#include "dom/domCOLLADA.h"
#ifdef __dom150COLLADA_h__
using namespace ColladaDOM150;
#endif

// copy from rviz/src/rviz/mesh_loader.cpp
class ResourceIOStream : public Assimp::IOStream
{
public:
  ResourceIOStream(const resource_retriever::MemoryResource& res)
  : res_(res)
  , pos_(res.data.get())
  {}

  ~ResourceIOStream()
  {}

  size_t Read(void* buffer, size_t size, size_t count)
  {
    size_t to_read = size * count;
    if (pos_ + to_read > res_.data.get() + res_.size)
    {
      to_read = res_.size - (pos_ - res_.data.get());
    }

    memcpy(buffer, pos_, to_read);
    pos_ += to_read;

    return to_read;
  }

  size_t Write( const void* buffer, size_t size, size_t count) { ROS_BREAK(); return 0; }

  aiReturn Seek( size_t offset, aiOrigin origin)
  {
    uint8_t* new_pos = 0;
    switch (origin)
    {
    case aiOrigin_SET:
      new_pos = res_.data.get() + offset;
      break;
    case aiOrigin_CUR:
      new_pos = pos_ + offset; // TODO is this right?  can offset really not be negative
      break;
    case aiOrigin_END:
      new_pos = res_.data.get() + res_.size - offset; // TODO is this right?
      break;
    default:
      ROS_BREAK();
    }

    if (new_pos < res_.data.get() || new_pos > res_.data.get() + res_.size)
    {
      return aiReturn_FAILURE;
    }

    pos_ = new_pos;
    return aiReturn_SUCCESS;
  }

  size_t Tell() const
  {
    return pos_ - res_.data.get();
  }

  size_t FileSize() const
  {
    return res_.size;
  }

  void Flush() {}

private:
  resource_retriever::MemoryResource res_;
  uint8_t* pos_;
};

class ResourceIOSystem : public Assimp::IOSystem
{
public:
  ResourceIOSystem()
  {
  }

  ~ResourceIOSystem()
  {
  }

  // Check whether a specific file exists
  bool Exists(const char* file) const
  {
    // Ugly -- two retrievals where there should be one (Exists + Open)
    // resource_retriever needs a way of checking for existence
    // TODO: cache this
    resource_retriever::MemoryResource res;
    try
    {
      res = retriever_.get(file);
    }
    catch (resource_retriever::Exception& e)
    {
      return false;
    }

    return true;
  }

  // Get the path delimiter character we'd like to see
  char getOsSeparator() const
  {
    return '/';
  }

  // ... and finally a method to open a custom stream
  Assimp::IOStream* Open(const char* file, const char* mode = "rb")
  {
    ROS_ASSERT(mode == std::string("r") || mode == std::string("rb"));

    // Ugly -- two retrievals where there should be one (Exists + Open)
    // resource_retriever needs a way of checking for existence
    resource_retriever::MemoryResource res;
    try
    {
      res = retriever_.get(file);
    }
    catch (resource_retriever::Exception& e)
    {
      return 0;
    }

    return new ResourceIOStream(res);
  }

  void Close(Assimp::IOStream* stream);

private:
  mutable resource_retriever::Retriever retriever_;
};

void ResourceIOSystem::Close(Assimp::IOStream* stream)
{
  delete stream;
}

////
using namespace urdf;
using namespace std;

#define FLOAT_PRECISION_FINE   "%.16e"
#define FLOAT_PRECISION_COARSE "%.3f"
//
class ModelEuslisp {

public:
  ModelEuslisp () { } ;
#if URDFDOM_1_0_0_API
  ModelEuslisp (ModelInterfaceSharedPtr r);
#else
  ModelEuslisp (boost::shared_ptr<ModelInterface> r);
#endif
  ~ModelEuslisp ();

  // accessor
  void setRobotName (string &name) { arobot_name = name; };
  void setUseCollision(bool &b) { use_collision = b; };
  void setUseSimpleGeometry(bool &b) { use_simple_geometry = b; };
  void setUseLoadbleMesh(bool &b) { use_loadable_mesh = b; };
  void setRemoveNormal(bool &b) { remove_normal = b; };
  void setAddJointSuffix(bool &b) { add_joint_suffix = b; };
  void setAddLinkSuffix(bool &b) { add_link_suffix = b; };
  void setAddSensorSuffix(bool &b) { add_sensor_suffix = b; };

  // methods for parsing robot model for euslisp
  void addLinkCoords();
  void printMesh(const aiScene* scene, const aiNode* node, const Vector3 &scale,
                 const string &material_name, vector<coordT> &store_pt, bool printq);
  void readYaml(string &config_file);

#if URDFDOM_1_0_0_API
  Pose getLinkPose(LinkConstSharedPtr link) {
#else
  Pose getLinkPose(boost::shared_ptr<const Link> link) {
#endif
    if (!link->parent_joint) {
      Pose ret;
      return ret;
    }
    Pose p_pose = getLinkPose(link->getParent());
    Pose l_pose = link->parent_joint->parent_to_joint_origin_transform;
    Pose ret;
    ret.rotation = p_pose.rotation * l_pose.rotation;
    ret.position = (p_pose.rotation * l_pose.position) +  p_pose.position;

    return ret;
  }

#if URDFDOM_1_0_0_API
  void printLink (LinkConstSharedPtr Link, Pose &pose);
  void printJoint (JointConstSharedPtr joint);
  void printGeometry (GeometrySharedPtr g, const Pose &pose,
                      const string &name, const string &material_name);
#else
  void printLink (boost::shared_ptr<const Link> Link, Pose &pose);
  void printJoint (boost::shared_ptr<const Joint> joint);
  void printGeometry (boost::shared_ptr<Geometry> g, const Pose &pose,
                      const string &name, const string &material_name);
#endif
  void printLinks ();
  void printJoints ();
  void printEndCoords();
  void printSensors();
  void printSensorLists();
  void printGeometries();

  // print methods
  void copyRobotClassDefinition ();
  void printRobotDefinition();
  void printRobotMethods();

  void writeToFile (string &filename);

  // temporary using collada file directly for sensors
  string collada_file;
  daeDocument *g_document;
  //boost::shared_ptr< DAE> dae;
  DAE dae;
  string getSensorType (const domExtraRef pextra);
  domLink* findLinkfromKinematics (domLink* thisLink, const string& link_name);
  void parseSensors();
  class daeSensor {
  public:
    string sensor_type;
    string parent_link;
    string sensor_id;
    string name;
    vector<daeElementRef> ptrans;

    static bool compare(const daeSensor& a, const daeSensor& b) {
      return (a.sensor_id < b.sensor_id);
    }
  };
  vector<daeSensor> m_sensors;

private:
#if URDFDOM_1_0_0_API
  typedef map <string, VisualConstSharedPtr> MapVisual;
  typedef map <string, CollisionConstSharedPtr> MapCollision;
#else
  typedef map <string, boost::shared_ptr<const Visual> > MapVisual;
  typedef map <string, boost::shared_ptr<const Collision> > MapCollision;
#endif
  typedef pair<vector<string>, vector<string> > link_joint;
  typedef pair<string, link_joint > link_joint_pair;

#if URDFDOM_1_0_0_API
  ModelInterfaceSharedPtr robot;
#else
  boost::shared_ptr<ModelInterface> robot;
#endif
  string arobot_name;
#if URDFDOM_1_0_0_API
  map <LinkConstSharedPtr, Pose > m_link_coords;
  map <LinkConstSharedPtr, MapVisual > m_link_visual;
  map <LinkConstSharedPtr, MapCollision > m_link_collision;
  map <string, MaterialConstSharedPtr> m_materials;
#else
  map <boost::shared_ptr<const Link>, Pose > m_link_coords;
  map <boost::shared_ptr<const Link>, MapVisual > m_link_visual;
  map <boost::shared_ptr<const Link>, MapCollision > m_link_collision;
  map <string, boost::shared_ptr<const Material> > m_materials;
#endif
  vector<pair<string, string> > g_all_link_names;
  vector<link_joint_pair> limbs;

  FILE *fp;
  YAML::Node doc;

  //
  bool add_joint_suffix;
  bool add_link_suffix;
  bool add_sensor_suffix;
  bool use_simple_geometry;
  bool use_collision;
  bool use_loadable_mesh;
  bool remove_normal;

};

#if URDFDOM_1_0_0_API
ModelEuslisp::ModelEuslisp (ModelInterfaceSharedPtr r) {
#else
ModelEuslisp::ModelEuslisp (boost::shared_ptr<ModelInterface> r) {
#endif
  robot = r;
  add_joint_suffix = true;
  add_link_suffix = true;
  add_sensor_suffix = false;
  use_simple_geometry = false;
  use_collision = false;
  use_loadable_mesh = false;
  remove_normal = false;
}

ModelEuslisp::~ModelEuslisp () {

}

void ModelEuslisp::addLinkCoords() {
#if URDFDOM_1_0_0_API
  for (map<string, LinkSharedPtr>::iterator link = robot->links_.begin();
#else
  for (map<string, boost::shared_ptr<Link> >::iterator link = robot->links_.begin();
#endif
       link != robot->links_.end(); link++) {
    Pose p = getLinkPose(link->second);
    m_link_coords.insert
#if URDFDOM_1_0_0_API
      (map<LinkConstSharedPtr, Pose >::value_type (link->second, p));
#else
      (map<boost::shared_ptr<const Link>, Pose >::value_type (link->second, p));
#endif
#if DEBUG
    cerr << "name: " << link->first;
    cerr << ", #f(" << p.position.x << " ";
    cerr << p.position.y << " ";
    cerr << p.position.z << ") #f(";
    cerr << p.rotation.w << " ";
    cerr << p.rotation.x << " ";
    cerr << p.rotation.y << " ";
    cerr << p.rotation.z << ")" << endl;
#endif
    if (use_collision) {
      int counter = 0;
      if(link->second->collision_array.size() > 0) {
        MapCollision mc;
#if URDFDOM_1_0_0_API
        for (vector<CollisionSharedPtr >::iterator it = link->second->collision_array.begin();
#else
        for (vector<boost::shared_ptr <Collision> >::iterator it = link->second->collision_array.begin();
#endif
             it != link->second->collision_array.end(); it++) {
          stringstream ss;
          ss << link->second->name << "_geom" << counter;
          mc.insert(MapCollision::value_type (ss.str(), (*it)));
          counter++;
        }
        m_link_collision.insert
#if URDFDOM_1_0_0_API
          (map <LinkConstSharedPtr, MapCollision >::value_type
#else
          (map <boost::shared_ptr<const Link>, MapCollision >::value_type
#endif
           (link->second, mc));
      } else if(!!link->second->collision) {
        MapCollision mc;
        string gname(link->second->name);
        gname += "_geom0";
        mc.insert(MapCollision::value_type (gname, link->second->collision));
        m_link_collision.insert
#if URDFDOM_1_0_0_API
          (map <LinkConstSharedPtr, MapCollision >::value_type
#else
          (map <boost::shared_ptr<const Link>, MapCollision >::value_type
#endif
           (link->second, mc));
      }
    } else {
      int counter = 0;
      if(link->second->visual_array.size() > 0) {
        MapVisual mv;
#if URDFDOM_1_0_0_API
        for (vector<VisualSharedPtr>::iterator it = link->second->visual_array.begin();
#else
        for (vector<boost::shared_ptr <Visual> >::iterator it = link->second->visual_array.begin();
#endif
             it != link->second->visual_array.end(); it++) {
          m_materials.insert
#if URDFDOM_1_0_0_API
            (map <string, MaterialConstSharedPtr>::value_type ((*it)->material_name, (*it)->material));
#else
            (map <string, boost::shared_ptr<const Material> >::value_type ((*it)->material_name, (*it)->material));
#endif
          stringstream ss;
          ss << link->second->name << "_geom" << counter;
          mv.insert(MapVisual::value_type (ss.str(), (*it)));
          counter++;
        }
        m_link_visual.insert
#if URDFDOM_1_0_0_API
          (map <LinkConstSharedPtr, MapVisual >::value_type
#else
          (map <boost::shared_ptr<const Link>, MapVisual >::value_type
#endif
           (link->second, mv));
      } else if(!!link->second->visual) {
        m_materials.insert
#if URDFDOM_1_0_0_API
          (map <string, MaterialConstSharedPtr>::value_type
#else
          (map <string, boost::shared_ptr<const Material> >::value_type
#endif
           (link->second->visual->material_name, link->second->visual->material));
        MapVisual mv;
        string gname(link->second->name);
        gname += "_geom0";
        mv.insert(MapVisual::value_type (gname, link->second->visual));
        m_link_visual.insert
#if URDFDOM_1_0_0_API
          (map <LinkConstSharedPtr, MapVisual >::value_type
#else
          (map <boost::shared_ptr<const Link>, MapVisual >::value_type
#endif
           (link->second, mv));
      }
    }
  }
}

void ModelEuslisp::printMesh(const aiScene* scene, const aiNode* node, const Vector3 &scale,
                             const string &material_name, vector<coordT> &store_pt, bool printq) {
  aiMatrix4x4 transform = node->mTransformation;
  aiNode *pnode = node->mParent;
  while (pnode)  {
    if (pnode->mParent != NULL) {
      transform = pnode->mTransformation * transform;
    }
    pnode = pnode->mParent;
  }

  aiMatrix3x3 rotation(transform);
  aiMatrix3x3 inverse_transpose_rotation(rotation);
  inverse_transpose_rotation.Inverse();
  inverse_transpose_rotation.Transpose();
  for (uint32_t i = 0; i < node->mNumMeshes; i++) {
    aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
    if (input_mesh->mPrimitiveTypes != aiPrimitiveType_TRIANGLE) {
      continue; // print only triangle mesh
    }
    if (printq) fprintf(fp, "                  (list ;; mesh description\n");
    if (printq) fprintf(fp, "                   (list :type :triangles)\n");
    if (printq) fprintf(fp, "                   (list :material (list");
    if (material_name.size() > 0) {
      if (printq) fprintf(fp, ";; material: %s\n", material_name.c_str());
#if URDFDOM_1_0_0_API
      map <string, MaterialConstSharedPtr>::iterator it = m_materials.find(material_name);
#else
      map <string, boost::shared_ptr<const Material> >::iterator it = m_materials.find(material_name);
#endif
      if (it != m_materials.end()) {

#if URDFDOM_1_0_0_API
        MaterialConstSharedPtr m = it->second;
#else
        boost::shared_ptr<const Material> m = it->second;
#endif
        float col_r = m->color.r;
        float col_g = m->color.g;
        float col_b = m->color.b;
        float col_a = m->color.a;
        if (printq) fprintf(fp, "\n                    (list :ambient (float-vector %f %f %f %f))", col_r, col_g, col_b, col_a);
        if (printq) fprintf(fp, "\n                    (list :diffuse (float-vector %f %f %f %f))", col_r, col_g, col_b, col_a);
      } else {
        std::cerr << "can not find material " << material_name << std::endl;
      }
    } else {
      if (!!scene->mMaterials) {
        aiMaterial *am = scene->mMaterials[input_mesh->mMaterialIndex];
        aiReturn ar;
        aiColor4D clr4d( 0.0, 0.0, 0.0, 0.0);
        ar = am->Get (AI_MATKEY_COLOR_AMBIENT, clr4d);
        if (ar == aiReturn_SUCCESS) {
          if (printq) fprintf(fp, "\n                    (list :ambient (float-vector %f %f %f %f))",
                  clr4d[0], clr4d[1], clr4d[2], clr4d[3]);
        }
        ar = am->Get (AI_MATKEY_COLOR_DIFFUSE, clr4d);
        if (ar == aiReturn_SUCCESS) {
          if (printq) fprintf(fp, "\n                    (list :diffuse (float-vector %f %f %f %f))",
                  clr4d[0], clr4d[1], clr4d[2], clr4d[3]);
        }
        // ar = am->Get (AI_MATKEY_COLOR_SPECULAR, clr4d);
        // ar = am->Get (AI_MATKEY_COLOR_EMISSIVE, clr4d);
        // float val;
        // ar = am->Get (AI_MATKEY_SHININESS, val);
      }
    }
    if (printq) fprintf(fp, "))\n");

    if (printq) fprintf(fp, "                   (list :indices #i(");
    for (uint32_t j = 0; j < input_mesh->mNumFaces; j++) {
      aiFace& face = input_mesh->mFaces[j];
      for (uint32_t k = 0; k < face.mNumIndices; ++k) {
        if (printq) fprintf(fp, " %d", face.mIndices[k]);
        aiVector3D p = input_mesh->mVertices[face.mIndices[k]];
        p *= transform;
        store_pt.push_back(p.x * scale.x);
        store_pt.push_back(p.y * scale.y);
        store_pt.push_back(p.z * scale.z);
      }
    }
    if (printq) fprintf(fp, "))\n");

    if (printq) fprintf(fp, "                   (list :vertices (let ((mat (make-matrix %d 3))) (fvector-replace (array-entity mat) #f(", input_mesh->mNumVertices);
    // Add the vertices
    for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)  {
      aiVector3D p = input_mesh->mVertices[j];
      p *= transform;
      //p *= scale;
      if (printq) fprintf(fp, FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" ",
                          1000 * p.x * scale.x, 1000 * p.y * scale.y, 1000 * p.z * scale.z);
    }
    if (printq) fprintf(fp, ")) mat))");

    if (input_mesh->HasNormals()) {
      if (printq) fprintf(fp, "\n");
      if (printq) fprintf(fp, "                   (list :normals (let ((mat (make-matrix %d 3))) (fvector-replace (array-entity mat) #f(", input_mesh->mNumVertices);
      for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)  {
        if (isnan(input_mesh->mNormals[j].x) ||
            isnan(input_mesh->mNormals[j].y) ||
            isnan(input_mesh->mNormals[j].z)) {
          if (printq) fprintf(fp, " 0 0 0"); // should be normalized vector #f(1 0 0) ???
        } else {
          aiVector3D n = inverse_transpose_rotation * input_mesh->mNormals[j];
          n.Normalize();
          if (printq) fprintf(fp, FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" ", n.x, n.y, n.z);
        }
      }
      if (printq) fprintf(fp, ")) mat))");
    }
#if 0
    if (input_mesh->HasTextureCoords(0)) {
      for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)  {
        aiVector3D n = inverse_transpose_rotation * input_mesh->mNormals[j];
        //*vertices++ = input_mesh->mTextureCoords[0][j].x;
        //*vertices++ = input_mesh->mTextureCoords[0][j].y;
      }
    }
#endif
    if (printq) fprintf(fp, ")\n");
  }
  for (uint32_t i = 0; i < node->mNumChildren; ++i) {
    printMesh(scene, node->mChildren[i], scale, material_name, store_pt, printq);
  }
}

bool limb_order_asc(const pair<string, size_t>& left, const pair<string, size_t>& right) { return left.second < right.second; }
void ModelEuslisp::readYaml (string &config_file) {
  // read yaml
  string limb_candidates[] = {"torso", "larm", "rarm", "lleg", "rleg", "head"}; // candidates of limb names

  vector<pair<string, size_t> > limb_order;
#ifndef USE_CURRENT_YAML
  ifstream fin(config_file.c_str());
  if (fin.fail()) {
    fprintf(stderr, "%c[31m;; Could not open %s%c[m\n", 0x1b, config_file.c_str(), 0x1b);
  } else {
    YAML::Parser parser(fin);
    parser.GetNextDocument(doc);

#else
  {
    // yaml-cpp is greater than 0.5.0
    doc = YAML::LoadFile(config_file.c_str());
#endif
    /* re-order limb name by lines of yaml */
    BOOST_FOREACH(string& limb, limb_candidates) {
#ifdef USE_CURRENT_YAML
      if (doc[limb]) {
        int line=0;
        ifstream fin2(config_file.c_str()); // super ugry hack until yaml-cpp 0.5.2
        string buffer;
        for (;fin2;) {
          getline(fin2, buffer); line++;
          if(buffer == limb+":") break;
        }
        fin2.close();
        std::cerr << limb << "@" << line << std::endl;
        limb_order.push_back(pair<string, size_t>(limb, line));
      }
#else
      if ( doc.FindValue(limb) ) {
        cerr << limb << "@" << doc[limb].GetMark().line << endl;
        limb_order.push_back(pair<string, size_t> (limb, doc[limb].GetMark().line));
      }
#endif
    }
    sort(limb_order.begin(), limb_order.end(), limb_order_asc);
  }

  // generate limbs including limb_name, link_names, and joint_names
  for (size_t i = 0; i < limb_order.size(); i++) {
    string limb_name = limb_order[i].first;
    vector<string> tmp_link_names, tmp_joint_names;
    try {
      const YAML::Node& limb_doc = doc[limb_name];
      for(unsigned int i = 0; i < limb_doc.size(); i++) {
        const YAML::Node& n = limb_doc[i];
#ifdef USE_CURRENT_YAML
        for(YAML::const_iterator it=n.begin();it!=n.end();it++) {
          string key, value;
          key = it->first.as<std::string>();
          value = it->second.as<std::string>();
#else
        for(YAML::Iterator it=n.begin();it!=n.end();it++) {
          string key, value; it.first() >> key; it.second() >> value;
#endif
          tmp_joint_names.push_back(key);
#if URDFDOM_1_0_0_API
          JointConstSharedPtr jnt = robot->getJoint(key);
#else
          boost::shared_ptr<const Joint> jnt = robot->getJoint(key);
#endif
          if (!!jnt) {
            tmp_link_names.push_back(jnt->child_link_name);
          } else {
            // error
          }
          g_all_link_names.push_back(pair<string, string>(key, value));
        }
      }
      limbs.push_back(link_joint_pair(limb_name, link_joint(tmp_link_names, tmp_joint_names)));
    } catch(YAML::RepresentationException& e) {
    }
  }
}

void ModelEuslisp::copyRobotClassDefinition () {
  // load class definition
  fprintf(fp, ";;\n");
  fprintf(fp, ";; copy euscollada-robot class definition from euscollada/src/euscollada-robot.l\n");
  fprintf(fp, ";;\n");
  try {
    string euscollada_path;

    rospack::Rospack rp;
    vector<string> search_path;
    rp.getSearchPathFromEnv(search_path);
    rp.crawl(search_path, 1);
    string path;
    rp.find("euscollada",euscollada_path);

    euscollada_path += "/src/euscollada-robot.l";
    ifstream fin(euscollada_path.c_str());
    string buf;
    while(fin && getline(fin, buf)) {
      fprintf(fp, "%s\n", buf.c_str());
    }

    fprintf(fp, ";;\n");
    fprintf(fp, ";; end of copy from %s\n", euscollada_path.c_str());
    fprintf(fp, ";;\n");
  } catch (runtime_error &e) {
    cerr << "cannot resolve euscollada package path" << endl;
  }
}

void ModelEuslisp::printRobotDefinition() {
  fprintf(fp, "(defun %s () (setq *%s* (instance %s-robot :init)))\n",
          arobot_name.c_str(), arobot_name.c_str(), arobot_name.c_str());
  fprintf(fp, "\n");
  fprintf(fp, "(defclass %s-robot\n", arobot_name.c_str());
  fprintf(fp, "  :super euscollada-robot\n");
  fprintf(fp, "  :slots ( ;; link names\n");
  fprintf(fp, "         ");
#if URDFDOM_1_0_0_API
  for (map<string, LinkSharedPtr>::iterator link = robot->links_.begin();
#else
  for (map<string, boost::shared_ptr<Link> >::iterator link = robot->links_.begin();
#endif
       link != robot->links_.end(); link++) {
    if (add_link_suffix) {
      fprintf(fp," %s_lk", link->second->name.c_str());
    } else {
      fprintf(fp," %s", link->second->name.c_str());
    }
  }
  fprintf(fp, "\n         ;; joint names\n");
  fprintf(fp, "         ");
#if URDFDOM_1_0_0_API
  for (map<string, JointSharedPtr>::iterator joint = robot->joints_.begin();
#else
  for (map<string, boost::shared_ptr<Joint> >::iterator joint = robot->joints_.begin();
#endif
       joint != robot->joints_.end(); joint++) {
    if(add_joint_suffix) {
      if (joint->second->type == Joint::FIXED) {
        fprintf(fp, " %s_fixed_jt", joint->second->name.c_str());
      } else {
        fprintf(fp, " %s_jt", joint->second->name.c_str());
      }
    } else {
      fprintf(fp, " %s", joint->second->name.c_str());
    }
  }
  fprintf(fp, "\n         ;; sensor names\n");
  fprintf(fp, "         ");
  for (vector<daeSensor>::iterator it = m_sensors.begin(); it != m_sensors.end(); it++) {
    fprintf(fp, " %s-sensor-coords", it->name.c_str());
  }
  fprintf(fp, "\n         )\n  )\n");
  // TODO: add openrave manipulator tip frame
}

void ModelEuslisp::printRobotMethods() {
  if (use_loadable_mesh) {
    fprintf(fp, ";;\n(load \"package://eus_assimp/euslisp/eus-assimp.l\") ;; for loadable mesh\n;;\n\n");
  }
  fprintf(fp, "(defmethod %s-robot\n", arobot_name.c_str());
  fprintf(fp, "  (:init\n");
  fprintf(fp, "   (&rest args)\n");
  fprintf(fp, "   (let ()\n");
  // send super :init
  fprintf(fp, "     (send-super* :init :name \"%s\" args)\n", arobot_name.c_str());
  fprintf(fp, "\n");

  printLinks();

  printJoints();

  printEndCoords();

  printSensors();

  printGeometries();

  fprintf(fp, "  )\n");
}

void ModelEuslisp::printLinks () {
#if URDFDOM_1_0_0_API
  for (map<LinkConstSharedPtr, Pose >::iterator it = m_link_coords.begin();
#else
  for (map<boost::shared_ptr<const Link>, Pose >::iterator it = m_link_coords.begin();
#endif
       it != m_link_coords.end(); it++) {
    printLink(it->first, it->second);
  }

#if URDFDOM_1_0_0_API
  for (map<string, JointSharedPtr>::iterator it = robot->joints_.begin();
#else
  for (map<string, boost::shared_ptr<Joint> >::iterator it = robot->joints_.begin();
#endif
       it != robot->joints_.end(); it++) {
    if (add_link_suffix) {
      fprintf(fp, "     (send %s_lk :assoc %s_lk)\n",
              it->second->parent_link_name.c_str(), it->second->child_link_name.c_str());
    } else {
      fprintf(fp, "     (send %s :assoc %s)\n",
              it->second->parent_link_name.c_str(), it->second->child_link_name.c_str());
    }
  }

  if (add_link_suffix) {
    fprintf(fp, "     (send self :assoc %s_lk)\n", robot->root_link_->name.c_str());
  } else {
    fprintf(fp, "     (send self :assoc %s)\n", robot->root_link_->name.c_str());
  }
}

#if URDFDOM_1_0_0_API
void ModelEuslisp::printLink (LinkConstSharedPtr link, Pose &pose) {
#else
void ModelEuslisp::printLink (boost::shared_ptr<const Link> link, Pose &pose) {
#endif
  string thisNodeName;
  if (add_link_suffix) {
    thisNodeName.assign(link->name);
    thisNodeName += "_lk";
  } else {
    thisNodeName.assign(link->name);
  }
  fprintf(fp, "     ;; link: %s\n", thisNodeName.c_str());
  fprintf(fp, "     (let ((geom-lst (list\n");
  {
    int geom_counter = 0;
#if URDFDOM_1_0_0_API
    map <LinkConstSharedPtr, MapVisual >::iterator it = m_link_visual.find (link);
#else
    map <boost::shared_ptr<const Link>, MapVisual >::iterator it = m_link_visual.find (link);
#endif
    if (it != m_link_visual.end()) {
      for( MapVisual::iterator vmap = it->second.begin();
           vmap != it->second.end(); vmap++) {
        if(geom_counter > 0) fprintf(fp, "\n");
        fprintf(fp, "                       (send self :_make_instance_%s)", vmap->first.c_str());
        geom_counter++;
      }
    }
#if 0
    if(geom_counter == 0) {
      fprintf(fp, "(make-cube 10 10 10)))) ;; no geometry in this link\n");
    } else {
      fprintf(fp, ")))\n");
    }
#endif
    fprintf(fp, ")))\n"); //
  }
  fprintf(fp, "       (dolist (g (cdr geom-lst)) (send (car geom-lst) :assoc g))\n");
  fprintf(fp, "       (setq %s\n", thisNodeName.c_str());
  fprintf(fp, "             (instance bodyset-link\n");
  fprintf(fp, "                       :init (make-cascoords)\n");
  fprintf(fp, "                       :bodies geom-lst\n");
  fprintf(fp, "                       :name \"%s\"))\n", link->name.c_str());

  if (!!link->inertial) {
    fprintf(fp, "       ;; inertial parameter for %s\n", thisNodeName.c_str());
    fprintf(fp, "       (let ((tmp-cds (make-coords :pos ");
    fprintf(fp, "(float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE")",
            link->inertial->origin.position.x * 1000,
            link->inertial->origin.position.y * 1000,
            link->inertial->origin.position.z * 1000);
    {
      double qx, qy, qz, qw;
      link->inertial->origin.rotation.getQuaternion(qx, qy, qz, qw);
      if (qx != 0.0 || qy != 0.0 || qz != 0.0 || qw != 1.0) {
        fprintf(fp, "\n                                   :rot (quaternion2matrix ");
        fprintf(fp, "(float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE"))",
                qw, qx, qy, qz);
      }
      fprintf(fp, ")\n");
    }
    fprintf(fp, "                      ))\n");
    //
    fprintf(fp, "         (send %s :weight %.3f)\n", thisNodeName.c_str(), link->inertial->mass * 1000);
    fprintf(fp, "         (setq (%s . inertia-tensor)\n", thisNodeName.c_str());
    fprintf(fp, "               (m* (m* (send tmp-cds :worldrot) (matrix\n");
    fprintf(fp, "                                                  (float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE")\n",
            link->inertial->ixx * 1000 * 1000 * 1000,
            link->inertial->ixy * 1000 * 1000 * 1000,
            link->inertial->ixz * 1000 * 1000 * 1000);
    fprintf(fp, "                                                  (float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE")\n",
            link->inertial->ixy * 1000 * 1000 * 1000,
            link->inertial->iyy * 1000 * 1000 * 1000,
            link->inertial->iyz * 1000 * 1000 * 1000);
    fprintf(fp, "                                                  (float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE"))\n",
            link->inertial->ixz * 1000 * 1000 * 1000,
            link->inertial->iyz * 1000 * 1000 * 1000,
            link->inertial->izz * 1000 * 1000 * 1000);
    fprintf(fp, "                 ) (transpose (send tmp-cds :worldrot))))\n");
    fprintf(fp, "         (setq (%s . acentroid) (copy-seq (send tmp-cds :worldpos)))\n", thisNodeName.c_str());
    fprintf(fp, "         )\n");
  } else {
    fprintf(fp, "       (progn (send %s :weight 0.0) (setq (%s . acentroid) (float-vector 0 0 0)) (send %s :inertia-tensor #2f((0 0 0)(0 0 0)(0 0 0))))\n",
            thisNodeName.c_str(), thisNodeName.c_str(), thisNodeName.c_str());
  }
  //
  fprintf(fp, "       ;; global coordinates for %s\n", thisNodeName.c_str());
  fprintf(fp, "       (let ((world-cds (make-coords :pos ");
  fprintf(fp, "(float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE")",
          pose.position.x * 1000, pose.position.y * 1000, pose.position.z * 1000);
  {
    double qx, qy, qz, qw;
    pose.rotation.getQuaternion(qx, qy, qz, qw);
    if (qx != 0.0 || qy != 0.0 || qz != 0.0 || qw != 1.0) {
      fprintf(fp, "\n                                   :rot (quaternion2matrix ");
      fprintf(fp, "(float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE"))",
              qw, qx, qy, qz);
    }
    fprintf(fp, ")\n");
  }
  fprintf(fp, "                        ))\n");
  fprintf(fp, "         (send %s :transform world-cds))\n", thisNodeName.c_str());
  fprintf(fp, "       )\n\n");
}

void ModelEuslisp::printJoints () {
  fprintf(fp, "\n     ;; joint models\n");
#if URDFDOM_1_0_0_API
  for (map<string, JointSharedPtr>::iterator joint = robot->joints_.begin();
#else
  for (map<string, boost::shared_ptr<Joint> >::iterator joint = robot->joints_.begin();
#endif
       joint != robot->joints_.end(); joint++) {
    printJoint(joint->second);
  }
}

#if URDFDOM_1_0_0_API
void ModelEuslisp::printJoint (JointConstSharedPtr joint) {
#else
void ModelEuslisp::printJoint (boost::shared_ptr<const Joint> joint) {
#endif
  bool linear = (joint->type==Joint::PRISMATIC);
  if (joint->type != Joint::REVOLUTE && joint->type !=Joint::CONTINUOUS
      && joint->type !=Joint::PRISMATIC && joint->type != Joint::FIXED) {
    // error
  }
  string thisJointName;
  if (add_joint_suffix) {
    thisJointName.assign(joint->name);
    if (joint->type == Joint::FIXED) {
      thisJointName += "_fixed_jt";
    } else {
      thisJointName += "_jt";
    }
  } else {
    thisJointName.assign(joint->name);
  }
  fprintf(fp, "     ;; joint: %s\n", thisJointName.c_str());
  fprintf(fp, "     (setq %s\n", thisJointName.c_str());
  fprintf(fp, "           (instance %s :init\n", linear?"linear-joint":"rotational-joint");
  fprintf(fp, "                     :name \"%s\"\n", joint->name.c_str());
  if (add_link_suffix) {
    fprintf(fp, "                     :parent-link %s_lk :child-link %s_lk\n",
            joint->parent_link_name.c_str(), joint->child_link_name.c_str());
  } else {
    fprintf(fp, "                     :parent-link %s :child-link %s\n",
            joint->parent_link_name.c_str(), joint->child_link_name.c_str());
  }
  if (joint->axis.x == 0.0 && joint->axis.y == 0.0 && joint->axis.z == 0.0) {
    fprintf(fp, "                     :axis (float-vector 1 1 1) ;; fixed joint??\n");
  } else
  { // axis
    fprintf(fp, "                     :axis ");
    fprintf(fp, "(float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE")\n",
            joint->axis.x, joint->axis.y, joint->axis.z);
  }
  if (!!joint->limits) {
    float min = joint->limits->lower;
    float max = joint->limits->upper;
    fprintf(fp, "                     ");
    if (joint->type ==Joint::CONTINUOUS) {
      fprintf(fp, "                     :min *-inf* :max *inf*\n");
    } else {
      fprintf(fp, ":min ");
      if (min == -FLT_MAX) fprintf(fp, "*-inf*"); else
        fprintf(fp, "%f", joint->type ==Joint::PRISMATIC ? min * 1000 : min * 180.0 / M_PI);
      fprintf(fp, " :max ");
      if (max == FLT_MAX) fprintf(fp,  "*inf*"); else
        fprintf(fp, "%f", joint->type ==Joint::PRISMATIC ? max * 1000 : max * 180.0 / M_PI);
      fprintf(fp, "\n");
    }
    fprintf(fp, "                     :max-joint-velocity %f\n", joint->limits->velocity);
    fprintf(fp, "                     :max-joint-torque %f\n", joint->limits->effort);
  } else if (joint->type ==Joint::CONTINUOUS) {
    // no limitation for rotation
    fprintf(fp, "                     :min *-inf* :max *inf*\n");
  } else {
    // fixed joint
    fprintf(fp, "                     :min 0.0 :max 0.0\n");
  }
  fprintf(fp, "                     ))\n");
}

void ModelEuslisp::printEndCoords () {
  // TODO: end coords from collada ...
  fprintf(fp, "     ;; end coords from yaml file\n");
  BOOST_FOREACH(link_joint_pair& limb, limbs) {
    string limb_name = limb.first;
    vector<string> link_names = limb.second.first;

    if (link_names.size()>0) {
      string end_coords_parent_name(link_names.back());
      try {
        const YAML::Node& n = doc[limb_name+"-end-coords"]["parent"];
#ifdef USE_CURRENT_YAML
        end_coords_parent_name = n.as<std::string>();
#else
        n >> end_coords_parent_name;
#endif
      } catch(YAML::RepresentationException& e) {
      }
      if (add_link_suffix) {
        fprintf(fp, "     (setq %s-end-coords (make-cascoords :coords (send %s_lk :copy-worldcoords) :name :%s-end-coords))\n",
                limb_name.c_str(), end_coords_parent_name.c_str(), limb_name.c_str());
      } else {
        fprintf(fp, "     (setq %s-end-coords (make-cascoords :coords (send %s :copy-worldcoords) :name %s-end-coords))\n",
                limb_name.c_str(), end_coords_parent_name.c_str(), limb_name.c_str());
      }
      try {
        const YAML::Node& n = doc[limb_name+"-end-coords"]["translate"];
        if ( n.size() > 0 ) {
          double value;
          fprintf(fp, "     (send %s-end-coords :translate (float-vector", limb_name.c_str());
#ifdef USE_CURRENT_YAML
          for(unsigned int i = 0; i < 3; i++) fprintf(fp, " "FLOAT_PRECISION_FINE"", 1000*n[i].as<double>());
#else
          for(unsigned int i = 0; i < 3; i++) { n[i]>>value; fprintf(fp, " "FLOAT_PRECISION_FINE"", 1000*value);}
#endif
          fprintf(fp, "))\n");
        }
      } catch(YAML::RepresentationException& e) {
      }
      try {
        const YAML::Node& n = doc[limb_name+"-end-coords"]["rotate"];
        if ( n.size() > 0 ) {
          double value;
          fprintf(fp, "     (send %s-end-coords :rotate", limb_name.c_str());
#if USE_CURRENT_YAML
          for(unsigned int i = 3; i < 4; i++) fprintf(fp, " "FLOAT_PRECISION_FINE"", M_PI/180*n[i].as<double>());
#else
          for(unsigned int i = 3; i < 4; i++) { n[i]>>value; fprintf(fp, " "FLOAT_PRECISION_FINE"", M_PI/180*value);}
#endif
          fprintf(fp, " (float-vector");
#if USE_CURRENT_YAML
          for(unsigned int i = 0; i < 3; i++) fprintf(fp, " "FLOAT_PRECISION_FINE"", n[i].as<double>());
#else
          for(unsigned int i = 0; i < 3; i++) { n[i]>>value; fprintf(fp, " "FLOAT_PRECISION_FINE"", value);}
#endif
          fprintf(fp, "))\n");
        }
      } catch(YAML::RepresentationException& e) {
      }
      if(add_link_suffix) {
        fprintf(fp, "     (send %s_lk :assoc %s-end-coords)\n", end_coords_parent_name.c_str(), limb_name.c_str());
      } else {
        fprintf(fp, "     (send %s :assoc %s-end-coords)\n", end_coords_parent_name.c_str(), limb_name.c_str());
      }
    }
  }
  fprintf(fp, "\n");

  // limb name
  fprintf(fp, "     ;; limbs\n");
  BOOST_FOREACH(link_joint_pair& limb, limbs) {
    string limb_name = limb.first;
    vector<string> link_names = limb.second.first;
    if ( link_names.size() > 0 ) {
      fprintf(fp, "     (setq %s (list", limb_name.c_str());
      if (add_link_suffix) {
        for (unsigned int i = 0; i < link_names.size(); i++)
          fprintf(fp, " %s_lk", link_names[i].c_str());
        fprintf(fp, "))\n");
      } else {
        for (unsigned int i = 0; i < link_names.size(); i++)
          fprintf(fp, " %s", link_names[i].c_str());
        fprintf(fp, "))\n");
      }
      fprintf(fp, "\n");
      // find root link by tracing limb's link list
      fprintf(fp, "     (setq %s-root-link\n", limb_name.c_str());
      fprintf(fp, "           (labels ((find-parent (l) (if (find (send l :parent) %s) (find-parent (send l :parent)) l)))\n",
              limb_name.c_str());
      fprintf(fp, "             (find-parent (car %s))))\n", limb_name.c_str());
    }
  }
  fprintf(fp, "\n");

  // link name
  fprintf(fp, "     ;; links\n");
  if (add_link_suffix) {
    fprintf(fp, "     (setq links (list %s_lk", robot->root_link_->name.c_str());
  } else {
    fprintf(fp, "     (setq links (list %s", robot->root_link_->name.c_str());
  }
  BOOST_FOREACH(link_joint_pair& limb, limbs) {
    string limb_name = limb.first;
    vector<string> link_names = limb.second.first;
    if (add_link_suffix) {
      for (unsigned int i = 0; i < link_names.size(); i++) {
        fprintf(fp, " %s_lk", link_names[i].c_str());
      }
    } else {
      for (unsigned int i = 0; i < link_names.size(); i++) {
        fprintf(fp, " %s", link_names[i].c_str());
      }
    }
  }
  fprintf(fp, "))\n");
  fprintf(fp, "\n");

  fprintf(fp, "     ;; joint-list\n");
  fprintf(fp, "     (setq joint-list (list");
  BOOST_FOREACH(link_joint_pair& limb, limbs) {
    vector<string> joint_names = limb.second.second;
    if(add_joint_suffix) {
      for (unsigned int i = 0; i < joint_names.size(); i++) {
        fprintf(fp, " %s_jt", joint_names[i].c_str());
      }
    } else {
      for (unsigned int i = 0; i < joint_names.size(); i++) {
        fprintf(fp, " %s", joint_names[i].c_str());
      }
    }
  }
  fprintf(fp, "))\n");
  fprintf(fp, "\n");
  fprintf(fp, "     ;; sensor-coords\n");
  for (vector<daeSensor>::iterator it = m_sensors.begin(); it != m_sensors.end(); it++) {
    string plink;
    if (add_link_suffix) {
      plink = it->parent_link + "_lk";
    } else {
      plink = it->parent_link;
    }
    string name = it->name;

    fprintf(fp, "     ;;\n");
    fprintf(fp, "     (setq %s-sensor-coords (make-cascoords :name \"%s\" :coords (send %s :copy-worldcoords)))\n",
            name.c_str(), name.c_str(), plink.c_str());
    fprintf(fp, "     (send %s-sensor-coords :put :sensor-type :%s)\n", name.c_str(), it->sensor_type.c_str());
    fprintf(fp, "     (send %s-sensor-coords :put :sensor-id %s)\n", name.c_str(), it->sensor_id.c_str());

    fprintf(fp, "     (send %s-sensor-coords :transform (let ((cds (make-coords)))\n", name.c_str());
    for (size_t i = 0; i < it->ptrans.size(); i++) {
      domRotateRef protate = daeSafeCast<domRotate>(it->ptrans[i]);
      if( !!protate ) {
        fprintf(fp, "                                               (send cds :transform (make-coords :axis ");
        fprintf(fp, "(let ((tmp-axis (float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE"))) (if (eps= (norm tmp-axis) 0.0) (float-vector 1 0 0) tmp-axis))",
                protate->getValue()[0], protate->getValue()[1], protate->getValue()[2]);
        fprintf(fp, " :angle");
        fprintf(fp, " "FLOAT_PRECISION_FINE"))\n", protate->getValue()[3]*(M_PI/180.0));
        continue;
      }
      domTranslateRef ptrans = daeSafeCast<domTranslate>(it->ptrans[i]);
      if( !!ptrans ) {
        fprintf(fp, "                                               (send cds :transform (make-coords :pos (float-vector ");
        for(size_t i = 0; i < 3; i++) { fprintf(fp, " "FLOAT_PRECISION_FINE"", 1000*(ptrans->getValue()[i])); }
        fprintf(fp, ")))\n");
        continue;
      }
    }
    fprintf(fp, "                                               ))\n");

    fprintf(fp, "     (send %s :assoc %s-sensor-coords)\n", plink.c_str(), name.c_str());
  }
  // sensors
  printSensorLists();
  fprintf(fp, "\n");
  // init ending
  fprintf(fp, "     ;; init-ending\n");
  fprintf(fp, "     (send self :init-ending) ;; :urdf\n\n");

  // bodies
  fprintf(fp, "     ;; overwrite bodies to return draw-things links not (send link :bodies)\n");
  fprintf(fp, "     (setq bodies (flatten (mapcar #'(lambda (b) (if (find-method b :bodies) (send b :bodies))) (list");
#if URDFDOM_1_0_0_API
  for (map<string, LinkSharedPtr>::iterator link = robot->links_.begin();
#else
  for (map<string, boost::shared_ptr<Link> >::iterator link = robot->links_.begin();
#endif
       link != robot->links_.end(); link++) {
    if (add_link_suffix) {
      fprintf(fp, " %s_lk", link->first.c_str());
    } else {
      fprintf(fp, " %s", link->first.c_str());
    }
  }
  fprintf(fp, "))))\n\n");

  // when - angle-vector: reset-pose is defined in yaml file
  fprintf(fp, "     (when (member :reset-pose (send self :methods))");
  fprintf(fp, "           (send self :reset-pose)) ;; :set reset-pose\n\n");
  //fprintf(fp, "     (send self :reset-pose) ;; :set reset-pose\n\n");

  fprintf(fp, "     self)) ;; end of :init\n\n");

  try {
    const YAML::Node& n = doc["angle-vector"];
    if ( n.size() > 0 ) fprintf(fp, "  ;; pre-defined pose methods\n");
#ifdef USE_CURRENT_YAML
    for(YAML::const_iterator it = n.begin(); it != n.end(); it++) {
      string name = it->first.as<std::string>();
#else
    for(YAML::Iterator it = n.begin(); it != n.end(); it++) {
      string name; it.first() >> name;
#endif
      string limbs_symbols = "";
      for (size_t i = 0; i < limbs.size(); i++) {
        string limb_name = limbs[i].first;
        limbs_symbols += i == 0 ? (":" + limb_name) : (" :" + limb_name);
      }
      fprintf(fp,
              "\n"
              "    (:%s (&optional (limbs '(%s)))\n"
              "      \"Predefined pose named %s.\"\n"
              "      (unless (listp limbs) (setq limbs (list limbs)))\n"
              "      (dolist (limb limbs)\n"
              "        (case limb", name.c_str(), limbs_symbols.c_str(), name.c_str());
#ifdef USE_CURRENT_YAML
      const YAML::Node& v = it->second;
#else
      const YAML::Node& v = it.second();
#endif
      size_t i_joint = 0;
      for (size_t i = 0; i < limbs.size(); i++) {
        string limb_name = limbs[i].first;
        fprintf(fp,
                "\n"
                "          (:%s (send self limb :angle-vector (float-vector",
                limb_name.c_str());
        vector<string> joint_names = limbs[i].second.second;
        size_t j;
        try {
        for (j = 0; j < joint_names.size(); j++) {
#ifdef USE_CURRENT_YAML
          fprintf(fp, " %f", v[i_joint].as<double>());
#else
          double d; v[i_joint] >> d;
          fprintf(fp, " %f", d);
#endif
          i_joint += 1;
        }
        } catch(YAML::RepresentationException& e) {
          ROS_ERROR("****** Angle-vector may be shorter than joint-list, please fix .yaml ******");
          while ( j < joint_names.size() ) {
            fprintf(fp, " 0.0"); // padding dummy
            j++;
          }
        }
        fprintf(fp, ")))");
      }
      fprintf(fp,
              "\n"
              "          (t (format t \"Unknown limb is passed: ~a~%\" limb))");
      fprintf(fp,
              "))\n"
              "      (send self :angle-vector))");
    }
  } catch(YAML::RepresentationException& e) {
    ROS_ERROR("****** Some trouble for reading limbs, please fix .yaml ******");
  }

  // all joint and link name
  fprintf(fp, "\n  ;; all joints\n");
#if URDFDOM_1_0_0_API
  for (map<string, JointSharedPtr>::iterator joint = robot->joints_.begin();
#else
  for (map<string, boost::shared_ptr<Joint> >::iterator joint = robot->joints_.begin();
#endif
       joint != robot->joints_.end(); joint++) {
    if(add_joint_suffix) {
      if (joint->second->type == Joint::FIXED) {
        fprintf(fp, "  (:%s (&rest args) (forward-message-to %s_fixed_jt args))\n", joint->first.c_str(), joint->first.c_str());
      } else {
        fprintf(fp, "  (:%s (&rest args) (forward-message-to %s_jt args))\n", joint->first.c_str(), joint->first.c_str());
      }
    } else {
      fprintf(fp, "  (:%s (&rest args) (forward-message-to %s args))\n", joint->first.c_str(), joint->first.c_str());
    }
  }

  if (add_link_suffix) {
    fprintf(fp, "\n  ;; all links forwarding\n");
    fprintf(fp, "  (:links (&rest args)\n");
    fprintf(fp, "   (if (null args) (return-from :links (send-super :links)))\n");
    fprintf(fp, "   (let ((key (car args))\n           (nargs (cdr args)))\n");
    fprintf(fp, "     (unless (keywordp key)\n         (return-from :links (send-super* :links args)))\n       (case key\n");
#if URDFDOM_1_0_0_API
    for (map<string, LinkSharedPtr>::iterator link = robot->links_.begin();
#else
    for (map<string, boost::shared_ptr<Link> >::iterator link = robot->links_.begin();
#endif
         link != robot->links_.end(); link++) {
      fprintf(fp, "       (:%s (forward-message-to %s_lk nargs))\n", link->first.c_str(), link->first.c_str());
    }
    fprintf(fp, "       (t (send-super* :links args)))))\n");
  }
  fprintf(fp, "\n  ;; all links\n");

#if URDFDOM_1_0_0_API
  for (map<string, LinkSharedPtr>::iterator link = robot->links_.begin();
#else
  for (map<string, boost::shared_ptr<Link> >::iterator link = robot->links_.begin();
#endif
       link != robot->links_.end(); link++) {
    if (add_link_suffix) {
      fprintf(fp, "  (:%s_lk (&rest args) (forward-message-to %s_lk args))\n", link->first.c_str(), link->first.c_str());
    } else {
      fprintf(fp, "  (:%s (&rest args) (forward-message-to %s args))\n", link->first.c_str(), link->first.c_str());
    }
  }

  fprintf(fp, "\n  ;; user-defined joint\n");
  for(vector<pair<string, string> >::iterator it = g_all_link_names.begin();
      it != g_all_link_names.end(); it++){
    if(add_joint_suffix) {
      fprintf(fp, "  (:%s (&rest args) (forward-message-to %s_jt args))\n", it->second.c_str(), it->first.c_str());
    } else {
      fprintf(fp, "  (:%s (&rest args) (forward-message-to %s args))\n", it->second.c_str(), it->first.c_str());
    }
  }
}

string ModelEuslisp::getSensorType (const domExtraRef pextra) {
  // get sensor_type from extra tag
  string sensor_type;
  for (size_t ii = 0; ii < dae.getDatabase()->getElementCount(NULL, "extra", NULL); ii++) {
    domExtra *tmpextra;
    dae.getDatabase()->getElement((daeElement**)&tmpextra, ii, NULL, "extra");
    if (tmpextra->getType() == string("library_sensors")) {
      for (size_t icon = 0; icon < tmpextra->getTechnique_array()[0]->getContents().getCount(); icon++) {
        if ((string("#") + tmpextra->getTechnique_array()[0]->getContents()[icon]->getAttribute("id")) ==
            pextra->getTechnique_array()[0]->getChild("instance_sensor")->getAttribute("url")) {
          sensor_type = tmpextra->getTechnique_array()[0]->getContents()[icon]->getAttribute("type");
        }
      }
    }
  }
  return sensor_type;
}
domLink* ModelEuslisp::findLinkfromKinematics (domLink* thisLink, const string& link_name) {
  if (thisLink->getName()==link_name) return thisLink;
  for(size_t ii = 0; ii < thisLink->getAttachment_full_array().getCount(); ++ii) {
    domLink* tmpLink = findLinkfromKinematics(thisLink->getAttachment_full_array()[ii]->getLink(), link_name);
    if (tmpLink) return tmpLink;
  }
  return NULL;
}
void ModelEuslisp::parseSensors () {
  int iRet = DAE_OK + 1;
  if(!collada_file.empty()) {
    iRet = dae.load(collada_file.c_str());
  }
  if ( iRet != DAE_OK ) {
    ROS_WARN("read sensor settings from yaml");
    // read yaml
    // sensor_name: 'sname', sensor_type: 'type', parent_link: 'LINK', translate: '0 0 0',  rotate: '1 0 0 90'
    // type -> base_force6d {force}, base_imu {gyro, acceleration}, base_pinhole_camera {camera}, they came from openrave collada
    try {
      const YAML::Node& sensor_doc = doc["sensors"];
      for(unsigned int i = 0; i < sensor_doc.size(); i++) {
        daeSensor s;
        const YAML::Node& n = sensor_doc[i];
#ifdef USE_CURRENT_YAML
        if( n["sensor_name"] )
          s.name = n["sensor_name"].as<std::string>();
        if( n["sensor_type"] )
          s.sensor_type = n["sensor_type"].as<std::string>();
        if( n["parent_link"] )
          s.parent_link = n["parent_link"].as<std::string>();
        if( n["sensor_id"] )
          s.sensor_id = n["sensor_id"].as<std::string>();
#else
        n["sensor_name"] >> s.name;
        n["sensor_type"] >> s.sensor_type;
        n["parent_link"] >> s.parent_link;
        if(!!n.FindValue("sensor_id")) {
          n["sensor_id"] >> s.sensor_id;
        }
#endif
        if (s.sensor_id == "") {
          stringstream str;
          str << (i+1);
          s.sensor_id = str.str();
        }
        std::cerr << ";; add sensor id: " << s.sensor_id;
        std::cerr << ", name: " << s.name;
        std::cerr << ", type: " << s.sensor_type;
        std::cerr << ", parent_link: " << s.parent_link;
        if (s.sensor_type == "gyro" || s.sensor_type == "acceleration") s.sensor_type = "base_imu";
        if (s.sensor_type == "force") s.sensor_type = "base_force6d";

#ifdef USE_CURRENT_YAML
        if(n["translate"]) {
          std::string translate = n["translate"].as<std::string>();
#else
        if(const YAML::Node *pn = n.FindValue("translate")) {
          std::string translate;
          *pn >> translate;
#endif
          std::istringstream strm(translate);
          double x, y, z;
          strm >> x; strm >> y; strm >> z;
          daeElementRef ref = domTranslate::create (dae);
          s.ptrans.push_back(ref);
          domTranslateRef ptrans = daeSafeCast<domTranslate>(ref);
          ptrans->getValue().set(0, x);
          ptrans->getValue().set(1, y);
          ptrans->getValue().set(2, z);
          std::cerr << ", trans: " << x << " " << y << " " << z;
        }
#ifdef USE_CURRENT_YAML
        if(n["rotate"]) {
          std::string rotate = n["rotate"].as<std::string>();
#else
        if(const YAML::Node *pn = n.FindValue("rotate")) {
          std::string rotate;
          *pn >> rotate;
#endif
          std::istringstream strm(rotate);
          double r, p, y, ang;
          strm >> r; strm >> p; strm >> y; strm >> ang;
          daeElementRef ref = domRotate::create (dae);
          s.ptrans.push_back(ref);
          domRotateRef prot = daeSafeCast<domRotate>(ref);
          prot->getValue().set(0, r);
          prot->getValue().set(1, p);
          prot->getValue().set(2, y);
          prot->getValue().set(3, ang);
          std::cerr << ", rot: " << r << " " << p << " " << y << " " << ang;
        }
        std::cerr << std::endl;
        m_sensors.push_back(s);
      }
    } catch(YAML::Exception& e) {
      std::cerr << "[YAML error] : " << e.msg << std::endl;
    }
    stable_sort(m_sensors.begin(), m_sensors.end(), ModelEuslisp::daeSensor::compare);
    return;
  }
  if ( dae.getDatabase()->getDocumentCount() != 1 ) {
    ROS_WARN("Number of documnet is not 1 / %d", dae.getDatabase()->getDocumentCount());
    return;
  }
  g_document = dae.getDatabase()->getDocument((daeUInt)0);

  if ( dae.getDatabase()->getElementCount(NULL, "articulated_system", NULL) > 0 ) {
    domKinematics_model *thisKinematics;
    dae.getDatabase()->getElement((daeElement**)&thisKinematics, 0, NULL, "kinematics_model");

#if URDFDOM_1_0_0_API
    for (map<string, LinkSharedPtr>::iterator link = robot->links_.begin();
#else
    for (map<string, boost::shared_ptr<Link> >::iterator link = robot->links_.begin();
#endif
         link != robot->links_.end(); link++) {

      domLink* thisLink = findLinkfromKinematics(thisKinematics->getTechnique_common()->getLink_array()[0],
                                                 link->second->name);
      if (!thisLink) continue;
      domArticulated_system *thisArticulated;
      for ( size_t ii = 0; ii < dae.getDatabase()->getElementCount(NULL, "articulated_system", NULL); ii++) {
        dae.getDatabase()->getElement((daeElement**)&thisArticulated, ii, NULL, "articulated_system");
        if ( thisArticulated->getExtra_array().getCount() > 0 ) break;
      }
      for(size_t ie = 0; ie < thisArticulated->getExtra_array().getCount(); ++ie) {
        domExtraRef pextra = thisArticulated->getExtra_array()[ie];
        // find element which type is attach_sensor and is attached to thisNode
        if ( strcmp(pextra->getType(), "attach_sensor") == 0 ) {

          daeElement* frame_origin = pextra->getTechnique_array()[0]->getChild("frame_origin");
          if ( string(thisKinematics->getId())+string("/")+string(thisLink->getSid()) ==
               frame_origin->getAttribute("link") ) {
            daeSensor dsensor;
            dsensor.name = pextra->getName();
            dsensor.parent_link = link->second->name;
            dsensor.sensor_type = getSensorType(pextra);
            string sensor_url(pextra->getTechnique_array()[0]->getChild("instance_sensor")->getAttribute("url"));
            dsensor.sensor_id = sensor_url.erase(sensor_url.find( "#sensor" ), 7);
            daeTArray<daeElementRef> children;
            frame_origin->getChildren(children);
            for(size_t i = 0; i < children.getCount(); ++i) {
              dsensor.ptrans.push_back(children[i]);
            }
            m_sensors.push_back(dsensor);
            ROS_WARN_STREAM("Sensor " << pextra->getName() << " is attached to " << link->second->name
                            << " " << dsensor.sensor_type << " " << sensor_url);
          }
        }
      }
    }
    // sort sensor
    stable_sort(m_sensors.begin(), m_sensors.end(), ModelEuslisp::daeSensor::compare);
  }
}

void ModelEuslisp::printSensors() {
  fprintf(fp, "\n  ;; attach_sensor methods\n");
  for (vector<daeSensor>::iterator it = m_sensors.begin(); it != m_sensors.end(); it++) {
    if(add_sensor_suffix) {
      fprintf(fp, "  (:%s_sn (&rest args) (forward-message-to %s-sensor-coords args))\n",
              it->name.c_str(), it->name.c_str());
    } else {
      fprintf(fp, "  (:%s (&rest args) (forward-message-to %s-sensor-coords args))\n",
              it->name.c_str(), it->name.c_str());
    }
  }
}

void ModelEuslisp::printSensorLists() {
  fprintf(fp, "\n     ;; attach_sensor lists\n");
  fprintf(fp, "     (setq force-sensors (list ");
  for (vector<daeSensor>::iterator it = m_sensors.begin(); it != m_sensors.end(); it++) {
    if (it->sensor_type == "base_force6d") {
      fprintf(fp, "%s-sensor-coords ", it->name.c_str());
    }
  }
  fprintf(fp, "))\n");
  fprintf(fp, "     (setq imu-sensors (list ");
  for (vector<daeSensor>::iterator it = m_sensors.begin(); it != m_sensors.end(); it++) {
    if (it->sensor_type == "base_imu") {
      fprintf(fp, "%s-sensor-coords ", it->name.c_str());
    }
  }
  fprintf(fp, "))\n");
  fprintf(fp, "     (setq cameras (list ");
  for (vector<daeSensor>::iterator it = m_sensors.begin(); it != m_sensors.end(); it++) {
    if (it->sensor_type == "base_pinhole_camera") {
      fprintf(fp, "%s-sensor-coords ", it->name.c_str());
    }
  }
  fprintf(fp, "))\n");
}

#if URDFDOM_1_0_0_API
void ModelEuslisp::printGeometry (GeometrySharedPtr g, const Pose &pose,
#else
void ModelEuslisp::printGeometry (boost::shared_ptr<Geometry> g, const Pose &pose,
#endif
                                  const string &name, const string &material_name) {
  string gname(name);
  if (g->type == Geometry::MESH) gname = ((Mesh *)(g.get()))->filename;
  fprintf(fp, "  (:_make_instance_%s ()\n", name.c_str());
  fprintf(fp, "    (let (geom glv qhull\n");
  fprintf(fp, "          (local-cds (make-coords :pos ");
  fprintf(fp, "(float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE")",
              pose.position.x * 1000,
              pose.position.y * 1000,
              pose.position.z * 1000);
  {
    double qx, qy, qz, qw;
    pose.rotation.getQuaternion(qx, qy, qz, qw);
    if (qx != 0.0 || qy != 0.0 || qz != 0.0 || qw != 1.0) {
      fprintf(fp, "\n                                    :rot (quaternion2matrix ");
      fprintf(fp, "(float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE"))",
              qw, qx, qy, qz);
    }
    fprintf(fp, ")))\n");
  }

  if (g->type != Geometry::MESH) {
    float col_r = 0.6;
    float col_g = 0.6;
    float col_b = 0.6;
    float col_a = 1.0;
    if (material_name.size() > 0) {
#if URDFDOM_1_0_0_API
      map <string, MaterialConstSharedPtr>::iterator it = m_materials.find(material_name);
#else
      map <string, boost::shared_ptr<const Material> >::iterator it = m_materials.find(material_name);
#endif
      if (it != m_materials.end()) {
#if URDFDOM_1_0_0_API
        MaterialConstSharedPtr m = it->second;
#else
        boost::shared_ptr<const Material> m = it->second;
#endif
        col_r = m->color.r;
        col_g = m->color.g;
        col_b = m->color.b;
        col_a = m->color.a;
#if 0
        std::cerr << "material " << material_name << " found" << std::endl;
        std::cerr << "color: "
                  << m->color.r << " "
                  << m->color.g << " "
                  << m->color.b << " "
                  << m->color.a << std::endl;
        std::cerr << "fname: " << m->texture_filename << std::endl;
#endif
      } else {
        std::cerr << "can not find material " << material_name << std::endl;
      }
    }
    if (g->type == Geometry::SPHERE) {
#if 0
      std::cerr << "SPHERE: " << name
                << ", radius = " << ((Sphere *)g.get())->radius
                << std::endl;
#endif
      double radius = ((Sphere *)g.get())->radius;
      fprintf(fp, "      (let ((bdy (make-sphere %f)))\n", 1000*radius);
      fprintf(fp, "         (setq qhull bdy)\n");
      fprintf(fp, "         (setq glv (gl::make-glvertices-from-faceset bdy\n");
      fprintf(fp, "                     :material (list (list :ambient (float-vector %f %f %f %f))\n", col_r, col_g, col_b, col_a);
      fprintf(fp, "                                     (list :diffuse (float-vector %f %f %f %f)))))\n", col_r, col_g, col_b, col_a);
      fprintf(fp, "         (send glv :transform local-cds)\n");
      fprintf(fp, "         (send bdy :transform local-cds)\n");
      fprintf(fp, "         (send glv :calc-normals))\n");
    } else if (g->type == Geometry::BOX) {
      Vector3 vec = ((Box *)g.get())->dim;
#if 0
      std::cerr << "BOX: " << name
                << ", dim = " << vec.x << " " << vec.y << " " << vec.z
                << std::endl;
#endif
      fprintf(fp, "      (let ((bdy (make-cube %f %f %f)))\n", 1000*vec.x, 1000*vec.y, 1000*vec.z);
      fprintf(fp, "         (setq qhull bdy)\n");
      fprintf(fp, "         (setq glv (gl::make-glvertices-from-faceset bdy\n");
      fprintf(fp, "                     :material (list (list :ambient (float-vector %f %f %f %f))\n", col_r, col_g, col_b, col_a);
      fprintf(fp, "                                     (list :diffuse (float-vector %f %f %f %f)))))\n", col_r, col_g, col_b, col_a);
      fprintf(fp, "         (send glv :transform local-cds)\n");
      fprintf(fp, "         (send bdy :transform local-cds)\n");
      fprintf(fp, "         (send glv :calc-normals))\n");
    } else if (g->type == Geometry::CYLINDER) {
#if 0
      std::cerr << "CYLINDER: " << name
                << ", len = " << ((Cylinder *)g.get())->length
                << ", radius = " << ((Cylinder *)g.get())->radius
                << std::endl;
#endif
      double length = ((Cylinder *)g.get())->length;
      double radius = ((Cylinder *)g.get())->radius;
      fprintf(fp, "      (let ((bdy (make-cylinder %f %f :segments 24)))\n", 1000*radius, 1000*length);
      fprintf(fp, "         (send bdy :translate-vertices (float-vector 0 0 %f))\n", -500*length);
      fprintf(fp, "         (setq qhull bdy)\n");
      fprintf(fp, "         (setq glv (gl::make-glvertices-from-faceset bdy\n");
      fprintf(fp, "                     :material (list (list :ambient (float-vector %f %f %f %f))\n", col_r, col_g, col_b, col_a);
      fprintf(fp, "                                     (list :diffuse (float-vector %f %f %f %f)))))\n", col_r, col_g, col_b, col_a);
      fprintf(fp, "         (send glv :transform local-cds)\n");
      fprintf(fp, "         (send bdy :transform local-cds)\n");
      fprintf(fp, "         (send glv :calc-normals))\n");
    } else {
      std::cerr << "unknown geometry type: " << name << std::endl;
    }
  } else { // g->type == Geometry::MESH
    Assimp::Importer importer;
    importer.SetIOHandler(new ResourceIOSystem());

    const aiScene* raw_scene = importer.ReadFile(gname, 0);
    if (remove_normal) {
      // remove normal for reducing vertices
      for (unsigned int m = 0; m < raw_scene->mNumMeshes; m++) {
        aiMesh *a = raw_scene->mMeshes[m];
        if (!!a->mNormals) { a->mNormals = NULL; }
      }
    }
    const aiScene* scene = importer.ApplyPostProcessing (aiProcessPreset_TargetRealtime_MaxQuality &
                                                         ((~aiProcess_GenNormals) & (~aiProcess_GenSmoothNormals)));

    Vector3 scale = ((Mesh *)(g.get()))->scale;
    vector<coordT> points;
    if (scene && scene->HasMeshes() && !use_loadable_mesh) {
      fprintf(fp, "      (setq glv\n");
      fprintf(fp, "       (instance gl::glvertices :init\n");
      fprintf(fp, "                 (list ;; mesh list\n");
      // TODO: use g->scale
      printMesh(scene, scene->mRootNode, scale, material_name, points, true);
      fprintf(fp, "                  )\n");
      fprintf(fp, "                 ))\n");
      fprintf(fp, "      (send glv :transform local-cds)\n");
      fprintf(fp, "      (send glv :calc-normals)\n");
    } else if (scene && scene->HasMeshes()) {
      fprintf(fp, "      (setq glv (load-mesh-file (ros::resolve-ros-path \"%s\")\n", gname.c_str());
      fprintf(fp, "                                       :scale %f :process-max-quality t))\n", scale.x*1000);
      printMesh(scene, scene->mRootNode, scale, material_name, points, false);
      fprintf(fp, "      (send glv :transform local-cds)\n");
      fprintf(fp, "      (send glv :calc-normals)\n");
    } else {
      // error
    }
    // qhull
    if (points.size() > 0) {
      char qhull_attr[] = "qhull C-0.001";
      int ret = qh_new_qhull (3, points.size()/3, &points[0], 0, qhull_attr, NULL, stderr);
      fprintf(fp, "      (setq qhull\n");
      if ( ret ) {
        fprintf(fp, "            (instance faceset :init :faces (list\n");
        std::cerr << ";; points " << points.size() << std::endl; 
        for (unsigned int i = 0; i < points.size()/9; i++ ) {
          fprintf(fp, "              (instance face :init :vertices\n");
          fprintf(fp, "                (mapcar #'(lambda (v) (send local-cds :transform-vector v))\n");
          fprintf(fp, "                  (list (float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE") (float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE") (float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE"))))\n",
                  1000*points[i*9+0], 1000*points[i*9+1], 1000*points[i*9+2],
                  1000*points[i*9+3], 1000*points[i*9+4], 1000*points[i*9+5],
                  1000*points[i*9+6], 1000*points[i*9+7], 1000*points[i*9+8]);
        }
        fprintf(fp, "              ))\n");
      } else {
        fprintf(fp, "            ;; qhull %zd -> %d faces\n", points.size()/3, qh num_facets);
        fprintf(fp, "            (instance faceset :init :faces (list\n");
        // get faces
        facetT *facet;
        vertexT *vertex, **vertexp;
        FORALLfacets {
          fprintf(fp, "              (instance face :init :vertices\n");
          fprintf(fp, "                (mapcar #'(lambda (v) (send local-cds :transform-vector v))\n");
          fprintf(fp, "                  (nreverse (list ");
          setT *vertices = qh_facet3vertex(facet); // ccw?
          FOREACHvertex_(vertices) {
            fprintf(fp, " (float-vector "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE" "FLOAT_PRECISION_FINE")",
                    1000*vertex->point[0], 1000*vertex->point[1], 1000*vertex->point[2]);
          }
          fprintf(fp, "))))\n");
          qh_settempfree(&vertices);
        }
        fprintf(fp, "              ))\n");
      }
      fprintf(fp, "             )\n");
      qh_freeqhull(!qh_ALL);
      int curlong, totlong;    // memory remaining after qh_memfreeshort
      qh_memfreeshort (&curlong, &totlong);    // free short memory and memory allocator
      if (curlong || totlong) {
        fprintf (stderr, "qhull internal warning (user_eg, #1): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
      }
    }
  }
  fprintf(fp, "      (setq geom (instance collada-body :init :replace-obj qhull :name \"%s\"))\n", gname.c_str());
  fprintf(fp, "      (when glv\n");
  fprintf(fp, "        (setq (geom . gl::aglvertices) glv)\n");
  fprintf(fp, "        (send geom :assoc glv))\n");
  fprintf(fp, "      geom))\n");
}

void ModelEuslisp::printGeometries () {
  fprintf(fp, "\n  ;; geometries\n");
  if (use_collision) {
#if URDFDOM_1_0_0_API
    for(map <LinkConstSharedPtr, MapCollision >::iterator it = m_link_collision.begin();
#else
    for(map <boost::shared_ptr<const Link>, MapCollision >::iterator it = m_link_collision.begin();
#endif
        it != m_link_collision.end(); it++) {
      for( MapCollision::iterator cmap = it->second.begin();
           cmap != it->second.end(); cmap++) {
        printGeometry(cmap->second->geometry, cmap->second->origin,
                      cmap->first, "");
      }
    }
  } else {
#if URDFDOM_1_0_0_API
    for(map <LinkConstSharedPtr, MapVisual >::iterator it = m_link_visual.begin();
#else
    for(map <boost::shared_ptr<const Link>, MapVisual >::iterator it = m_link_visual.begin();
#endif
        it != m_link_visual.end(); it++) {
      for( MapVisual::iterator vmap = it->second.begin();
           vmap != it->second.end(); vmap++) {
        printGeometry(vmap->second->geometry, vmap->second->origin,
                      vmap->first, vmap->second->material_name);
      }
    }
  }
}

void ModelEuslisp::writeToFile (string &filename) {
  if (!robot) {
    cerr << ";; not robot" << endl;
    return;
  }

  fp = fopen(filename.c_str(),"w");

  if (fp == NULL) {
    return;
  }

  // print header
  utsname uname_buf;
  uname(&uname_buf);
  time_t tm;
  char timestr[100];
  time(&tm);
  strftime(timestr, 100, "%F %T %Z", localtime(&tm));

  fprintf(fp, ";;\n");
  fprintf(fp, ";; DO NOT EDIT THIS FILE\n");
  fprintf(fp, ";;\n");
  fprintf(fp, ";; this file is automatically generated from %s on (%s %s %s %s) at %s\n",
          filename.c_str(), uname_buf.sysname, uname_buf.nodename, uname_buf.release,
          uname_buf.machine, timestr);
  fprintf(fp, ";;\n");
  fprintf(fp, ";; %s $ ", boost::filesystem::current_path().c_str());
  //for (int i = 0; i < argc; i++) fprintf(fp, "%s ", argv[i]); fprintf(fp, "\n");
  fprintf(fp, ";;\n");
  fprintf(fp, "\n");

  // pase
  addLinkCoords();
  parseSensors();

  // start printing
  copyRobotClassDefinition();

  printRobotDefinition();

  printRobotMethods();

  fprintf(fp, "\n\n(provide :%s \"(%s %s %s) at %s\")\n\n",
          arobot_name.c_str(), uname_buf.nodename, uname_buf.release,
          uname_buf.machine, timestr);
  fflush(fp);
}

//// main ////
namespace po = boost::program_options;
int main(int argc, char** argv)
{
  bool use_collision = false;
  bool use_simple_geometry = false;
  bool use_loadable_mesh = false;
  bool remove_normal = true;

  string arobot_name;

  string input_file;
  string config_file;
  string pconfig_file; // backward compatibility (positional option)
  string output_file;

  po::options_description desc("Options for collada_to_urdf");
  desc.add_options()
    ("help", "produce help message")
    ("simple_geometry,V", "use bounding box for geometry")
    ("loadable_mesh,L", "loading mesh when creating robot model")
    ("add_normal,R", "remove normals from mesh for reducing vertices")
    ("use_collision,U", "use collision geometry (default collision is the same as visual)")
    ("robot_name,N", po::value< vector<string> >(), "output robot name")
    ("input_file,I", po::value< vector<string> >(), "input file")
    ("config_file,C", po::value< vector<string> >(), "configuration yaml file")
    ("pconfig_file", po::value< vector<string> >(), "not used (used internally)")
    ("output_file,O", po::value< vector<string> >(), "output file")
    ;

  po::positional_options_description p;
  p.add("input_file",  1);
  p.add("pconfig_file", 1);
  p.add("output_file", 1);

  po::variables_map vm;
  try {
    po::store(po::command_line_parser(argc, argv).
              options(desc).positional(p).run(), vm);
    po::notify(vm);
  }
  catch (po::error e) {
    cerr << ";; option parse error / " << e.what() << endl;
    return 1;
  }
  if (vm.count("help")) {
    cout << desc << "\n";
    return 1;
  }
  if (vm.count("input_file")) {
    vector<string> aa = vm["input_file"].as< vector<string> >();
    input_file = aa[0];
  }
  if (vm.count("config_file")) {
    vector<string> aa = vm["config_file"].as< vector<string> >();
    config_file = aa[0];
  }
  if (vm.count("pconfig_file")) {
    vector<string> aa = vm["pconfig_file"].as< vector<string> >();
    pconfig_file = aa[0];
  }
  if (vm.count("output_file")) {
    vector<string> aa = vm["output_file"].as< vector<string> >();
    output_file = aa[0];
  }
  if (vm.count("simple_geometry")) {
    use_simple_geometry = true;
    cerr << ";; Using simple_geometry" << endl;
  }
  if (vm.count("loadable_mesh")) {
    use_loadable_mesh = true;
    cerr << ";; Using loadable mesh" << endl;
  }
  if (vm.count("use_collision")) {
    use_collision = true;
    cerr << ";; Using simple_geometry" << endl;
  }
  if (vm.count("add_normal")) {
    remove_normal = false;
    cerr << ";; Adding normals from mesh" << endl;
  }
  if (vm.count("robot_name")) {
    vector<string> aa = vm["robot_name"].as< vector<string> >();
    arobot_name = aa[0];
  }

  cerr << ";; Input file is: "
       <<  input_file << endl;
  string xml_string;
  fstream xml_file(input_file.c_str(), fstream::in);
  while ( xml_file.good() )
  {
    string line;
    getline( xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();

#if URDFDOM_1_0_0_API
  ModelInterfaceSharedPtr robot;
#else
  boost::shared_ptr<ModelInterface> robot;
#endif
  if( xml_string.find("<COLLADA") != string::npos )
  {
    ROS_DEBUG("Parsing robot collada xml string");
    robot = parseCollada(xml_string);
  }
  else
  {
    ROS_DEBUG("Parsing robot urdf xml string");
    robot = parseURDF(xml_string);
    input_file.clear(); // this file is urdf
  }

  if (!robot){
    cerr << "ERROR: Model Parsing the xml failed" << endl;
    return -1;
  }

  if (arobot_name.empty()) {
    arobot_name = robot->getName();
  }
  if (output_file.empty()) {
    if(pconfig_file.empty()) {
      output_file =  arobot_name + ".l";
    } else {
      // assume existing arguments of positional1 and positional2
      output_file = pconfig_file;
      pconfig_file.clear();
    }
  }
  if (config_file.empty() && !pconfig_file.empty()) {
    config_file = pconfig_file;
  }
  if (!config_file.empty()) {
    cerr << ";; Config file is: "
	 <<  config_file << endl;
  }
  cerr << ";; Output file is: "
       <<  output_file << endl;
  cerr << ";; robot_name is: "
       <<  arobot_name << endl;

  ModelEuslisp eusmodel(robot);
  eusmodel.setRobotName(arobot_name);
  eusmodel.setUseCollision(use_collision);
  eusmodel.setUseSimpleGeometry(use_simple_geometry);
  eusmodel.setUseLoadbleMesh(use_loadable_mesh);
  eusmodel.setRemoveNormal(remove_normal);
  eusmodel.collada_file = input_file;
  //eusmodel.setAddJointSuffix();
  //eusmodel.setAddLinkSuffix();

  if (!config_file.empty()) {
    eusmodel.readYaml(config_file);
  }
  eusmodel.writeToFile (output_file);
}
