#include <iostream>
#include <vector>
#include <math.h>
#include <sys/utsname.h>
using namespace std;

#include "dae.h"
#include "dom/domCOLLADA.h"
#ifdef __dom150COLLADA_h__
using namespace ColladaDOM150;
#endif

#include <fstream>
#include "yaml-cpp/yaml.h"

#include <boost/foreach.hpp>

extern "C" {
#include <qhull/qhull_a.h>
}

#include "rospack/rospack.h"

daeDocument *g_document;
DAE* g_dae = NULL;
float g_scale = 1.0;
bool use_technique_limit = true;
vector<pair<string, string> > g_all_link_names;

// returns max offsset value
unsigned int getMaxOffset( domInput_local_offset_Array &input_array )
{
  unsigned int maxOffset = 0;
  for ( unsigned int i = 0; i < input_array.getCount(); i++ ) {
    if ( input_array[i]->getOffset() > maxOffset ) {
      maxOffset = (unsigned int)input_array[i]->getOffset();
    }
  }
  return maxOffset;
}

void writeTriangle(FILE *fp, domGeometry *thisGeometry, const char* robot_name) {
  std::vector<coordT> points;

  // get mesh
  domMesh *thisMesh = thisGeometry->getMesh();
  int triangleElementCount = thisMesh?(int)(thisMesh->getTriangles_array().getCount()):0;

  fprintf(fp, "(defclass %s_%s\n", robot_name, thisGeometry->getId());
  fprintf(fp, "  :super body\n");
  fprintf(fp, "  :slots ())\n");
  fprintf(fp, "(defmethod %s_%s\n", robot_name, thisGeometry->getId());
  if ( thisMesh == NULL || triangleElementCount == 0 )  {
    fprintf(fp, "  (:init (&key (name))\n");
    fprintf(fp, "         (replace-object self (make-cube 10 10 10))\n");
    fprintf(fp, "         (if name (send self :name name))\n");
    fprintf(fp, "         self)\n");
    fprintf(fp, "   )\n");
    return;
  }
  if (thisGeometry->getName()){
    fprintf(fp, "  (:init (&key (name \"%s\"))\n", thisGeometry->getName());
  } else {
    fprintf(fp, "  (:init (&key (name))\n");
  }
  fprintf(fp, "         (replace-object self (send self :qhull-faceset))\n");
  fprintf(fp, "         (if name (send self :name name))\n");
  fprintf(fp, "         self)\n");
  fprintf(fp, "  (:draw (vwr)\n");
  fprintf(fp, "   (let ((mat (send (send self :worldcoords) :4x4))\n");
  fprintf(fp, "#+:jsk\n");
  fprintf(fp, "         (glcon (cdr (assq (sys:thread-self)\n");
  fprintf(fp, "                           ((send vwr :viewsurface) . gl::glcon))))\n");
  fprintf(fp, "#-:jsk\n");
  fprintf(fp, "         (glcon ((send vwr :viewsurface) . gl::glcon))\n");
  fprintf(fp, "         newlis)\n");
  fprintf(fp, "#+:jsk\n");
  fprintf(fp, "     (sys::mutex-lock gl::*opengl-lock*)\n");
  fprintf(fp, "     (send vwr :viewsurface :makecurrent)\n");
  fprintf(fp, "     (gl::glPushAttrib gl::GL_ALL_ATTRIB_BITS)\n");
  fprintf(fp, "     (gl::glPushMatrix)\n");
  fprintf(fp, "     (gl::glMultMatrixf (array-entity (transpose mat gl::*temp-matrix*)))\n");
  fprintf(fp, "     (if (setq newlis (cdr (assq glcon (get self :GL-DISPLAYLIST-ID))))\n");
  fprintf(fp, "         (progn\n");
  fprintf(fp, "           (gl::glCallList newlis)\n");
  fprintf(fp, "           )\n");
  fprintf(fp, "       (progn\n");
  fprintf(fp, "         (setq newlis (gl::glGenLists 1))\n");
  fprintf(fp, "         (gl::glNewList newlis gl::GL_COMPILE)\n");
  // Triangulation
  // based on http://www.shader.jp/xoops/html/modules/mydownloads/singlefile.php?cid=5&lid=6
  for(int currentTriangle=0;currentTriangle<triangleElementCount;currentTriangle++)
    {
      domTriangles* thisTriangles = thisMesh->getTriangles_array().get(currentTriangle);

      // NEED FIX, we mut get matriar target name from database
      domMaterial* thisMaterial; string materialTarget;
      materialTarget = string(thisGeometry->getId())+string(".mat");
      thisMaterial = daeSafeCast<domMaterial>(g_dae->getDatabase()->idLookup(materialTarget, g_document));
      if ( thisMaterial == NULL ) {
	materialTarget = string(thisGeometry->getId())+string("_mat");
	thisMaterial = daeSafeCast<domMaterial>(g_dae->getDatabase()->idLookup(materialTarget, g_document));
      }
      if ( thisMaterial == NULL ) {
	fprintf(stderr, "Could not find material %s, use 0.8 0.8 0.8\n", materialTarget.c_str());
	fprintf(fp, "         (gl::glColor3fv (float-vector 0.1 0.1 0.1))\n");
	fprintf(fp, "         (gl::glMaterialfv gl::GL_FRONT gl::GL_AMBIENT (float-vector %f %f %f %f))\n", 0.8, 0.8, 0.8, 1.0);
	fprintf(fp, "         (gl::glMaterialfv gl::GL_FRONT gl::GL_DIFFUSE (float-vector %f %f %f %f))\n", 0.8, 0.8, 0.8, 1.0);
      } else {
	domInstance_effect* thisInstanceEffect = thisMaterial->getInstance_effect();
	domEffect* thisEffect = daeSafeCast<domEffect>(g_dae->getDatabase()->idLookup(thisInstanceEffect->getUrl().id(),g_document));

	fprintf(fp, "         (gl::glColor3fv (float-vector 0.1 0.1 0.1))\n");
	// euslisp uses diffuse for ambient
	fprintf(fp, "         (gl::glMaterialfv gl::GL_FRONT gl::GL_AMBIENT (float-vector %f %f %f %f))\n",
		thisEffect->getFx_profile_array()[0]->getProfile_COMMON()->getTechnique()->getPhong()->getDiffuse()->getColor()->getValue()[0],
		thisEffect->getFx_profile_array()[0]->getProfile_COMMON()->getTechnique()->getPhong()->getDiffuse()->getColor()->getValue()[1],
		thisEffect->getFx_profile_array()[0]->getProfile_COMMON()->getTechnique()->getPhong()->getDiffuse()->getColor()->getValue()[2],
		thisEffect->getFx_profile_array()[0]->getProfile_COMMON()->getTechnique()->getPhong()->getDiffuse()->getColor()->getValue()[3]);
	fprintf(fp, "         (gl::glMaterialfv gl::GL_FRONT gl::GL_DIFFUSE (float-vector %f %f %f %f))\n",
		thisEffect->getFx_profile_array()[0]->getProfile_COMMON()->getTechnique()->getPhong()->getDiffuse()->getColor()->getValue()[0],
		thisEffect->getFx_profile_array()[0]->getProfile_COMMON()->getTechnique()->getPhong()->getDiffuse()->getColor()->getValue()[1],
		thisEffect->getFx_profile_array()[0]->getProfile_COMMON()->getTechnique()->getPhong()->getDiffuse()->getColor()->getValue()[2],
		thisEffect->getFx_profile_array()[0]->getProfile_COMMON()->getTechnique()->getPhong()->getDiffuse()->getColor()->getValue()[3]);
      }

      int numberOfInputs = (int)getMaxOffset(thisTriangles->getInput_array()) +1;// offset
      int numberOfTriangles = (int)(thisTriangles->getP()->getValue().getCount() / numberOfInputs);	// elements

      // offset of index
      unsigned int offset = 0;
      int texoffset = -255, noroffset = -255;
      for(unsigned int i=0;i<thisTriangles->getInput_array().getCount();i++)
        {
          if(strcmp(thisTriangles->getInput_array()[i]->getSemantic(), "VERTEX")==0)
            offset = thisTriangles->getInput_array()[i]->getOffset();
          if(strcmp(thisTriangles->getInput_array()[i]->getSemantic(), "TEXCOORD")==0)
            texoffset = thisTriangles->getInput_array()[i]->getOffset();
          if(strcmp(thisTriangles->getInput_array()[i]->getSemantic(), "NORMAL")==0)
            noroffset = thisTriangles->getInput_array()[i]->getOffset();
        }
      fprintf(fp, "         ;; draw %d triangles\n", numberOfTriangles);
      fprintf(fp, "         (gl::glBegin gl::GL_TRIANGLES)\n");
      for(int i=0;i<numberOfTriangles;i++)
        {
          int index = thisTriangles->getP()->getValue().get(i*numberOfInputs+offset);
          int sourceElements = thisMesh->getSource_array().getCount();
	  // check for null <source /> tag
	  for (int j = 0; j < thisMesh->getSource_array().getCount(); j++) {
	      if ( thisMesh->getSource_array()[j]->getFloat_array()==NULL ) {
		  sourceElements--;
	      }
	  }

          // normal
          if ( sourceElements  > 1 ) {
            if(noroffset==-255)
              {
                // normal vectur shares same index of vertices
                fprintf(fp, "         (gl::glNormal3fv (float-vector %f %f %f))\n",
                        thisMesh->getSource_array()[1]->getFloat_array()->getValue().get(index*3),
                        thisMesh->getSource_array()[1]->getFloat_array()->getValue().get(index*3+1),
                        thisMesh->getSource_array()[1]->getFloat_array()->getValue().get(index*3+2)
                        );
              }
            else
              {
                // index normal vector is indicated in <p></p>
                int norindex = thisTriangles->getP()->getValue().get(i*numberOfInputs+noroffset);
                fprintf(fp, "         (gl::glNormal3fv (float-vector %f %f %f))\n",
                        thisMesh->getSource_array()[1]->getFloat_array()->getValue().get(norindex*3),
                        thisMesh->getSource_array()[1]->getFloat_array()->getValue().get(norindex*3+1),
                        thisMesh->getSource_array()[1]->getFloat_array()->getValue().get(norindex*3+2)
                        );
              }
          } else {
            if ( i % 3 == 0 ) {
              int normal_index_0 = thisTriangles->getP()->getValue().get((i+0)*numberOfInputs+offset);
              int normal_index_1 = thisTriangles->getP()->getValue().get((i+1)*numberOfInputs+offset);
              int normal_index_2 = thisTriangles->getP()->getValue().get((i+2)*numberOfInputs+offset);
              float a0 =
                thisMesh->getSource_array()[0]->getFloat_array()->getValue().get((normal_index_2)*3+0) -
                thisMesh->getSource_array()[0]->getFloat_array()->getValue().get((normal_index_0)*3+0);
              float a1 =
                thisMesh->getSource_array()[0]->getFloat_array()->getValue().get((normal_index_2)*3+1) -
                thisMesh->getSource_array()[0]->getFloat_array()->getValue().get((normal_index_0)*3+1);
              float a2 =
                thisMesh->getSource_array()[0]->getFloat_array()->getValue().get((normal_index_2)*3+2) -
                thisMesh->getSource_array()[0]->getFloat_array()->getValue().get((normal_index_0)*3+2);
              float b0 =
                thisMesh->getSource_array()[0]->getFloat_array()->getValue().get((normal_index_1)*3+0) -
                thisMesh->getSource_array()[0]->getFloat_array()->getValue().get((normal_index_0)*3+0);
              float b1 =
                thisMesh->getSource_array()[0]->getFloat_array()->getValue().get((normal_index_1)*3+1) -
                thisMesh->getSource_array()[0]->getFloat_array()->getValue().get((normal_index_0)*3+1);
              float b2 =
                thisMesh->getSource_array()[0]->getFloat_array()->getValue().get((normal_index_1)*3+2) -
                thisMesh->getSource_array()[0]->getFloat_array()->getValue().get((normal_index_0)*3+2);
              float aa = sqrt(a0*a0+a1*a1+a2*a2); a0 /= aa; a1 /= aa; a2 /= aa;
              float bb = sqrt(b0*b0+b1*b1+b2*b2); b0 /= bb; b1 /= bb; b2 /= bb;
	      if ( !(isnan(a0*a1*a2*b0*b1*b2) || isinf(a0*a1*a2*b0*b1*b2)) ) {
		fprintf(fp, "         (gl::glNormal3fv (float-vector %f %f %f))\n",
			(a1*b2 - a2*b1), (a2*b0 - a0*b2), (a0*b1 - a1*b0));
	      } else {
		fprintf(stderr, "calcNormal error : %f %f %f -> %f -> %f %f %f\n", a0, a1, a2, aa, a0, a1, a2);
		fprintf(stderr, "calcNormal error : %f %f %f -> %f -> %f %f %f\n", b0, b1, b2, bb, b0, b1, b2);
	      }
            }
          }

          if ( sourceElements  > 2 ) {
            // texture coordinates
            if(texoffset!=-255)
              {
                int texindex = thisTriangles->getP()->getValue().get(i*numberOfInputs+texoffset);
                fprintf(fp, "         (gl::glTexCoord2f %f %f)\n",
                        thisMesh->getSource_array()[2]->getFloat_array()->getValue().get(texindex*2),
                        thisMesh->getSource_array()[2]->getFloat_array()->getValue().get(texindex*2+1)
                        );
              }
          }

          // vertex vector
          float a0,a1,a2;
          a0 = thisMesh->getSource_array()[0]->getFloat_array()->getValue().get(index*3);
          a1 = thisMesh->getSource_array()[0]->getFloat_array()->getValue().get(index*3+1);
          a2 = thisMesh->getSource_array()[0]->getFloat_array()->getValue().get(index*3+2);
          fprintf(fp, "         (gl::glVertex3fv (float-vector %f %f %f))\n",
                  g_scale*1000*a0,g_scale*1000*a1,g_scale*1000*a2);
          // store vertex vector to qhull
          points.push_back(a0);
          points.push_back(a1);
          points.push_back(a2);
        }
      fprintf(fp, "         (gl::glEnd)\n");
    }
  fprintf(fp, "         (gl::glEndList)\n");
  fprintf(fp, "         (setf (get self :GL-DISPLAYLIST-ID)\n");
  fprintf(fp, "               (cons (cons glcon newlis)\n");
  fprintf(fp, "                     (get self :GL-DISPLAYLIST-ID)))\n");
  fprintf(fp, "         (setq newlis nil)))\n");
  fprintf(fp, "     (gl::glPopMatrix)\n");
  fprintf(fp, "     (gl::glPopAttrib)\n");
  fprintf(fp, "#+:jsk\n");
  fprintf(fp, "     (sys::mutex-unlock gl::*opengl-lock*)\n");
  fprintf(fp, "     (unless newlis (send self :draw vwr))\n");
  fprintf(fp, "     ))\n");

  // do qhull
  if ( points.size() > 0 ) {
      char qhull_attr[] = "qhull C-0.001";
      int ret = qh_new_qhull (3, points.size()/3, &points[0], 0, qhull_attr, NULL, stderr);
      fprintf(fp, "  (:qhull-faceset ()\n");
      if ( ret ) {
        fprintf(fp, "   (instance faceset :init :faces (list\n");
	for (unsigned int i = 0; i < points.size()/9; i++ ) {
          fprintf(fp, "    (instance face :init :vertices (list (float-vector %f %f %f) (float-vector %f %f %f) (float-vector %f %f %f)))\n",
		  g_scale*1000*points[i*9+0], g_scale*1000*points[i*9+1], g_scale*1000*points[i*9+2],
		  g_scale*1000*points[i*9+3], g_scale*1000*points[i*9+4], g_scale*1000*points[i*9+5],
		  g_scale*1000*points[i*9+6], g_scale*1000*points[i*9+7], g_scale*1000*points[i*9+8]);
	}
	fprintf(fp, "    ))\n");
      } else {
        fprintf(fp, "   ;; qhull %zd -> %d faces\n", points.size()/3, qh num_facets);
        fprintf(fp, "   (instance faceset :init :faces (list\n");
        // get faces
        facetT *facet;
        vertexT *vertex, **vertexp;
        FORALLfacets {
          fprintf(fp, "    (instance face :init :vertices (list");
          setT *vertices = qh_facet3vertex(facet); // ccw?
          FOREACHvertex_(vertices) {
            fprintf(fp, " (float-vector %f %f %f)", g_scale*1000*vertex->point[0], g_scale*1000*vertex->point[1], g_scale*1000*vertex->point[2]);
          }
          fprintf(fp, "))\n");
          qh_settempfree(&vertices);
        }
        fprintf(fp, "    ))\n");
      }
      fprintf(fp, "   )\n");
      qh_freeqhull(!qh_ALL);
      int curlong, totlong;    // memory remaining after qh_memfreeshort
      qh_memfreeshort (&curlong, &totlong);    // free short memory and memory allocator
      if (curlong || totlong) {
        fprintf (stderr, "qhull internal warning (user_eg, #1): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
      }
  }
  fprintf(fp, "  )\n\n");

  // Polylist
  int polylistElementCount = (int)(thisMesh->getPolylist_array().getCount());
  //Polygons
  int polygonesElementCount = (int)(thisMesh->getPolygons_array().getCount());
  assert(polylistElementCount==0);
  assert(polygonesElementCount==0);
}

void writeGeometry(FILE *fp, daeDatabase *thisDatabase, const char* robot_name) {
  // number of geometry
  int geometryElementCount =  thisDatabase->getElementCount(NULL, "geometry", NULL);
  for(int currentGeometry=0;currentGeometry<geometryElementCount;currentGeometry++) {
    // get current geometry
    domGeometry *thisGeometry;
    thisDatabase->getElement((daeElement**)&thisGeometry, currentGeometry, NULL, "geometry");

    fprintf(stderr, "geometry %d id:%s (%s)\n",
            currentGeometry, thisGeometry->getId(), thisGeometry->getName());

    // write geometry information
    writeTriangle(fp, thisGeometry, robot_name);
  }
}

//
domJoint *findJointFromName(const char *jointName) {
  int jointElementCount;
  jointElementCount = g_dae->getDatabase()->getElementCount(NULL, "joint", NULL);
  for(int currentJoint=0;currentJoint<jointElementCount;currentJoint++) {
    domJoint *thisJoint = NULL;
    // get current geometry
    g_dae->getDatabase()->getElement((daeElement**)&thisJoint, currentJoint, NULL, "joint");
    if ( thisJoint == NULL ) {
      break;
    }
    if ( thisJoint->getSid() == NULL ) {
      break;
    }
    string jointSid_str = string(((domKinematics_model *)(thisJoint->getParentElement()->getParentElement()))->getId())+string("/")+string(thisJoint->getSid());
    if ( jointSid_str.compare(jointName) == 0 ||
         string(thisJoint->getName()).compare(jointName) == 0 ) {
      return thisJoint;
    }
  }
  fprintf(stderr, "\n[ERROR] Counld not found joint [%s]\n", jointName);
  fprintf(stderr, "[ERROR] you have to set joint name from followings\n");
  for(int currentJoint=0;currentJoint<jointElementCount;currentJoint++) {
    domJoint *thisJoint = NULL;
    g_dae->getDatabase()->getElement((daeElement**)&thisJoint, currentJoint, NULL, "joint");
    fprintf(stderr, "%d : ", currentJoint);
    if ( thisJoint != NULL ) fprintf(stderr, "%s ", thisJoint->getName());
    if ( thisJoint->getSid() != NULL ) {
      string jointSid_str = string(((domKinematics_model *)(thisJoint->getParentElement()->getParentElement()))->getId())+string("/")+string(thisJoint->getSid());
      fprintf(stderr, "(%s)", jointSid_str.c_str());
    }
    fprintf(stderr, "\n");
  }
  exit(1);
}

domLink *findNonFixChildLink(domLink *thisLink) {
  int attachmentCount = (int)(thisLink->getAttachment_full_array().getCount());
  for(int currentAttachment=0;currentAttachment<attachmentCount;currentAttachment++){
    domJoint *thisJoint = findJointFromName(thisLink->getAttachment_full_array()[currentAttachment]->getJoint());
    domAxis_constraint_Array jointAxis_array;
    if ( thisJoint->getPrismatic_array().getCount() > 0 ) {
      jointAxis_array = thisJoint->getPrismatic_array();
    } else if ( thisJoint->getRevolute_array().getCount() > 0 ) {
      jointAxis_array = thisJoint->getRevolute_array();
    }
    //
    if (jointAxis_array[0]->getLimits() &&
        jointAxis_array[0]->getLimits()->getMin()->getValue() == 0 &&
        jointAxis_array[0]->getLimits()->getMax()->getValue() == 0 ) {
      return thisLink->getAttachment_full_array()[currentAttachment]->getLink();
    } else {
      return thisLink;
    }
  }
  return NULL;
}

domLink *findChildLinkFromJointName(const char *jointName) {
  domJoint* thisJoint = findJointFromName(jointName);
  string jointSid_str = string(((domKinematics_model *)(thisJoint->getParentElement()->getParentElement()))->getId())+string("/")+string(thisJoint->getSid());
  int linkElementCount = g_dae->getDatabase()->getElementCount(NULL, "link", NULL);
  for(int currentLink=0;currentLink<linkElementCount;currentLink++) {
    domLink *thisLink;
    g_dae->getDatabase()->getElement((daeElement**)&thisLink, currentLink, NULL, "link");
    for(int currentAttachment=0;currentAttachment<(int)(thisLink->getAttachment_full_array().getCount());currentAttachment++){
      if ( jointSid_str.compare(thisLink->getAttachment_full_array()[currentAttachment]->getJoint()) == 0 ) {
        domLink *childLink = thisLink->getAttachment_full_array()[currentAttachment]->getLink();
        return childLink;
      }
    }
  }
  fprintf(stderr, "Counld not found joint %s\n", jointName);
  exit(1);
}

const char* findLinkName(const char *link_name) {
  for(vector<pair<string, string> >::iterator it=g_all_link_names.begin();it!=g_all_link_names.end();it++){
    if ( it->first.compare(link_name) == 0 ) {
      return it->second.c_str();
    }
  }
  return link_name;
}

// write euslisp joint instance from jointSid, parentLink, childLink
void writeJoint(FILE *fp, const char *jointSid, domLink *parentLink, domLink *childLink) {
  //
  // get number of joints
  domJoint *thisJoint = findJointFromName(jointSid);
  fprintf(fp, "     (setq %s\n", thisJoint->getName());
  fprintf(fp, "           (instance %s :init\n",
          (thisJoint->getPrismatic_array().getCount()>0)?"linear-joint":"rotational-joint");
  fprintf(fp, "                     :name \"%s\"\n", thisJoint->getName());
  fprintf(fp, "                     :parent-link %s :child-link %s\n", parentLink->getName(), childLink->getName());
  domAxis_constraint_Array jointAxis_array;
  float axis[3], min = FLT_MAX, max = -FLT_MAX, scale = 1.0;
  if ( thisJoint->getPrismatic_array().getCount() > 0 ) {
    jointAxis_array = thisJoint->getPrismatic_array();
    if ( jointAxis_array[0]->getLimits() ) {
      scale = 1000;
      min = scale*jointAxis_array[0]->getLimits()->getMin()->getValue();
      max = scale*jointAxis_array[0]->getLimits()->getMax()->getValue();
    }
  } else if ( thisJoint->getRevolute_array().getCount() > 0 ) {
    jointAxis_array = thisJoint->getRevolute_array();
    if ( jointAxis_array[0]->getLimits() ) {
      min = jointAxis_array[0]->getLimits()->getMin()->getValue();
      max = jointAxis_array[0]->getLimits()->getMax()->getValue();
    }
  }
  fprintf(stderr, "  writeJoint %s\n", thisJoint->getName());
  fprintf(stderr, "    min = %f, max = %f\n", min, max);
  // dump :max-joint-velocity of eus file from <speed> tag of collada file
  domMotion *thisMotion;
  g_dae->getDatabase()->getElement((daeElement**)&thisMotion, 0, NULL, "motion");
  if( !!thisMotion->getTechnique_common() ) {
    for(size_t i = 0; i < thisMotion->getTechnique_common()->getAxis_info_array().getCount(); ++i) {
      domMotion_axis_infoRef motion_axis_info = thisMotion->getTechnique_common()->getAxis_info_array()[i];
      // Sid name from Motion axis info
      string axis_info_name(daeSafeCast<domKinematics_axis_info>(daeSidRef(motion_axis_info->getAxis(), thisMotion->getInstance_articulated_system()->getUrl().getElement()).resolve().elt)->getAxis());
      // Sid name from joint axis info
      string joint_name(string(daeSafeCast<domKinematics_model>(thisJoint->getParentElement()->getParentElement())->getId()) // kinamtics_model's id
			+"/"+ string(thisJoint->getSid()) +"/"+ jointAxis_array[0]->getSid());
      if (axis_info_name == joint_name && // if thisJoint corresponds to Motion_axis
	  thisMotion->getTechnique_common()->getAxis_info_array()[i]->getSpeed()) {
	fprintf(fp, "                     :max-joint-velocity %f\n", fabs(thisMotion->getTechnique_common()->getAxis_info_array()[i]->getSpeed()->getFloat()->getValue()));
      }
    }
  }
  // use safety controller data
  domKinematics *thisKinematics;
  g_dae->getDatabase()->getElement((daeElement**)&thisKinematics, 0, NULL, "kinematics");
  if( !!thisKinematics->getTechnique_common() ) {
    for(size_t i = 0; i < thisKinematics->getTechnique_common()->getAxis_info_array().getCount(); ++i) {
      domKinematics_axis_infoRef kinematics_axis_info = thisKinematics->getTechnique_common()->getAxis_info_array()[i];
      // Sid name from Motion axis info
      string axis_info_name(string(kinematics_axis_info->getAxis()));
      // Sid name from joint axis info
      string joint_name(string(daeSafeCast<domKinematics_model>(thisJoint->getParentElement()->getParentElement())->getId()) // kinamtics_model's id
			+"/"+ string(thisJoint->getSid()) +"/"+ jointAxis_array[0]->getSid());
      if (axis_info_name == joint_name && // if thisJoint corresponds to kinematics_axis
          thisKinematics->getTechnique_common()->getAxis_info_array()[i]->getLimits()) {
        if (use_technique_limit) {
          min = scale*thisKinematics->getTechnique_common()->getAxis_info_array()[i]->getLimits()->getMin()->getFloat()->getValue();
          max = scale*thisKinematics->getTechnique_common()->getAxis_info_array()[i]->getLimits()->getMax()->getFloat()->getValue();
          fprintf(stderr, "    min = %f, max = %f (safety)\n", min, max);
        }
      }
    }
  }

  axis[0] = jointAxis_array[0]->getAxis()->getValue()[0];
  axis[1] = jointAxis_array[0]->getAxis()->getValue()[1];
  axis[2] = jointAxis_array[0]->getAxis()->getValue()[2];
  cerr << "    writeJoint " << thisJoint->getName() << ", parent = " << parentLink->getName() << ", child = " << childLink->getName()  << ", limit = " << min << "/" << max << endl;
  fprintf(fp, "                     :axis (let ((tmp-axis (float-vector %f %f %f))) (if (eps= (norm tmp-axis) 0.0) (float-vector 1 0 0) tmp-axis))\n", axis[0], axis[1], axis[2]);
  fprintf(fp, "                    ");
  fprintf(fp, " :min "); if (min == FLT_MAX) fprintf(fp, "*-inf*"); else fprintf(fp, "%f", min);
  fprintf(fp, " :max "); if (max ==-FLT_MAX) fprintf(fp,  "*inf*"); else fprintf(fp, "%f", max);
  fprintf(fp, "\n");
  fprintf(fp, "                     ))\n");
}

void writeKinematics(FILE *fp, domLink::domAttachment_full_Array thisAttachmentArray) {
  for(unsigned int currentAttachment=0;currentAttachment < thisAttachmentArray.getCount();currentAttachment++) {
    domLinkRef thisLink = thisAttachmentArray[currentAttachment]->getLink();
    writeKinematics(fp, thisLink->getAttachment_full_array());

    cerr << "writeKinematics " << thisLink->getName() << ", childlen = " << thisLink->getAttachment_full_array().getCount() << endl;
    for(unsigned int currentAttachment2=0;currentAttachment2 < (unsigned int)(thisLink->getAttachment_full_array().getCount());currentAttachment2++) {
      writeJoint(fp, thisLink->getAttachment_full_array()[currentAttachment2]->getJoint(),
                 thisLink,
                 thisLink->getAttachment_full_array()[currentAttachment2]->getLink()
                 );
    }
  }
}

void writeTransform(FILE *fp, const char *indent, const char *name, domNode *thisNode, int targetCount, const char *parentName=":world") {
  domTranslateRef thisTranslate = NULL;
  domRotateRef thisRotate = NULL;
  domMatrixRef thisMatrix = NULL;

  domTranslate_Array translateArray = thisNode->getTranslate_array();
  int translateCount = translateArray.getCount();
  domRotate_Array rotateArray = thisNode->getRotate_array();
  int rotateCount = rotateArray.getCount();
  domMatrix_Array matrixArray = thisNode->getMatrix_array();
  int matrixCount = matrixArray.getCount();

  fprintf(fp, "\n");
  fprintf(fp, "%s;; writeTransform(name=%s,domNode=%s,targetCount=%d,parent=%s), translateCount=%d, rotateCount=%d, matrixCount=%d\n", indent, name, thisNode->getName(), targetCount, parentName, translateCount, rotateCount, matrixCount);
  fprintf(stderr, "%s;; writeTransform(name=%s,domNode=%s,targetCount=%d,parent=%s), translateCount=%d, rotateCount=%d, matrixCount=%d\n", indent, name, thisNode->getName(), targetCount, parentName, translateCount, rotateCount, matrixCount);

  for (int i=0, currentTranslate=0, currentRotate=0, skipTranslate=0, skipRotate=0; i < min(translateCount, rotateCount); i++, currentTranslate++, currentRotate++){
    if ( translateArray[currentTranslate]->getSid() ) {
      fprintf(stderr, " skip translate %s : %s\n", name, translateArray[currentTranslate]->getSid());
      skipTranslate++;
      currentTranslate++;
    }
    if ( rotateArray[currentRotate]->getSid() ) {
      fprintf(stderr, " skip rotate %s : %s\n", name, rotateArray[currentRotate]->getSid());
      skipRotate++;
      currentRotate++;
    }
    thisTranslate = translateArray[currentTranslate];
    thisRotate = rotateArray[currentRotate];

    if ( skipTranslate + skipRotate == targetCount ) {
      fprintf(fp, "%s(send %s :transform\n%s      (make-coords :pos (float-vector %f %f %f)\n%s                   :angle %f :axis (float-vector %f %f %f)) %s)\n",
	      indent, name, indent, 
	      thisTranslate ? 1000*thisTranslate->getValue()[0] : 0,
	      thisTranslate ? 1000*thisTranslate->getValue()[1] : 0,
	      thisTranslate ? 1000*thisTranslate->getValue()[2] : 0,
	      indent,
	      thisRotate ? (thisRotate->getValue()[3])*M_PI/180.0 : 0,
	      thisRotate ? thisRotate->getValue()[0] : 0,
	      thisRotate ? thisRotate->getValue()[1] : 0,
	      thisRotate ? thisRotate->getValue()[2] : 1,
	      parentName);
    }
  }
  for (int i=0, currentMatrix=0, skipMatrix=0; i < matrixCount; i++, currentMatrix++){
    thisMatrix = matrixArray[currentMatrix];

    if ( skipMatrix == targetCount ) {
      fprintf(fp, "%s(send %s :transform\n%s      (make-coords :4x4 #2f((%f %f %f %f)(%f %f %f %f)(%f %f %f %f)(%f %f %f %f))) %s)\n",
	      indent, name, indent,
	      thisMatrix->getValue()[0], thisMatrix->getValue()[1], thisMatrix->getValue()[2], 1000*thisMatrix->getValue()[3],
	      thisMatrix->getValue()[4], thisMatrix->getValue()[5], thisMatrix->getValue()[6], 1000*thisMatrix->getValue()[7],
	      thisMatrix->getValue()[8], thisMatrix->getValue()[9], thisMatrix->getValue()[10], 1000*thisMatrix->getValue()[11],
	      thisMatrix->getValue()[12], thisMatrix->getValue()[13], thisMatrix->getValue()[14], thisMatrix->getValue()[15],
	      parentName);
    }
  }
}

// find Link from kinematic
domLink* findLinkfromKinematics (domLink* thisLink, const std::string& link_name)
{
  if (thisLink->getName()==link_name) return thisLink;
  for(size_t ii = 0; ii < thisLink->getAttachment_full_array().getCount(); ++ii) {
    domLink* tmpLink = findLinkfromKinematics(thisLink->getAttachment_full_array()[ii]->getLink(), link_name);
    if (tmpLink) return tmpLink;
  }
  return NULL;
}

std::string getSensorType (const domExtraRef pextra) {
  // get sensor_type from extra tag
  std::string sensor_type;
  for (size_t ii = 0; ii < g_dae->getDatabase()->getElementCount(NULL, "extra", NULL); ii++) {
    domExtra *tmpextra;
    g_dae->getDatabase()->getElement((daeElement**)&tmpextra, ii, NULL, "extra");
    if (tmpextra->getType() == std::string("library_sensors")) {
      for (size_t icon = 0; icon < tmpextra->getTechnique_array()[0]->getContents().getCount(); icon++) {
        if ((std::string("#") + tmpextra->getTechnique_array()[0]->getContents()[icon]->getAttribute("id")) ==
            pextra->getTechnique_array()[0]->getChild("instance_sensor")->getAttribute("url")) {
          sensor_type = tmpextra->getTechnique_array()[0]->getContents()[icon]->getAttribute("type");
        }
      }
    }
  }
  return sensor_type;
}

void writeNodes(FILE *fp, domNode_Array thisNodeArray, domRigid_body_Array thisRigidbodyArray, const char* robot_name) {
  int nodeArrayCount = thisNodeArray.getCount();
  for(int currentNodeArray=0;currentNodeArray<nodeArrayCount;currentNodeArray++) {
    domNode *thisNode = thisNodeArray[currentNodeArray];
    string parentName = ":local";
    writeNodes(fp, thisNode->getNode_array(), thisRigidbodyArray, robot_name);

    if ( strcmp(thisNode->getName(),"visual") == 0 ) continue; //@@@ OK??
    // link
    fprintf(stderr, "writeNodes link sid:%s name:%s node_array:%zd\n",
            thisNode->getSid(), thisNode->getName(),thisNode->getNode_array().getCount() );

    domNode *geomNode = NULL;

    for (unsigned int currentNodeArray=0;currentNodeArray<thisNode->getNode_array().getCount();currentNodeArray++) {
      if (strcmp(thisNode->getNode_array()[currentNodeArray]->getName(),"visual") == 0) {
	geomNode = thisNode->getNode_array()[currentNodeArray];
      }
    }
    if ( geomNode == NULL ) {
      fprintf(stderr, "writeNodes no visual found for link sid:%s name:%s node_array:%zd\n",
	      thisNode->getSid(), thisNode->getName(),thisNode->getNode_array().getCount() );
      geomNode = thisNode;
    }
    fprintf(fp, "     ;; node id=%s, name=%s, sid=%s\n", thisNode->getId(), thisNode->getName(), thisNode->getSid());
    //fprintf(stderr, "%d\n", (int)thisRigidbodyArray.getCount());
    domRigid_body* thisRigidbody = NULL;
    for(unsigned int currentRigidbodyArray=0;currentRigidbodyArray<thisRigidbodyArray.getCount();currentRigidbodyArray++){
      if ( strcmp(thisNode->getName(),thisRigidbodyArray[currentRigidbodyArray]->getName()) == 0 )  {
	thisRigidbody = thisRigidbodyArray[currentRigidbodyArray];
      }
      //fprintf(stderr, "%s - %f (%d)\n", thisRigidbodyArray[currentRigidbodyArray]->getName(), thisRigidbodyArray[currentRigidbodyArray]->getTechnique_common()->getMass()->getValue()*1000, strcmp(thisNode->getName(),thisRigidbodyArray[currentRigidbodyArray]->getName()));
      // geometry we assume Node_array()[0] contatins geometry
    }
    fprintf(fp, "     (let (");
    if ( geomNode && geomNode->getInstance_geometry_array().getCount() > 0 ) {
      int geometryCount = geomNode->getInstance_geometry_array().getCount();
      vector<pair<domInstance_geometry*, string> > geometryNameArray;
      for(int currentGeometryCount=0;currentGeometryCount<geometryCount;currentGeometryCount++) {
	domInstance_geometry *thisGeometry  = geomNode->getInstance_geometry_array()[currentGeometryCount];
	const char * geometryName = (string("b_")+thisGeometry->getUrl().id()).c_str();
	fprintf(stderr, " geometry:%d %s\n",currentGeometryCount, geometryName);
	geometryNameArray.push_back(pair<domInstance_geometry*, string>(thisGeometry, geometryName));
      }


      for(vector<pair<domInstance_geometry*, string> >::iterator it=geometryNameArray.begin();it!=geometryNameArray.end();it++){
	fprintf(fp, " %s", it->second.c_str());
      }
      fprintf(fp, ")\n");
      fprintf(fp, "       ;; define bodyset-link for %s : %s\n", geomNode->getName(), geomNode->getId());
      for(vector<pair<domInstance_geometry *, string> >::iterator it=geometryNameArray.begin();it!=geometryNameArray.end();it++){
	domInstance_geometry *thisGeometry = it->first;
	const char * geometryName = it->second.c_str();
	fprintf(fp, "       (setq %s (instance %s_%s :init))\n",  geometryName, robot_name, thisGeometry->getUrl().id().c_str());

	// note that geometryNameCount 0 means root node and >1 indicates the coordinates of each geometry
	if ( thisNode == geomNode ) {
	  writeTransform(fp, "       ", geometryName, geomNode, 1);
	} else {
	  writeTransform(fp, "       ", geometryName, geomNode, 0);
	}
      }
      if ( geometryNameArray.size() > 0 ) {
	//int geometryNameCount = 1;
	const char *rootName = geometryNameArray.begin()->second.c_str();
	for(vector<pair<domInstance_geometry *, string> >::iterator it=geometryNameArray.begin();(++it)!=geometryNameArray.end();){
	  const char * geometryName = it->second.c_str();
	  // transform
	  // note that geometryNameCount 0 means root node and >1 indicates the coordinates of each geometry
	  //writeTransform(fp, "       ", geometryName, geomNode, geometryNameCount++);
	  fprintf(fp, "       (send %s :assoc %s)\n", rootName, geometryName);
	}
	// transform
	//writeTransform(fp, "       ", rootName, thisNode, 1);
      }
      fprintf(fp, "       (setq %s\n", thisNode->getName());
      fprintf(fp, "             (instance bodyset-link\n");
      fprintf(fp, "                       :init (make-cascoords)\n");
      fprintf(fp, "                       :bodies (list ");
      for(vector<pair<domInstance_geometry *, string> >::iterator it=geometryNameArray.begin();it!=geometryNameArray.end();it++){
	const char * geometryName = it->second.c_str();
	fprintf(fp, " %s", geometryName);
      }
      fprintf(fp, ")\n");
      fprintf(fp, "                       :name \"%s\"\n", thisNode->getName());
      if ( thisRigidbody && thisRigidbody->getTechnique_common()->getMass_frame() ) {
	fprintf(fp, "                       :weight %.3f))\n",
		/* weight : collada [kg] -> eus : [g] */
		thisRigidbody->getTechnique_common()->getMass()->getValue()*1000);
        domTranslate_Array translateArray = thisRigidbody->getTechnique_common()->getMass_frame()->getTranslate_array();
        domRotate_Array rotateArray = thisRigidbody->getTechnique_common()->getMass_frame()->getRotate_array();
        fprintf(fp, "       (let ((tmp-c-list (list\n");
        for (size_t ii = 0; ii < translateArray.getCount(); ii++) {
          domTranslateRef thisTranslate = translateArray[ii];
          fprintf(fp, "                          (make-coords :pos (float-vector %.3f %.3f %.3f) ",
                  /* centroid : collada [m] -> eus : [mm] */
                  thisTranslate->getValue()[0]*1000, thisTranslate->getValue()[1]*1000, thisTranslate->getValue()[2]*1000);
          domRotateRef thisRotate = rotateArray[ii];
          fprintf(fp, ":rot (matrix-exponent (scale %.6f (float-vector %.6f %.6f %.6f))))\n",
                  thisRotate ? thisRotate->getValue()[3]*M_PI/180.0 : 0,
                  thisRotate ? thisRotate->getValue()[0] : 0,
                  thisRotate ? thisRotate->getValue()[1] : 0,
                  thisRotate ? thisRotate->getValue()[2] : 1);
        }
        fprintf(fp, "                          ))\n");
        fprintf(fp, "             (tmp-c (make-coords)))\n");
        fprintf(fp, "         (dolist (cc tmp-c-list)\n");
        fprintf(fp, "           (setq tmp-c (send tmp-c :transform cc)))\n");
        fprintf(fp, "         (setq (%s . inertia-tensor)\n", thisNode->getName());
        fprintf(fp, "               (m* (send tmp-c :worldrot) (diagonal (float-vector %.3f %.3f %.3f)) (transpose (send tmp-c :worldrot))))\n",
                /* inertia : collada [kg m^2] -> eus : [g mm^2] */
                thisRigidbody->getTechnique_common()->getInertia()->getValue()[0]*1e9,
                thisRigidbody->getTechnique_common()->getInertia()->getValue()[1]*1e9,
                thisRigidbody->getTechnique_common()->getInertia()->getValue()[2]*1e9);
        fprintf(fp, "         (setq (%s . acentroid) (send tmp-c :worldpos))\n", thisNode->getName());
        fprintf(fp, "        )\n");
      } else {
	fprintf(fp, "                       :weight 0.0 :centroid (float-vector 0 0 0) :inertia-tensor #2f((0 0 0)(0 0 0)(0 0 0))))\n");
      }
    } else if ( (thisNode->getNode_array().getCount() > 0 &&
                 strcmp(thisNode->getNode_array()[0]->getName(),"visual") != 0 ) ||
		(thisNode->getNode_array().getCount() > 1 &&
                 strcmp(thisNode->getNode_array()[1]->getName(),"visual") != 0 ) ) {
      fprintf(fp, ")\n"); // let(
      cerr << ";; WARNING link without geometry : " << thisNode->getName() << endl;
      fprintf(fp, "       ;; define bodyset-link for %s\n", thisNode->getName());
      fprintf(fp, "       (setq %s (instance bodyset-link :init (make-cascoords) :bodies (list (make-cube 10 10 10)) :name :%s :weight 0.0 :centroid (float-vector 0 0 0) :inertia-tensor #2f((0 0 0)(0 0 0)(0 0 0))))\n", thisNode->getName(), thisNode->getName());
    } else {
      fprintf(fp, ")\n"); // let(
      cerr << ";; WARNING link without geometry nor node: " << thisNode->getName() << " geometry : " << thisNode->getInstance_geometry_array().getCount() << ", node : " << thisNode->getNode_array().getCount();;
      for (unsigned int i = 0; i < thisNode->getNode_array().getCount() ; i++) {
	cerr << ", " << thisNode->getNode_array()[i]->getName();
      }
      cerr << endl;
      fprintf(fp, "       ;; define cascaded-coords for %s\n", thisNode->getName());
      fprintf(fp, "       (setq %s (instance bodyset-link :init (make-cascoords) :name :%s :weight 0.0 :centroid (float-vector 0 0 0) :inertia-tensor #2f((0 0 0)(0 0 0)(0 0 0))))\n", thisNode->getName(), thisNode->getName());
      parentName = ":world";
    }

    //transform
    string nodeName = string(thisNode->getName());
    writeTransform(fp, "       ", nodeName.c_str(), thisNode, 0, parentName.c_str());

    fprintf(fp, "       ;;\n");

   // assoc
    for(unsigned int currentNodeArray=0;currentNodeArray<thisNode->getNode_array().getCount();currentNodeArray++) {
      if ( strcmp(thisNode->getNode_array()[currentNodeArray]->getName(),"visual") == 0 ) continue; //@@@ OK??
      domNode *thisNode2 = thisNode->getNode_array()[currentNodeArray];
      for(unsigned int geometryNameCount = 0; geometryNameCount < thisNode->getTranslate_array().getCount(); geometryNameCount++) {
      //for(int geometryNameCount = (int)thisNode->getTranslate_array().getCount()-1; geometryNameCount >=0 ; geometryNameCount--) {
	// transform
	string nodeName = string(thisNode->getName());
	string nodeName2 = string(thisNode2->getName());
	writeTransform(fp, "       ", nodeName2.c_str(), thisNode, geometryNameCount, nodeName.c_str());
      }
      fprintf(fp, "       ;;\n");
      fprintf(fp, "       (send %s :assoc %s)\n",
	      thisNode->getName(),
	      thisNode->getNode_array()[currentNodeArray]->getName());
    }
    // sensor
    if ( g_dae->getDatabase()->getElementCount(NULL, "articulated_system", NULL) > 0 ) {
      domKinematics_model *thisKinematics;
      g_dae->getDatabase()->getElement((daeElement**)&thisKinematics, 0, NULL, "kinematics_model");
      domLink* thisLink = findLinkfromKinematics(thisKinematics->getTechnique_common()->getLink_array()[0], std::string(thisNode->getName()));
      domArticulated_system *thisArticulated;
      g_dae->getDatabase()->getElement((daeElement**)&thisArticulated, 0, NULL, "articulated_system");
      for(size_t ie = 0; ie < thisArticulated->getExtra_array().getCount(); ++ie) {
	domExtraRef pextra = thisArticulated->getExtra_array()[ie];
	// find element which type is attach_sensor and is attached to thisNode
	if ( strcmp(pextra->getType(), "attach_sensor") == 0 ) {
	  daeElement* frame_origin = pextra->getTechnique_array()[0]->getChild("frame_origin");
	  if ( std::string(thisKinematics->getId())+std::string("/")+std::string(thisLink->getSid()) == frame_origin->getAttribute("link")) {
            std::string sensor_type = getSensorType(pextra);
            std::string sensor_url(pextra->getTechnique_array()[0]->getChild("instance_sensor")->getAttribute("url"));
	    std::cerr << "Sensor " << pextra->getName() << " is attached to " << thisNode->getName() << " " << sensor_type << " " << sensor_url << std::endl;
	    fprintf(fp, "       (setq %s-sensor-coords (make-cascoords :name :%s))\n", pextra->getName(), pextra->getName());
            fprintf(fp, "       (send %s-sensor-coords :put :sensor-type :%s)\n", pextra->getName(), sensor_type.c_str());
            fprintf(fp, "       (send %s-sensor-coords :put :sensor-id %s)\n", pextra->getName(), sensor_url.erase( sensor_url.find( "#sensor" ), 7 ).c_str());
	    fprintf(fp, "       (send %s-sensor-coords :transform (make-coords ", pextra->getName());
	    domTranslateRef ptrans = daeSafeCast<domTranslate>(frame_origin->getChild("translate"));
	    domRotateRef prot = daeSafeCast<domRotate>(frame_origin->getChild("rotate"));
	    if ( ptrans ) {
	      fprintf(fp, ":pos #f(");
	      for(unsigned int i=0;i<3;i++) { fprintf(fp, " %f", 1000*ptrans->getValue()[i]);}
	      fprintf(fp, ") ");
	    }
	    if ( prot ) {
	      fprintf(fp, ":axis ");
              fprintf(fp, "(let ((tmp-axis (float-vector %f %f %f))) (if (eps= (norm tmp-axis) 0.0) (float-vector 1 0 0) tmp-axis))",
                      prot->getValue()[0], prot->getValue()[1], prot->getValue()[2]);
	      fprintf(fp, " :angle");
	      fprintf(fp, " %f", prot->getValue()[3]*(M_PI/180.0));
	    }
	    fprintf(fp, "))\n");
	    fprintf(fp, "       (send %s :assoc %s-sensor-coords)\n", thisNode->getName(), pextra->getName());
	  }
	}
      }
    }
    fprintf(fp, "       )\n\n");
  }
}

bool limb_order_asc(const pair<string, size_t>& left, const pair<string, size_t>& right) { return left.second < right.second; }

void copy_euscollada_robot_class_definition (FILE *output_fp)
{
  fprintf(output_fp, ";; copy euscollada-robot class definition from euscollada/src/euscollada-robot.l\n");
  fprintf(output_fp, ";;\n");
  try {
    std::string euscollada_robot_path;
#ifdef ROSPACK_EXPORT
    rospack::ROSPack rp;
    rospack::Package *p = rp.get_pkg("euscollada");
    if (p!=NULL) euscollada_robot_path = p->path;
#else
    rospack::Rospack rp;
    std::vector<std::string> search_path;
    rp.getSearchPathFromEnv(search_path);
    rp.crawl(search_path, 1);
    std::string path;
    rp.find("euscollada",euscollada_robot_path);
#endif
    euscollada_robot_path += "/src/euscollada-robot.l";
    ifstream fin(euscollada_robot_path.c_str());
    std::string buf;
    while(fin && getline(fin, buf))
      fprintf(output_fp, "%s\n", buf.c_str());
  } catch (runtime_error &e) {
    std::cerr << "cannot resolve euscollada package path" << std::endl;
  }
  fprintf(output_fp, ";;\n");
}

int main(int argc, char* argv[]){
  FILE *output_fp;
  char *input_filename, *yaml_filename, *output_filename;
  for(int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--without-technique-limit") == 0) {
      use_technique_limit = false;
      if (i != argc-1) {
        argv[i] = argv[i+1];
      }
      argc--;
      break;
    }
  }
  switch (argc) {
  case 3:
    input_filename  = argv[1];
    output_filename = argv[2];
    yaml_filename   = NULL;
    break;
  case 4:
    input_filename  = argv[1];
    yaml_filename   = argv[2];
    output_filename = argv[3];
    break;
  default:
    fprintf(stderr, "Usage: %s <input dae filename> <input yaml filename> <output lisp filename>\n",argv[0]);
    exit(-1);
  }
  output_fp = fopen(output_filename,"w");
  if ( output_fp == NULL ) {
    fprintf(stderr, "could not write to %s\n", output_filename);
    exit(-1);
  }
  fprintf(stderr, "Convert %s to %s\n with %s", input_filename, output_filename, yaml_filename);

  // init COLLADA
  DAE dae;
  g_dae = &dae;
  int iRet = dae.load(input_filename);
  if ( iRet != DAE_OK ) {
    exit(1);
  }

  if ( g_dae->getDatabase()->getDocumentCount() != 1 ) {
    fprintf(stderr, "Number of documnet is not 1\n");
    exit(1);
  }
  g_document = g_dae->getDatabase()->getDocument((daeUInt)0);

  // read yaml
  typedef pair<vector<string>, vector<string> > link_joint;
  typedef pair<string, link_joint > link_joint_pair;
  string limb_candidates[] = {"torso", "larm", "rarm", "lleg", "rleg", "head"}; // candidates of limb names

  vector<pair<string, size_t> > limb_order;
  YAML::Node doc;
  if (yaml_filename != NULL) {
    ifstream fin(yaml_filename);
    if (fin.fail()) {
      fprintf(stderr, "%c[31m;; Could not open %s%c[m\n", 0x1b, yaml_filename, 0x1b);
    }
    YAML::Parser parser(fin);
    parser.GetNextDocument(doc);

    /* re-order limb name by lines of yaml */
    BOOST_FOREACH(string& limb, limb_candidates) {
    if ( doc.FindValue(limb) ) {
      std::cerr << limb << "@" << doc[limb].GetMark().line << std::endl;
      limb_order.push_back(pair<string, size_t>(limb, doc[limb].GetMark().line));
    }
    }
    std::sort(limb_order.begin(), limb_order.end(), limb_order_asc);
  }

  // generate limbs including limb_name, link_names, and joint_names
  vector<link_joint_pair> limbs;
  for (size_t i = 0; i < limb_order.size(); i++) {
    string limb_name = limb_order[i].first;
    vector<string> tmp_link_names, tmp_joint_names;
    try {
      const YAML::Node& limb_doc = doc[limb_name];
      for(unsigned int i = 0; i < limb_doc.size(); i++) {
	const YAML::Node& n = limb_doc[i];
	for(YAML::Iterator it=n.begin();it!=n.end();it++) {
	  string key, value; it.first() >> key; it.second() >> value;
	  tmp_joint_names.push_back(key);
	  tmp_link_names.push_back(findChildLinkFromJointName(key.c_str())->getName());
	  g_all_link_names.push_back(pair<string, string>(key, value));
	}
      }
      limbs.push_back(link_joint_pair(limb_name, link_joint(tmp_link_names, tmp_joint_names)));
    } catch(YAML::RepresentationException& e) {
    }
  }

  // get number of kinmatics
  int visualSceneCount;
  visualSceneCount = g_dae->getDatabase()->getElementCount(NULL, "visual_scene", NULL);
  fprintf(stderr, "Number of Visual Scene %d (= 1)\n", visualSceneCount); // this shoule be 1
  domVisual_scene *thisVisualscene;
  g_dae->getDatabase()->getElement((daeElement**)&thisVisualscene, 0, NULL, "visual_scene");
  int nodeCount = thisVisualscene->getNode_array().getCount();
  fprintf(stderr, "Number of Nodes %d (= 1)\n", nodeCount); // this shoule be 1
  domNode* thisNode= thisVisualscene->getNode_array()[0];

  fprintf(stderr, "Visual_scene %s\n", thisNode->getName());
  if( !!((domCOLLADA *)dae.getDom(input_filename))->getAsset()) {
      if( !!((domCOLLADA *)dae.getDom(input_filename))->getAsset()->getUnit() ) {
	  g_scale = ((domCOLLADA *)dae.getDom(input_filename))->getAsset()->getUnit()->getMeter();
      }
  }

  utsname uname_buf;
  uname(&uname_buf);
  time_t tm;
  time(&tm);
  localtime(&tm);

  fprintf(output_fp, ";;\n");
  fprintf(output_fp, ";; DO NOT EDIT THIS FILE\n");
  fprintf(output_fp, ";;\n");
  fprintf(output_fp, ";; this file is automatically generated from %s on (%s %s %s %s) at %s\n", input_filename, uname_buf.sysname, uname_buf.nodename, uname_buf.release, uname_buf.machine, ctime(&tm));
  fprintf(output_fp, ";;\n");
  fprintf(output_fp, ";; %s $ ", get_current_dir_name());for(int i=0;i<argc;i++) fprintf(output_fp, "%s ", argv[i]); fprintf(output_fp, "\n");
  fprintf(output_fp, ";;\n");
  fprintf(output_fp, "\n");
  std::string robot_name(thisNode->getName());
  if ( thisNode->getNode_array().getCount() == 0 ) {
      fprintf(output_fp, "(defun %s () (setq *%s* (instance %s-object :init)))\n", robot_name.c_str(), robot_name.c_str(), robot_name.c_str());
      fprintf(output_fp, "\n");
      fprintf(output_fp, "(defclass %s-object\n", robot_name.c_str());
      fprintf(output_fp, "  :super bodyset-link\n");
      fprintf(output_fp, "  :slots ())\n\n");
      fprintf(output_fp, "(defmethod %s-object\n", robot_name.c_str());
      fprintf(output_fp, "  (:init\n");
      fprintf(output_fp, "   (&rest args)\n");
      fprintf(output_fp, "   (let ()\n");
      fprintf(output_fp, "     (send-super* :init (make-cascoords)\n");
      fprintf(output_fp, "                  :bodies (list");

      daeDatabase *thisDatabase = g_dae->getDatabase();
      int geometryElementCount =  thisDatabase->getElementCount(NULL, "geometry", NULL);
      for(int currentGeometry=0;currentGeometry<geometryElementCount;currentGeometry++) {
	  // get current geometry
	  domGeometry *thisGeometry;
	  thisDatabase->getElement((daeElement**)&thisGeometry, currentGeometry, NULL, "geometry");

	  fprintf(output_fp, " (instance %s :init)", thisGeometry->getId());
	  fprintf(stderr, "geometry %d id:%s (%s)\n",
		  currentGeometry, thisGeometry->getId(), thisGeometry->getName());

	  // write geometry information
      }
      fprintf(output_fp, ")\n");
      fprintf(output_fp, "                  :name \"%s\"\n", robot_name.c_str());
      fprintf(output_fp, "                  args))))\n");

      writeGeometry(output_fp, g_dae->getDatabase(), robot_name.c_str());

      fprintf(output_fp, "\n\n(provide :%s \"%s/%s\")\n\n", robot_name.c_str(), get_current_dir_name(), output_filename);
      fprintf(stderr, ";; generate lisp code for body\n");
      exit(0);
  }

  copy_euscollada_robot_class_definition(output_fp);

  fprintf(output_fp, "(defun %s () (setq *%s* (instance %s-robot :init)))\n", robot_name.c_str(), robot_name.c_str(), robot_name.c_str());
  fprintf(output_fp, "\n");
  fprintf(output_fp, "(defclass %s-robot\n", robot_name.c_str());
  fprintf(output_fp, "  :super euscollada-robot\n");
  fprintf(output_fp, "  :slots (");
  // all joint and link name
  for(int currentJoint=0;currentJoint<(int)(g_dae->getDatabase()->getElementCount(NULL, "joint", NULL));currentJoint++) {
    domJoint *thisJoint;
    g_dae->getDatabase()->getElement((daeElement**)&thisJoint, currentJoint, NULL, "joint");
    fprintf(output_fp, "%s ", thisJoint->getName());
  }
  for(int currentLink=0;currentLink<(int)(g_dae->getDatabase()->getElementCount(NULL, "link", NULL));currentLink++) {
    domJoint *thisLink;
    g_dae->getDatabase()->getElement((daeElement**)&thisLink, currentLink, NULL, "link");
    fprintf(output_fp, "%s ", thisLink->getName());
  }
  // add openrave manipulator tip frame
  if ( g_dae->getDatabase()->getElementCount(NULL, "kinematics_scene", NULL) > 0 ) {
    domKinematics_scene *thisKinematicsScene;
    g_dae->getDatabase()->getElement((daeElement**)&thisKinematicsScene, 0, NULL, "kinematics_scene");
    for(size_t ias = 0; ias < thisKinematicsScene->getInstance_articulated_system_array().getCount(); ++ias) {
      domArticulated_system *thisArticulated = daeSafeCast<domArticulated_system>(thisKinematicsScene->getInstance_articulated_system_array()[ias]->getUrl().getElement().cast());
      for(size_t ie = 0; ie < thisArticulated->getExtra_array().getCount(); ++ie) {
	domExtraRef pextra = thisArticulated->getExtra_array()[ie];
	if( strcmp(pextra->getType(), "manipulator") == 0 ) {
	  string armname = pextra->getAttribute("name");
	  fprintf(output_fp, "%s-frame-tip ", armname.c_str());
	}
      }
    }
  }
  // sensor
  if ( g_dae->getDatabase()->getElementCount(NULL, "articulated_system", NULL) > 0 ) {
    domArticulated_system *thisArticulated;
    g_dae->getDatabase()->getElement((daeElement**)&thisArticulated, 0, NULL, "articulated_system");
    for(size_t ie = 0; ie < thisArticulated->getExtra_array().getCount(); ++ie) {
      domExtraRef pextra = thisArticulated->getExtra_array()[ie];
      // find element which type is attach_sensor and is attached to thisNode
      if ( strcmp(pextra->getType(), "attach_sensor") == 0 ) {
	fprintf(output_fp, "%s-sensor-coords ", pextra->getName());
      }
    }
  }
  //
  fprintf(output_fp, "))\n");

  fprintf(output_fp, "(defmethod %s-robot\n", robot_name.c_str());
  fprintf(output_fp, "  (:init\n");
  fprintf(output_fp, "   (&rest args)\n");
  fprintf(output_fp, "   (let ()\n");

  // send super :init
  fprintf(output_fp, "     (send-super* :init :name \"%s\" args)\n", robot_name.c_str());
  fprintf(output_fp, "\n");

  // write kinemtaics
  domPhysics_model *thisPhysicsmodel;
  g_dae->getDatabase()->getElement((daeElement**)&thisPhysicsmodel, 0, NULL, "physics_model");
  writeNodes(output_fp, thisNode->getNode_array(), thisPhysicsmodel?(thisPhysicsmodel->getRigid_body_array()):(domRigid_body_Array)NULL, robot_name.c_str());
  fprintf(output_fp, "     (send self :assoc %s)\n", thisNode->getNode_array()[0]->getName());
  
  // write joint
  domKinematics_model *thisKinematics;
  g_dae->getDatabase()->getElement((daeElement**)&thisKinematics, 0, NULL, "kinematics_model");
  if ( thisKinematics ) {
      writeKinematics(output_fp, thisKinematics->getTechnique_common()->getLink_array()[0]->getAttachment_full_array());

      domLinkRef thisLink = thisKinematics->getTechnique_common()->getLink_array()[0];
      if ( thisLink ) {
	  for(unsigned int currentAttachment2=0;currentAttachment2 < (unsigned int)(thisLink->getAttachment_full_array().getCount());currentAttachment2++) {
	      writeJoint(output_fp, thisLink->getAttachment_full_array()[currentAttachment2]->getJoint(),
			 thisLink,
			 thisLink->getAttachment_full_array()[currentAttachment2]->getLink()
		  );
	  }
      }
  }

  // end-coords
  fprintf(output_fp, "\n");
  fprintf(output_fp, "     ;; end coords from openrave manipulater tag\n");
  if ( g_dae->getDatabase()->getElementCount(NULL, "kinematics_scene", NULL) > 0 ) {
    domKinematics_scene *thisKinematicsScene;
    g_dae->getDatabase()->getElement((daeElement**)&thisKinematicsScene, 0, NULL, "kinematics_scene");
    for(size_t ias = 0; ias < thisKinematicsScene->getInstance_articulated_system_array().getCount(); ++ias) {
      domArticulated_system *thisArticulated = daeSafeCast<domArticulated_system>(thisKinematicsScene->getInstance_articulated_system_array()[ias]->getUrl().getElement().cast());
      for(size_t ie = 0; ie < thisArticulated->getExtra_array().getCount(); ++ie) {
	domExtraRef pextra = thisArticulated->getExtra_array()[ie];
	if( strcmp(pextra->getType(), "manipulator") == 0 ) {
	  string armname = pextra->getAttribute("name");
	  daeElement* frame_tip = pextra->getTechnique_array()[0]->getChild("frame_tip");
	  if ( armname != "" ) {
	    domLinkRef pdomlink = daeSafeCast<domLink>(daeSidRef(frame_tip->getAttribute("link"), thisArticulated).resolve().elt);
	    domTranslateRef ptrans = daeSafeCast<domTranslate>(frame_tip->getChild("translate"));
	    domRotateRef prot = daeSafeCast<domRotate>(frame_tip->getChild("rotate"));

	    fprintf(output_fp, "     (setq %s-frame-tip (make-cascoords :coords (send %s :copy-worldcoords) :name :%s-frame-tip))\n", armname.c_str(), pdomlink->getName(), armname.c_str());
	    fprintf(output_fp, "     (send %s-frame-tip :transform (make-coords ", armname.c_str());
	    if ( ptrans ) {
	      fprintf(output_fp, ":pos #f(");
	      for(unsigned int i=0;i<3;i++) { fprintf(output_fp, " %f", 1000*ptrans->getValue()[i]);}
	      fprintf(output_fp, ") ");
	    }
	    if ( prot ) {
	      fprintf(output_fp, ":axis ");
              fprintf(output_fp, "(let ((tmp-axis (float-vector %f %f %f))) (if (eps= (norm tmp-axis) 0.0) (float-vector 1 0 0) tmp-axis))",
                      prot->getValue()[0], prot->getValue()[1], prot->getValue()[2]);
	      fprintf(output_fp, " :angle");
	      fprintf(output_fp, " %f", prot->getValue()[3]*(M_PI/180.0));
	    }
	    fprintf(output_fp, ") :local)\n");
	    fprintf(output_fp, "     (send %s :assoc %s-frame-tip)\n\n",pdomlink->getName(), armname.c_str());
	  }
	  if ( armname == "leftarm" || armname == "rightarm" ) {
	    fprintf(output_fp, "     (setq %carm-end-coords (make-cascoords :coords (send %s-frame-tip :copy-worldcoords)))\n", armname.c_str()[0], armname.c_str());
	    fprintf(output_fp, "     (send %s-frame-tip :assoc %carm-end-coords)\n\n", armname.c_str(), armname.c_str()[0]);
	  }
	}
      }
    }
  }

  fprintf(output_fp, "     ;; end coords from yaml file\n");
  BOOST_FOREACH(link_joint_pair& limb, limbs) {
    string limb_name = limb.first;
    vector<string> link_names = limb.second.first;

    if (link_names.size()>0) {
      string end_coords_parent_name(link_names.back());
      try {
        const YAML::Node& n = doc[limb_name+"-end-coords"]["parent"];
	n >> end_coords_parent_name;
      } catch(YAML::RepresentationException& e) {
      }
      fprintf(output_fp, "     (setq %s-end-coords (make-cascoords :coords (send %s :copy-worldcoords)))\n", limb_name.c_str(), end_coords_parent_name.c_str());
      try {
        const YAML::Node& n = doc[limb_name+"-end-coords"]["translate"];
        double value;
        fprintf(output_fp, "     (send %s-end-coords :translate (float-vector", limb_name.c_str());
        for(unsigned int i=0;i<3;i++) { n[i]>>value; fprintf(output_fp, " %f", 1000*value);}
        fprintf(output_fp, "))\n");
      } catch(YAML::RepresentationException& e) {
      }
      try {
        const YAML::Node& n = doc[limb_name+"-end-coords"]["rotate"];
        double value;
        fprintf(output_fp, "     (send %s-end-coords :rotate", limb_name.c_str());
        for(unsigned int i=3;i<4;i++) { n[i]>>value; fprintf(output_fp, " %f", M_PI/180*value);}
        fprintf(output_fp, " (float-vector");
        for(unsigned int i=0;i<3;i++) { n[i]>>value; fprintf(output_fp, " %f", value);}
        fprintf(output_fp, "))\n");
      } catch(YAML::RepresentationException& e) {
      }
      fprintf(output_fp, "     (send %s :assoc %s-end-coords)\n", end_coords_parent_name.c_str(), limb_name.c_str());
    }
  }
  fprintf(output_fp, "\n");

  // limb name
  fprintf(output_fp, "     ;; limbs\n");
  BOOST_FOREACH(link_joint_pair& limb, limbs) {
    string limb_name = limb.first;
    vector<string> link_names = limb.second.first;
    if ( link_names.size() > 0 ) {
      fprintf(output_fp, "     (setq %s (list", limb_name.c_str());
      for (unsigned int i=0;i<link_names.size();i++) fprintf(output_fp, " %s", link_names[i].c_str()); fprintf(output_fp, "))\n");
      fprintf(output_fp, "\n");
      // find root link by tracing limb's link list
      fprintf(output_fp, "     (setq %s-root-link (labels ((find-parent (l) (if (find (send l :parent) %s) (find-parent (send l :parent)) l))) (find-parent (car %s))))\n", limb_name.c_str(), limb_name.c_str(), limb_name.c_str());
    }
  }
  fprintf(output_fp, "\n");

  fprintf(output_fp, "     ;; links\n");

  domNode *rootNode = thisNode->getNode_array()[0];
  fprintf(output_fp, "     (setq links (list %s", rootNode->getName());
  BOOST_FOREACH(link_joint_pair& limb, limbs) {
    string limb_name = limb.first;
    vector<string> link_names = limb.second.first;
    for (unsigned int i=0;i<link_names.size();i++) fprintf(output_fp, " %s", link_names[i].c_str());
  }
  fprintf(output_fp, "))\n");

  fprintf(output_fp, "     ;; joint-list\n");
  fprintf(output_fp, "     (setq joint-list (list");
  BOOST_FOREACH(link_joint_pair& limb, limbs) {
    vector<string> joint_names = limb.second.second;
    for (unsigned int i=0;i<joint_names.size();i++) fprintf(output_fp, " %s", joint_names[i].c_str());
  }
  fprintf(output_fp, "))\n");
  fprintf(output_fp, "\n");

  // init ending
  fprintf(output_fp, "     ;; init-ending\n");
  fprintf(output_fp, "     (send self :init-ending)\n");
  fprintf(output_fp, "\n");

  // bodies
  fprintf(output_fp, "     ;; overwrite bodies to return draw-things links not (send link :bodies)\n");

  fprintf(output_fp, "     (setq bodies (flatten (mapcar #'(lambda (b) (if (find-method b :bodies) (send b :bodies))) (list");
  for(int currentLink=0;currentLink<(int)(g_dae->getDatabase()->getElementCount(NULL, "link", NULL));currentLink++) {
    domJoint *thisLink;
    g_dae->getDatabase()->getElement((daeElement**)&thisLink, currentLink, NULL, "link");
    fprintf(output_fp, " %s", thisLink->getName());
  }
  fprintf(output_fp, "))))\n\n");

  // when - angle-vector: reset-pose is defined in yaml file
  try {
    doc["angle-vector"]["reset-pose"];
    fprintf(output_fp, "     (send self :reset-pose) ;; :set reset-pose\n\n");
  } catch(YAML::RepresentationException& e) {
  }

  fprintf(output_fp, "     self)) ;; :init\n\n");

  try {
    const YAML::Node& n = doc["angle-vector"];
    if ( n.size() > 0 ) fprintf(output_fp, "    ;; pre-defined pose methods\n");
    for(YAML::Iterator it=n.begin();it!=n.end();it++) {
      string name; it.first() >> name;
      fprintf(output_fp, "    (:%s () (send self :angle-vector (float-vector", name.c_str());
      const YAML::Node& v = it.second();
      for(unsigned int i=0;i<v.size();i++){
        double d; v[i] >> d;
        fprintf(output_fp, " %f", d);
      }
      fprintf(output_fp, ")))\n");
    }
  } catch(YAML::RepresentationException& e) {
  }

  // all joint and link name
  fprintf(output_fp, "\n    ;; all joints\n");
  for(int currentJoint=0;currentJoint<(int)(g_dae->getDatabase()->getElementCount(NULL, "joint", NULL));currentJoint++) {
    domJoint *thisJoint;
    g_dae->getDatabase()->getElement((daeElement**)&thisJoint, currentJoint, NULL, "joint");
    fprintf(output_fp, "    (:%s (&rest args) (forward-message-to %s args))\n", thisJoint->getName(), thisJoint->getName());
  }
  fprintf(output_fp, "\n    ;; all links\n");
  for(int currentLink=0;currentLink<(int)(g_dae->getDatabase()->getElementCount(NULL, "link", NULL));currentLink++) {
    domJoint *thisLink;
    g_dae->getDatabase()->getElement((daeElement**)&thisLink, currentLink, NULL, "link");
    fprintf(output_fp, "    (:%s (&rest args) (forward-message-to %s args))\n", thisLink->getName(), thisLink->getName());
  }
  // add openrave manipulator tip frame
  fprintf(output_fp, "\n    ;; all manipulator\n");
  if ( g_dae->getDatabase()->getElementCount(NULL, "kinematics_scene", NULL) > 0 ) {
    domKinematics_scene *thisKinematicsScene;
    g_dae->getDatabase()->getElement((daeElement**)&thisKinematicsScene, 0, NULL, "kinematics_scene");
    for(size_t ias = 0; ias < thisKinematicsScene->getInstance_articulated_system_array().getCount(); ++ias) {
      domArticulated_system *thisArticulated = daeSafeCast<domArticulated_system>(thisKinematicsScene->getInstance_articulated_system_array()[ias]->getUrl().getElement().cast());
      for(size_t ie = 0; ie < thisArticulated->getExtra_array().getCount(); ++ie) {
	domExtraRef pextra = thisArticulated->getExtra_array()[ie];
	if( strcmp(pextra->getType(), "manipulator") == 0 ) {
	  string armname = pextra->getAttribute("name");
	  fprintf(output_fp, "    (:%s-frame-tip (&rest args) (forward-message-to %s-frame-tip args))\n", armname.c_str(), armname.c_str());
	}
      }
    }
  }
  fprintf(output_fp, "\n    ;; user-defined joint\n");
  for(vector<pair<string, string> >::iterator it=g_all_link_names.begin();it!=g_all_link_names.end();it++){
    fprintf(output_fp, "    (:%s (&rest args) (forward-message-to %s args))\n", it->second.c_str(), it->first.c_str());
  }
  // sensor
  fprintf(output_fp, "\n    ;; attach_sensor\n");
  if ( g_dae->getDatabase()->getElementCount(NULL, "articulated_system", NULL) > 0 ) {
    domArticulated_system *thisArticulated;
    g_dae->getDatabase()->getElement((daeElement**)&thisArticulated, 0, NULL, "articulated_system");
    std::vector<std::string> fsensor_list;
    for(size_t ie = 0; ie < thisArticulated->getExtra_array().getCount(); ++ie) {
      domExtraRef pextra = thisArticulated->getExtra_array()[ie];
      // find element which type is attach_sensor and is attached to thisNode
      if ( strcmp(pextra->getType(), "attach_sensor") == 0 ) {
	fprintf(output_fp, "    (:%s (&rest args) (forward-message-to %s-sensor-coords args))\n", pextra->getName(), pextra->getName());
        if (getSensorType(pextra) == "base_force6d" ) {
          fsensor_list.push_back(string(pextra->getName()));
        }
      }
    }
    fprintf(output_fp, "    (:force-sensors (&rest args) (forward-message-to-all (list ");
    for (size_t i = 0; i < fsensor_list.size(); i++) {
      fprintf(output_fp, "(send self :%s) ", fsensor_list[i].c_str());
    }
    fprintf(output_fp, ") args))\n");
  }
  fprintf(output_fp, "  )\n\n");

  writeGeometry(output_fp, g_dae->getDatabase(), robot_name.c_str());

  fprintf(output_fp, "\n\n(provide :%s \"%s/%s\")\n\n", robot_name.c_str(), get_current_dir_name(), output_filename);

  ifstream fin2(yaml_filename);
  if (fin2.fail()) {
    fprintf(stderr, ";; generate lisp code without yaml file\n");
    exit(-1);
  }

  fflush(output_fp);
  return 0;
}

