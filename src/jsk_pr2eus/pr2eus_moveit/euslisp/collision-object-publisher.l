(ros::load-ros-manifest "moveit_msgs")

(defclass collision-object-publisher
  :super propertied-object
  :slots (object-list attached-object-list
          servicename scene-srv))

(defmethod collision-object-publisher
  (:init (&key (service-name "apply_planning_scene")
               (scene-service "get_planning_scene"))
   (unless (ros::ok) (ros::roseus "publish_collision_eusobj"))
   (setq servicename service-name
         scene-srv scene-service)
   (ros::wait-for-service servicename)
   (setq object-list (make-hash-table))
   (setq attached-object-list (make-hash-table))
   self)
  (:add-object (obj &rest args)
   (let ((objmsg (send* self :make-object obj args))
         (req (instance moveit_msgs::ApplyPlanningSceneRequest :init)))
     (setf (gethash obj object-list) objmsg)
     (send req :scene :world :collision_objects (list objmsg))
     (send req :scene :is_diff t)
     (ros::service-call servicename req)
     obj))
  (:add-attached-object (obj link-name &rest args
                             &key (touch-links) (weight 0) (detach-posture)
                             &allow-other-keys)
   (let ((objmsg (send* self :make-object obj args))
         (attached-objmsg (instance moveit_msgs::AttachedCollisionObject :init))
         (req (instance moveit_msgs::ApplyPlanningSceneRequest :init)))
     (send attached-objmsg :link_name link-name)
     (send attached-objmsg :object objmsg)
     (send attached-objmsg :touch_links touch-links)
     (if detach-posture
       (send attached-objmsg :detach_posture detach-posture))
     (send attached-objmsg :weight weight)
     (setf (gethash obj object-list) objmsg)
     (setf (gethash obj attached-object-list) attached-objmsg)
     (send req :scene :robot_state :attached_collision_objects (list attached-objmsg))
     (send req :scene :is_diff t)
     (send req :scene :robot_state :is_diff t)
     (ros::service-call servicename req)
     obj))
  (:make-object
   (obj &key (frame-id "base_footprint") (relative-pose)
        (object-id (string (gensym "COLOBJ"))) &allow-other-keys)
   (let ((msg (gethash obj object-list)))
     (when msg (return-from :make-object msg)))
   (let ((colobj (instance moveit_msgs::CollisionObject :init :id object-id))
         geom-lst pose-lst)

     (send colobj :operation moveit_msgs::CollisionObject::*ADD*)

     (send colobj :header :frame_id frame-id)
     (send colobj :header :stamp (ros::time-now))

     (cond
      ((and (derivedp obj body) (eq (car (send obj :body-type)) :cylinder))
       (let ((geom (send colobj :primitives))
             (pose (send colobj :primitive_poses)))
         (send colobj :primitives
               (append geom
                       (list
                        (instance shape_msgs::SolidPrimitive
                                  :init :type shape_msgs::SolidPrimitive::*CYLINDER*
                                  :dimensions (float-vector
                                               (/ (radius-of-cylinder obj) 1000.0)
                                               (/ (height-of-cylinder obj) 1000.0))))))
         (send colobj :primitive_poses
               (append pose
                       (list
                        (ros::coords->tf-pose (if relative-pose relative-pose (send obj :worldcoords))))))))
      ((and (derivedp obj body) (eq (car (send obj :body-type)) :cube))
       (let ((geom (send colobj :primitives))
             (pose (send colobj :primitive_poses)))
         (send colobj :primitives
               (append geom
                       (list
                        (instance shape_msgs::SolidPrimitive
                                  :init :type shape_msgs::SolidPrimitive::*BOX*
                                  :dimensions (float-vector
                                               (/ (elt (send obj :body-type) 1) 1000.0)
                                               (/ (elt (send obj :body-type) 2) 1000.0)
                                               (/ (elt (send obj :body-type) 3) 1000.0))))))
         (send colobj :primitive_poses
               (append pose
                       (list
                        (ros::coords->tf-pose (if relative-pose relative-pose (send obj :worldcoords))))))))
      ((and (derivedp obj body) (eq (car (send obj :body-type)) :gdome))
       (let ((geom (send colobj :primitives))
             (pose (send colobj :primitive_poses)))
         (send colobj :primitives
               (append geom
                       (list
                        (instance shape_msgs::SolidPrimitive
                                  :init :type shape_msgs::SolidPrimitive::*SPHERE*
                                  :dimensions (float-vector
                                               (/ (radius-of-sphere obj) 1000.0))))))
         (send colobj :primitive_poses
               (append pose
                       (list
                        (ros::coords->tf-pose (if relative-pose relative-pose (send obj :worldcoords))))))))
      ((find-method obj :faces)
       (let ((org-cds (send obj :copy-worldcoords)))
         (send obj :reset-coords)
         (send obj :worldcoords)
         (let ((fs (body-to-faces obj))
               (geom (instance shape_msgs::mesh :init))
               idx-lst vertices (cntr 0))
           (dolist (f (send fs :faces))
             (let* ((vs (send f :vertices))
                    (v0 (car vs))
                    (v1 (cadr vs))
                    (v2 (caddr vs))
                    (p0
                     (instance geometry_msgs::Point :init
                               :x (/ (elt v0 0) 1000.0)
                               :y (/ (elt v0 1) 1000.0)
                               :z (/ (elt v0 2) 1000.0)))
                    (p1
                     (instance geometry_msgs::Point :init
                               :x (/ (elt v1 0) 1000.0)
                               :y (/ (elt v1 1) 1000.0)
                               :z (/ (elt v1 2) 1000.0)))
                    (p2
                     (instance geometry_msgs::Point :init
                               :x (/ (elt v2 0) 1000.0)
                               :y (/ (elt v2 1) 1000.0)
                               :z (/ (elt v2 2) 1000.0))))
               (push p0 vertices)
               (push p1 vertices)
               (push p2 vertices)
               (push (integer-vector cntr (+ cntr 1) (+ cntr 2)) idx-lst)
               (incf cntr 3)
               ))
           (send geom :triangles
                 (mapcar #'(lambda (idx)
                             (instance shape_msgs::MeshTriangle :init :vertex_indices idx))
                         (nreverse idx-lst)))
           (send geom :vertices (nreverse vertices))
           (let ((m (send colobj :meshes))
                 (mp (send colobj :mesh_poses)))
             (send colobj :meshes (append m (list geom)))
             (send colobj :mesh_poses (append mp (list
                                                  (ros::coords->tf-pose (if relative-pose
                                                                            relative-pose org-cds))))))
           )
         (send obj :transform org-cds)
         (send obj :worldcoords)))
      ((derivedp obj pointcloud)
       ;; making voxel grid ...
       )
      ((find-method obj :vertices)
       ;; making bounding box
       ;; (send obj :vertices)
       )
      (t
       (warn ";; not supported object type~%")
       (return-from :make-object)
       ))
     colobj))
  (:clear-all ()
   (dolist (obj (send object-list :list-keys))
     (send self :delete-object obj))
   (setq object-list (make-hash-table))
   t)
  (:clear-attached-all ()
   (dolist (obj (send attached-object-list :list-keys))
     (send self :delete-attached-object obj))
   (setq attached-object-list (make-hash-table))
   t)
  (:wipe-all ()
    (send self :wipe-all-attached-object)
    (send self :wipe-all-object))
  (:wipe-all-object ()
   (let ((scene (get-planning-scene :scene-service scene-srv
                                    :components (+ moveit_msgs::PlanningSceneComponents::*ROBOT_STATE_ATTACHED_OBJECTS*
                                                   moveit_msgs::PlanningSceneComponents::*WORLD_OBJECT_NAMES*
                                                   moveit_msgs::PlanningSceneComponents::*WORLD_OBJECT_GEOMETRY*
                                                   moveit_msgs::PlanningSceneComponents::*OCTOMAP*)))
         (req (instance moveit_msgs::ApplyPlanningSceneRequest :init)))
     (when scene
       (send req :scene :world :collision_objects
             (mapcar #'(lambda (msg)
                         (send msg :header :stamp (ros::time-now))
                         (send msg :operation moveit_msgs::CollisionObject::*REMOVE*)
                         msg)
                     (send scene :world :collision_objects)))
       (send req :scene :is_diff t)
       (ros::service-call servicename req)
       (setq object-list (make-hash-table))
       t)))
  (:wipe-all-attached-object ()
   (let ((scene (get-planning-scene :scene-service scene-srv
                                    :components (+ moveit_msgs::PlanningSceneComponents::*ROBOT_STATE_ATTACHED_OBJECTS*
                                                   moveit_msgs::PlanningSceneComponents::*WORLD_OBJECT_NAMES*
                                                   moveit_msgs::PlanningSceneComponents::*WORLD_OBJECT_GEOMETRY*
                                                   moveit_msgs::PlanningSceneComponents::*OCTOMAP*)))
         (req (instance moveit_msgs::ApplyPlanningSceneRequest :init)))
     (when scene
       (send req :scene :robot_state :attached_collision_objects
             (mapcar #'(lambda (msg)
                         (send msg :object :header :stamp (ros::time-now))
                         (send msg :object :operation moveit_msgs::CollisionObject::*REMOVE*)
                         msg)
                     (send scene :robot_state :attached_collision_objects)))
       (send req :scene :is_diff t)
       (send req :scene :robot_state :is_diff t)
       (ros::service-call servicename req)
       (setq attached-object-list (make-hash-table))
       t)))
  (:delete-object (obj)
   (let ((msg (gethash obj object-list))
         (req (instance moveit_msgs::ApplyPlanningSceneRequest :init)))
     (unless msg (return-from :delete-object))
     (send msg :header :stamp (ros::time-now))
     (send msg :operation moveit_msgs::CollisionObject::*REMOVE*)
     (remhash obj object-list)
     (send req :scene :world :collision_objects (list msg))
     (send req :scene :is_diff t)
     (ros::service-call servicename req)
     obj))
  (:delete-attached-object (obj)
   (let ((msg (gethash obj attached-object-list))
         (req (instance moveit_msgs::ApplyPlanningSceneRequest :init)))
     (unless msg (return-from :delete-attached-object))
     (send msg :object :header :stamp (ros::time-now))
     (send msg :object :operation moveit_msgs::CollisionObject::*REMOVE*)
     (remhash obj attached-object-list)
     (send req :scene :robot_state :attached_collision_objects (list msg))
     (send req :scene :is_diff t)
     (send req :scene :robot_state :is_diff t)
     (ros::service-call servicename req)
     obj))
  )

(defun get-planning-scene (&key (scene-service "get_planning_scene")
                                (components 1023))
  ;;moveit_msgs::PlanningSceneComponents::*SCENE_SETTINGS*
  ;;moveit_msgs::PlanningSceneComponents::*ROBOT_STATE*
  ;;moveit_msgs::PlanningSceneComponents::*ROBOT_STATE_ATTACHED_OBJECTS*
  ;;moveit_msgs::PlanningSceneComponents::*WORLD_OBJECT_NAMES*
  ;;moveit_msgs::PlanningSceneComponents::*WORLD_OBJECT_GEOMETRY*
  ;;moveit_msgs::PlanningSceneComponents::*OCTOMAP*
  ;;moveit_msgs::PlanningSceneComponents::*TRANSFORMS*
  ;;moveit_msgs::PlanningSceneComponents::*ALLOWED_COLLISION_MATRIX*
  ;;moveit_msgs::PlanningSceneComponents::*LINK_PADDING_AND_SCALING*
  ;;moveit_msgs::PlanningSceneComponents::*OBJECT_COLORS*
  (let ((req (instance moveit_msgs::GetPlanningSceneRequest :init))
        ret)
    (send req :components :components components)
    (setq ret (ros::service-call scene-service req))
    (if ret (send ret :scene))
    ))

(defun make-collision-map (vox-center-3dp-lst &key (stamp (ros::time-now))
                                              (frame "base_footprint") (grid-size 30))
  (let* ((hd (instance std_msgs::header :init :frame_id frame :stamp stamp))
         (cmap (instance moveit_msgs::CollisionMap :init :header hd))
         lst (meter-grid-size (* (/ grid-size 2) 0.001)))
    ;;
    (dolist (p vox-center-3dp-lst)
      (let ((bx
             (instance moveit_msgs::OrientedBoundingBox :init)))
        (send bx :pose :position :x (* 0.001 (elt p 0)))
        (send bx :pose :position :y (* 0.001 (elt p 1)))
        (send bx :pose :position :z (* 0.001 (elt p 2)))
        (send bx :pose :orientation :w 1)
        (send bx :extents :x meter-grid-size)
        (send bx :extents :y meter-grid-size)
        (send bx :extents :z meter-grid-size)
        (push bx lst)))
    (send cmap :boxes (nreverse lst))
    cmap))

(defun publish-collision-object
  (obj robot link-name &optional (collision-publisher *co*))
  (let ((cds (send (send robot
                         (intern (format nil "~A_LK" (string-upcase link-name))
                                 *keyword-package*) :worldcoords)
                   :transformation (send obj :worldcoords))))
    (send collision-publisher :add-object obj
          :frame-id link-name :relative-pose cds)
    obj))
