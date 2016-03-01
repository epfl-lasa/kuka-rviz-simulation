;;; Copyright (c) 2014, Jannik Buckelo <jannikbu@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :urdf-management-tutorial)

(defparameter *l-arm-joint-names* '("l_upper_arm_roll_joint"
                                    "l_shoulder_pan_joint"
                                    "l_shoulder_lift_joint"
                                    "l_forearm_roll_joint"
                                    "l_elbow_flex_joint"
                                    "l_wrist_flex_joint"
                                    "l_wrist_roll_joint"))

(defparameter *l-arm-goal-state* '("l_upper_arm_roll_joint" (:position 0.593)
                                   "l_shoulder_pan_joint" (:position 1.265)
                                   "l_shoulder_lift_joint" (:position 0.964)
                                   "l_forearm_roll_joint" (:position 0.524)
                                   "l_elbow_flex_joint" (:position -2.1)
                                   "l_wrist_flex_joint" (:position -0.067)
                                   "l_wrist_roll_joint" (:position 4.419)))

(defun make-joint-state-list (joint-names joint-positions)
  "Takes a list of `joint-names` and a list of `joint-positions`, and
returns a list joint-states. Input lists need to be of equal length."
  (mapcar (lambda (name position)
            (cl-robot-models:make-joint-state :name name :position position))
          joint-names joint-positions))

(defun move-to-grasp-position ()
  (roslisp:start-ros-node (format nil "urdf_management_tutorial_~a" (gensym)))
  (let((l-arm-grasping-configuration (cl-robot-models:make-robot-state
                                      "Raphael" "PR2"
                                      (make-joint-state-list
                                       *l-arm-joint-names*
                                       '(0.593 1.265 0.964 0.524 -2.1 -0.067 4.419))))
       (pr2-controller (make-pr2-arm-position-controller-handle 
                        "/l_arm_controller/joint_trajectory_action" 
                        *l-arm-joint-names*)))
    (roslisp:ros-info "urdf_management_tutorial" "Waiting for action sever.")
    (actionlib-lisp:wait-for-server (cram-pr2-controllers::client pr2-controller))
    (roslisp:ros-info "urdf_management_tutorial" "Moving arm.")
    (move-arm pr2-controller *l-arm-goal-state* 4.0))
  (roslisp:shutdown-ros-node))
