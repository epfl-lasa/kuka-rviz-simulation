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

(defun add-spatula ()
  (alter-urdf 1 "<link name=\"spatula\"><visual><origin rpy=\"0 0 0 \" xyz=\"0 0 0\" /><geometry><mesh filename=\"package://urdf_management_tutorial/meshes/kitchen/hand-tools/edeka_spatula1.dae\" /></geometry></visual></link><joint name=\"joint_spatula\" type=\"fixed\"><parent link=\"l_gripper_r_finger_tip_link\" /><child link=\"spatula\" /><origin rpy=\"-1.57 0 0.5\" xyz=\"0.22 0 0\"/></joint>" nil))

(defun remove-spatula ()
  (alter-urdf 2 "" (vector "joint_spatula" "spatula")))

(defun remove-left-gripper () 
  (alter-urdf 2 "" (vector "l_gripper_l_finger_link"          ;;links
                           "l_gripper_l_finger_tip_frame"
                           "l_gripper_l_finger_tip_link"
                           "l_gripper_led_frame"
                           "l_gripper_motor_accelerometer_link"
                           "l_gripper_motor_screw_link"
                           "l_gripper_motor_slider_link"
                           "l_gripper_palm_link"
                           "l_gripper_r_finger_link"
                           "l_gripper_r_finger_tip_link"
                           "l_gripper_tool_frame"
                           "l_gripper_l_finger_joint"          ;;joints
                           "l_gripper_r_finger_joint"
                           "l_gripper_l_finger_tip_joint"
                           "l_gripper_r_finger_tip_joint"
                           "l_gripper_palm_joint"
                           "l_gripper_led_joint"
                           "l_gripper_motor_accelerometer_joint"
                           "l_gripper_tool_joint"
                           "l_gripper_motor_slider_joint"
                           "l_gripper_motor_screw_joint"
                           "l_gripper_joint")))

(defun alter-urdf (action-id add remove)
  (with-ros-node ("urdf_management_tutorial")
    (ros-info "urdf_mangement_tutorial" "Action: ~a" action-id)
    (ros-info "urdf_mangement_tutorial" "xml_elements_to_add: ~a" add)
    (ros-info "urdf_mangement_tutorial" "element_names_to_remove: ~a" remove)
    (roslisp:call-service "alter_urdf" 'iai_urdf_msgs-srv:alterurdf 
                          :action action-id
                          :xml_elements_to_add add
                          :element_names_to_remove remove)))